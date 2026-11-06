
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <FastLED.h>
#include <DNSServer.h>

// =====================================================
// 1. WS2812 SETTINGS
// =====================================================

#define LED_PIN          5
#define NUM_LEDS         8
#define LED_TYPE         WS2812B
#define COLOR_ORDER      GRB
#define LED_BRIGHTNESS   80

CRGB leds[NUM_LEDS];

// =====================================================
// 2. ESP32 ACCESS POINT SETTINGS
// =====================================================

const char* AP_SSID = "ESP32_AP_MODE";
const char* AP_PASSWORD = "12345678";

IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// =====================================================
// 3. OBJECTS
// =====================================================

WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

// =====================================================
// 4. WIFI CREDENTIALS AND CONNECTION STATE
// =====================================================

String savedSSID = "";
String savedPassword = "";

String pendingSSID = "";
String pendingPassword = "";

bool wifiAttemptInProgress = false;
bool userAttempt = false;
bool userAttemptFailed = false;

unsigned long wifiAttemptStart = 0;

const unsigned long BOOT_WIFI_TIMEOUT_MS = 10000;
const unsigned long USER_WIFI_TIMEOUT_MS = 20000;

// =====================================================
// 5. LED STATUS
// =====================================================

enum LED_STATUS
{
    LED_AP_MODE,
    LED_PHONE_CONNECTED,
    LED_WIFI_CONNECTED,
    LED_WIFI_FAILED
};

LED_STATUS ledStatus = LED_AP_MODE;

// =====================================================
// 6. STEADY LED COLOR
// =====================================================

void showLEDColor(CRGB color)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color;
    }

    FastLED.show();
}

void setLEDStatus(LED_STATUS newStatus)
{
    if (ledStatus == newStatus)
    {
        return;
    }

    ledStatus = newStatus;

    switch (ledStatus)
    {
        case LED_AP_MODE:
            Serial.println("STATUS: Access point active; awaiting client.");
            showLEDColor(CRGB(148, 0, 211));
            break;

        case LED_PHONE_CONNECTED:
            Serial.println("STATUS: Device Connected in AP Mode");
            showLEDColor(CRGB(0, 0, 255));
            break;

        case LED_WIFI_CONNECTED:
            Serial.println("STATUS: Router connection established.");
            showLEDColor(CRGB(0, 255, 0));
            break;

        case LED_WIFI_FAILED:
            Serial.println("STATUS: Router connection failed.");
            showLEDColor(CRGB(255, 0, 0));
            break;
    }
}

// =====================================================
// 7. LOAD SAVED CREDENTIALS
// =====================================================

void loadWiFiCredentials()
{
    preferences.begin("wifi", true);

    savedSSID = preferences.getString("ssid", "");
    savedPassword = preferences.getString("password", "");

    preferences.end();

    if (savedSSID.length() > 0)
    {
        Serial.print("Saved network found: ");
        Serial.println(savedSSID);
    }
    else
    {
        Serial.println("No saved network available.");
    }
}

// =====================================================
// 8. SAVE CREDENTIALS AFTER SUCCESS
// =====================================================

void saveWiFiCredentials(String ssid, String password)
{
    preferences.begin("wifi", false);

    preferences.putString("ssid", ssid);
    preferences.putString("password", password);

    preferences.end();

    savedSSID = ssid;
    savedPassword = password;

    Serial.println("STATUS: Network credentials saved.");
}

// =====================================================
// 9. START WIFI CONNECTION
// =====================================================

void startWiFiConnection(String ssid,
                         String password,
                         bool initiatedByUser)
{
    Serial.println();
    Serial.println("================================");
    Serial.println("STATUS: Router Connection Started.");
    Serial.print("Target network: ");
    Serial.println(ssid);
    Serial.println("================================");

    pendingSSID = ssid;
    pendingPassword = password;

    userAttempt = initiatedByUser;
    userAttemptFailed = false;

    wifiAttemptInProgress = false;

    // Keep the ESP32 access point available.
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);

    // Stop any previous station connection attempt.
    WiFi.disconnect(false, false);
    delay(250);

    wifiAttemptStart = millis();
    wifiAttemptInProgress = true;

    // Do not force a LED color here.
    // updateLEDStatus() determines the color from the
    // actual AP client count and router connection state.
    WiFi.begin(
        pendingSSID.c_str(),
        pendingPassword.c_str()
    );
}

// =====================================================
// 10. CHECK WIFI CONNECTION RESULT
// =====================================================

void checkWiFiConnection()
{
    if (!wifiAttemptInProgress)
    {
        return;
    }

    // SUCCESS
    if (WiFi.status() == WL_CONNECTED)
    {
        wifiAttemptInProgress = false;
        userAttemptFailed = false;

        Serial.println();
        Serial.println("STATUS: Router connection successful.");

        Serial.print("Connected network: ");
        Serial.println(WiFi.SSID());

        Serial.print("Device IP address: ");
        Serial.println(WiFi.localIP());

        Serial.print("Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");

        // Save only after a successful connection.
        saveWiFiCredentials(
            pendingSSID,
            pendingPassword
        );

        return;
    }

    unsigned long timeout = userAttempt
        ? USER_WIFI_TIMEOUT_MS
        : BOOT_WIFI_TIMEOUT_MS;

    // TIMEOUT
    if (millis() - wifiAttemptStart >= timeout)
    {
        wifiAttemptInProgress = false;

        // Stop station activity but keep the access point.
        WiFi.disconnect(false, false);

        Serial.println();

        if (userAttempt)
        {
            userAttemptFailed = true;

            Serial.println("STATUS: Router connection failed.");
            Serial.println("ACTION: Verify the network name and password.");
        }
        else
        {
            // Startup failure is not an HTML configuration failure.
            userAttemptFailed = false;

            Serial.println("STATUS: Saved network unavailable.");
            Serial.println("STATUS: Access Point Connection Available.");
        }
    }
}

// =====================================================
// 11. UPDATE LED FROM ACTUAL CONNECTION STATE
// =====================================================

void updateLEDStatus()
{
    // 1. Router connection successful: green.
    if (WiFi.status() == WL_CONNECTED)
    {
        setLEDStatus(LED_WIFI_CONNECTED);
        return;
    }

    // 2. Check whether a phone or another device is
    // connected to the ESP32 access point.
    int deviceCount = WiFi.softAPgetStationNum();

    // 3. If HTML credentials failed, show red while
    // the configuration client remains connected.
    if (userAttemptFailed && deviceCount > 0)
    {
        setLEDStatus(LED_WIFI_FAILED);
        return;
    }

    // 4. A connected client means blue, including while
    // HTML credentials are being checked.
    if (deviceCount > 0)
    {
        setLEDStatus(LED_PHONE_CONNECTED);
        return;
    }

    // 5. No connected client and no router connection:
    // return to AP-ready state.
    setLEDStatus(LED_AP_MODE);
}

// =====================================================
// 12. HTML CONFIGURATION PAGE
// =====================================================

String getHTML()
{
    String html = "";

    html += "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>ESP32 Wi-Fi Configuration</title>";

    html += "<style>";
    html += "body{font-family:Arial;background:#eee;text-align:center;padding:20px;}";
    html += ".box{max-width:400px;margin:auto;background:white;padding:24px;";
    html += "border-radius:15px;box-shadow:0 0 10px #bbb;}";
    html += "input{width:100%;padding:12px;margin:8px 0;font-size:16px;";
    html += "box-sizing:border-box;}";
    html += "button{padding:12px 24px;background:#007bff;color:white;";
    html += "border:0;border-radius:8px;font-size:16px;cursor:pointer;}";
    html += ".password-wrap{position:relative;width:100%;}";
    html += ".password-wrap input{padding-right:48px;}";
    html += ".eye{position:absolute;right:3px;top:13px;width:44px;";
    html += "padding:7px 0;background:transparent;color:#333;border:none;}";
    html += ".eye svg{width:23px;height:23px;vertical-align:middle;}";
    html += ".status{background:#eee;border-radius:10px;padding:12px;margin:15px 0;}";
    html += "</style></head><body>";

    html += "<div class='box'>";
    html += "<h2>ESP32 Wi-Fi Configuration</h2>";
    html += "<div class='status' id='connectionStatus'>";

    if (WiFi.status() == WL_CONNECTED)
    {
        html += "<h3 style='color:green'>Wi-Fi Connected</h3>";
        html += "<p>Network: ";
        html += WiFi.SSID();
        html += "</p><p>Router IP: ";
        html += WiFi.localIP().toString();
        html += "</p>";
    }
    else if (userAttemptFailed)
    {
        html += "<h3 style='color:red'>Connection Failed</h3>";
        html += "<p>Check the Wi-Fi name and password.</p>";
    }
    else if (wifiAttemptInProgress)
    {
        html += "<h3>Connecting to Wi-Fi...</h3>";
        html += "<p>Please wait.</p>";
    }
    else
    {
        html += "<h3>ESP32 Ready</h3>";
        html += "<p>Enter your router Wi-Fi details.</p>";
    }

    html += "</div>";

    // SSID and password form.
    html += "<form action='/save' method='POST'>";

    html += "<input type='text' name='ssid' ";
    html += "placeholder='Enter Wi-Fi name' required>";

    // Password input and eye icon.
    html += "<div class='password-wrap'>";

    html += "<input type='password' id='wifiPassword' name='password' ";
    html += "placeholder='Enter Wi-Fi password' required>";

    html += "<button class='eye' type='button' id='eyeButton' ";
    html += "onclick='togglePassword()' aria-label='Show password'>";

    html += "<svg id='eyeIcon' viewBox='0 0 24 24' fill='none' ";
    html += "stroke='currentColor' stroke-width='2' ";
    html += "stroke-linecap='round' stroke-linejoin='round'>";

    html += "<path d='M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7-10-7-10-7z'/>";
    html += "<circle cx='12' cy='12' r='3'/>";
    html += "</svg></button></div>";

    html += "<br><br><button type='submit'>CONNECT</button>";
    html += "</form>";

    // Password show/hide and live status.
    html += "<script>";

    html += "function togglePassword(){";
    html += "var p=document.getElementById('wifiPassword');";
    html += "var b=document.getElementById('eyeButton');";
    html += "var i=document.getElementById('eyeIcon');";

    html += "if(p.type==='password'){";
    html += "p.type='text';";
    html += "b.setAttribute('aria-label','Hide password');";
    html += "i.innerHTML='<path d=\"M3 3l18 18\"/>' +";
    html += "'<path d=\"M6.6 6.6C3.7 8.2 2 12 2 12s3.5 7 10 7\"/>' +";
    html += "'<path d=\"M14.1 14.1A3 3 0 0 1 9.9 9.9\"/>' +";
    html += "'<path d=\"M17.4 17.4C20.3 15.8 22 12 22 12s-1.2-2.4-3.4-4.3\"/>';";
    html += "}else{";
    html += "p.type='password';";
    html += "b.setAttribute('aria-label','Show password');";
    html += "i.innerHTML='<path d=\"M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7-10-7-10-7z\"/>' +";
    html += "'<circle cx=\"12\" cy=\"12\" r=\"3\"/>';";
    html += "}}";

    // Update the connection result without reloading the page.
    html += "setInterval(async function(){";
    html += "try{";
    html += "const r=await fetch('/status',{cache:'no-store'});";
    html += "const d=await r.json();";
    html += "const s=document.getElementById('connectionStatus');";

    html += "if(d.state==='connected'){";
    html += "s.innerHTML='<h3 style=\"color:green\">Wi-Fi Connected</h3>' +";
    html += "'<p>Network: '+d.ssid+'</p><p>Router IP: '+d.ip+'</p>';";
    html += "}else if(d.state==='failed'){";
    html += "s.innerHTML='<h3 style=\"color:red\">Connection Failed</h3>' +";
    html += "'<p>Check the Wi-Fi name and password.</p>';";
    html += "}else if(d.state==='connecting'){";
    html += "s.innerHTML='<h3>Connecting to Wi-Fi...</h3><p>Please wait.</p>';";
    html += "}";
    html += "}catch(e){}";
    html += "},1000);";

    html += "</script>";
    html += "</div></body></html>";

    return html;
}

// =====================================================
// 13. HOME PAGE
// =====================================================

void handleRoot()
{
    server.send(200, "text/html", getHTML());
}

// =====================================================
// 14. SAVE FORM AND START CONNECTION
// =====================================================

void handleSave()
{
    String newSSID = server.arg("ssid");
    String newPassword = server.arg("password");

    newSSID.trim();

    if (newSSID.length() == 0)
    {
        server.send(400, "text/plain", "Wi-Fi name is required.");
        return;
    }

    Serial.println();
    Serial.println("STATUS: New network configuration received.");
    Serial.print("Target network: ");
    Serial.println(newSSID);

    // Start checking the submitted credentials.
    // LED logic remains blue if a phone is connected.
    startWiFiConnection(newSSID, newPassword, true);

    // Return a result page immediately.
    String html = "";

    html += "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>Wi-Fi Connection Status</title></head>";

    html += "<body style='font-family:Arial;text-align:center;padding:25px;'>";
    html += "<h2>Wi-Fi Connection Status</h2>";
    html += "<p id='result'>Checking network credentials...</p>";
    html += "<p><a href='/'>Back to configuration</a></p>";

    html += "<script>";

    html += "async function check(){";
    html += "try{";
    html += "const r=await fetch('/status',{cache:'no-store'});";
    html += "const d=await r.json();";
    html += "const e=document.getElementById('result');";

    html += "if(d.state==='connected'){";
    html += "e.innerHTML='<h3 style=\"color:green\">Connection Successful</h3>' +";
    html += "'<p>Network: '+d.ssid+'</p><p>Device IP: '+d.ip+'</p>';";
    html += "}else if(d.state==='failed'){";
    html += "e.innerHTML='<h3 style=\"color:red\">Connection Failed</h3>' +";
    html += "'<p>Verify the network name and password.</p>';";
    html += "}else{";
    html += "e.innerText='Checking network connection...';";
    html += "setTimeout(check,1000);";
    html += "}";
    html += "}catch(e){setTimeout(check,1000);}";
    html += "}";

    html += "setTimeout(check,500);";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
}

// =====================================================
// 15. CONNECTION STATUS API
// =====================================================

void handleStatus()
{
    String json = "{";

    if (WiFi.status() == WL_CONNECTED)
    {
        json += "\"state\":\"connected\",";
        json += "\"ssid\":\"";
        json += WiFi.SSID();
        json += "\",";
        json += "\"ip\":\"";
        json += WiFi.localIP().toString();
        json += "\"";
    }
    else if (wifiAttemptInProgress)
    {
        json += "\"state\":\"connecting\"";
    }
    else if (userAttemptFailed)
    {
        json += "\"state\":\"failed\"";
    }
    else
    {
        json += "\"state\":\"idle\"";
    }

    json += "}";

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

// =====================================================
// 16. UNKNOWN URL REDIRECT
// =====================================================

void handleNotFound()
{
    server.sendHeader(
        "Location",
        "http://192.168.4.1",
        true
    );

    server.send(302, "text/plain", "");
}

// =====================================================
// 17. SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("================================");
    Serial.println("DEVICE INITIALIZATION STARTED");
    Serial.println("================================");

    // Initialize WS2812.
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(
        leds,
        NUM_LEDS
    );

    FastLED.setBrightness(LED_BRIGHTNESS);

    // Initial state: AP ready.
    ledStatus = LED_AP_MODE;
    showLEDColor(CRGB(148, 0, 211));

    // Load stored credentials.
    loadWiFiCredentials();

    // Start AP + station mode.
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);

    WiFi.softAPConfig(
        apIP,
        gateway,
        subnet
    );

    // Start the configuration access point.
    WiFi.softAP(
        AP_SSID,
        AP_PASSWORD,
        1,
        0,
        4
    );

    // Start captive DNS.
    dnsServer.start(53, "*", apIP);

    // Configure web server.
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/status", HTTP_GET, handleStatus);
    server.onNotFound(handleNotFound);

    server.begin();

    Serial.println("STATUS: Access point is active.");
    Serial.print("Network name: ");
    Serial.println(AP_SSID);
    Serial.print("Configuration address: http://");
    Serial.println(WiFi.softAPIP());

    // Try the saved network for up to 10 seconds.
    // During this attempt, a connected phone still
    // gets the blue client-connected indication.
    if (savedSSID.length() > 0)
    {
        startWiFiConnection(
            savedSSID,
            savedPassword,
            false
        );
    }
    else
    {
        Serial.println("STATUS: No saved network; ready for configuration.");
    }
}

// =====================================================
// 18. LOOP
// =====================================================

void loop()
{
    dnsServer.processNextRequest();
    server.handleClient();

    // Process connection success or timeout.
    checkWiFiConnection();

    // Select the LED based on current connections.
    updateLEDStatus();
}
