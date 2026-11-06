1 – Basic Wi-Fi Configuration and RGB LED Control
Basic Embedded Wi-Fi Development


Objective: Develop the basic ESP32 Wi-Fi configuration system with an Access Point, HTML configuration page, Wi-Fi connection handling, credential storage, and RGB LED status indication.

This is the foundation for the advanced functionality developed in the subsequent tasks.
Components and technologies used
Component / Technology	Purpose
ESP32-WROOM-32	Main controller
WS2812B RGB LED	Visual status indication
Wi-Fi AP + STA	Wireless communication
WebServer	HTML configuration
DNSServer	Captive portal
Preferences / NVS	Credential storage
FastLED	RGB LED control
WiFi.begin()	Router connection
millis()	Connection timeout management

How it works
Step 1 – Hardware initialization
- Initializes the ESP32.
- Configures the WS2812B LED strip.
- Initializes Wi-Fi and web server objects.
Step 2 – Access Point creation
- ESP32 creates its own Wi-Fi network.
- Phone or laptop connects to the ESP32 AP.
- The device becomes accessible for configuration.
Step 3 – HTML Wi-Fi configuration
- User opens the configuration page.
- Enters router SSID and password.
- ESP32 receives the submitted credentials.
Step 4 – Router connection
- ESP32 switches to AP + STA mode.
- Starts the router connection attempt.
- Checks whether the connection succeeds within the configured timeout.
Step 5 – Credential storage

When connection succeeds:
- SSID and password are stored using Preferences.
- Credentials remain available after power-off.
- On the next boot, ESP32 can load the previously saved credentials.
Step 6 – RGB LED indication

The WS2812B LED strip indicates device status:
LED colour	Meaning
Violet	AP mode active
Blue	Phone connected to ESP32 AP
Green	Router connection successful
Red	User-entered Wi-Fi connection failed

Important firmware functions
- showLEDColor() – Displays LED colour.
- setLEDStatus() – Selects LED status.
- loadWiFiCredentials() – Loads saved credentials.
- saveWiFiCredentials() – Stores credentials.
- startWiFiConnection() – Starts Wi-Fi connection.
- checkWiFiConnection() – Checks connection result.
- updateLEDStatus() – Updates LED based on actual connection.
How it helps
- Establishes basic ESP32 wireless communication.
- Allows users to configure Wi-Fi without modifying firmware.
- Provides permanent credential storage.
- Makes device status visible through RGB LEDs.
- Provides the foundation for advanced Wi-Fi management.