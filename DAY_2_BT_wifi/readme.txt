Task 2 – Automatic Wi-Fi Retry and Time Synchronization
Connection Reliability 

Objective: Develop a Wi-Fi manager that automatically searches for previously saved networks, manages AP availability using a timer, and provides real-time connection and time status.

This task introduces more structured connection management and recovery compared with the basic Wi-Fi implementation.
Components and technologies used
Component / Technology	Purpose
ESP32-WROOM-32	Main controller
WS2812B	Status indication
GPIO4	Push-button input
GPIO5	LED data output
Wi-Fi AP + STA	Network management
WebServer	Wi-Fi configuration
DNSServer	Captive portal
Preferences / NVS	Save credentials
WiFiClient	HTTP communication
millis()	Timer management
State machine	Wi-Fi retry management

How it works
Step 1 – Device boot
- ESP32 initializes the LED and button.
- Loads previously saved Wi-Fi credentials.
- Starts AP mode immediately.
- Searches for the saved Wi-Fi network.
Step 2 – AP mode management
- AP is available for one minute after boot.
- Phone can connect to the ESP32 AP.
- User can configure router credentials through the captive portal.
- AP closes after its one-minute window.
- Holding the button for 10 seconds opens AP mode again for one minute.
Step 3 – Automatic Wi-Fi retry

The firmware uses a retry phase mechanism:
- RP_IDLE – No retry operation.
- RP_WAITING – Waiting for the next retry.
- RP_CHECKING – Attempting connection to saved Wi-Fi.
The saved network is periodically checked, and unsuccessful attempts are followed by a retry waiting period.

Step 4 – Connection recovery

If the router disconnects:
- Firmware detects the lost connection.
- Starts another saved-network connection attempt.
- Keeps the configuration AP functionality available according to its timing logic.
Step 5 – Time synchronization

The firmware obtains the HTTP Date header from external servers:
- Google
- Cloudflare
- Microsoft
It parses the date and updates the ESP32 system clock without using NTP.

Step 6 – Live serial monitoring

The serial monitor displays:
- Date and time.
- Saved network name.
- Router connection status.
- AP status.
- Retry countdown.
- AP countdown.
- Button status.
- LED status.
The information is continuously updated in one line.
Important firmware functions
- startAP() – Activates AP.
- stopAP() – Deactivates AP.
- startWiFiConnection() – Starts connection attempt.
- checkWiFiConnection() – Checks connection result.
- handleWiFiLoss() – Detects lost router connection.
- handleButton() – Detects 10-second button hold.
- handleAPHousekeeping() – Manages AP timeout.
- updateLEDStatus() – Updates LED indication.
- fetchHttpDate() – Retrieves HTTP date.
- printSerialStatus() – Prints live status.
How it helps
- Reduces manual Wi-Fi configuration.
- Improves network recovery.
- Controls AP availability to reduce unnecessary AP operation.
- Helps diagnose connectivity problems.
- Provides accurate system date and time when internet connectivity is available.
- Introduces timer-based state-machine development.