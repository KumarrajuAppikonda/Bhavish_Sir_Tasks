ESP32 Wi-Fi Configuration Manager (TCP Based)
Overview

This project is an ESP32-based Wi-Fi Configuration Manager that allows users to configure, save, manage, and switch Wi-Fi networks through a TCP server running on the ESP32 Access Point.

The device uses:

ESP32
WS2812B RGB LED strip
Push button
TCP server
Non-Volatile Storage (NVS)
Google HTTP Time Synchronization

The system automatically handles Wi-Fi connection attempts, AP mode operation, credential storage, network switching, time synchronization, and status indication.

Features
1. Automatic Wi-Fi Connection
Reads saved Wi-Fi credentials from NVS.
Tries to connect automatically at startup.
Retries connection every 10 seconds.
Continues retry attempts for 5 minutes.
Stops retrying if connection cannot be established.
2. Access Point (AP) Mode

Creates an ESP32 Access Point:

Plain Text
SSID: ESP32-WiFi-Setup
Password: esp32setup
Port: 5000

Used for:

Wi-Fi configuration
TCP communication
Remote management
3. TCP-Based Configuration Server

Runs a TCP server on:

Plain Text
Port 5000

Supports commands:

Plain Text
GET_STATUS
GET_WIFI_INFO
SET_WIFI|SSID|PASSWORD
CONNECT_WIFI
SAVE_CONFIG
FORGET_WIFI
RESTART
HELP
4. Secure Wi-Fi Credential Storage

Uses ESP32 Preferences (NVS):

C++
Preferences

Stores:

SSID
Password

Features:

Verification after saving
Protection against accidental overwrite
Safe credential removal
5. Wi-Fi Network Testing

Before storing credentials:

User sends Wi-Fi details.
ESP32 attempts connection.
Connection is verified.
Credentials are saved only if successful.

This prevents invalid networks from overwriting stored credentials.

6. Wi-Fi Forget Function

Credentials can be erased through:

TCP Command
Plain Text
FORGET_WIFI
 
Serial Command
Plain Text
CLEARNVS
Push Button

Hold button for:

Plain Text
10 seconds

After successful erase:

NVS is cleared
Current connection is disconnected
AP Mode starts automatically
LED Status Indication

The WS2812B LEDs provide device status.

Color	StatusViolet	AP Mode Active
Blue	TCP Client Connected
Green	Wi-Fi Connected
Red	Wi-Fi Not Connected
Off	Idle Mode
LED Behaviors
Green

Indicates:

Plain Text
Wi-Fi Connected

Shown for:

Plain Text
5 Minutes
Red

Indicates:

Plain Text
Wi-Fi Connection Failed

or

Plain Text
Retrying Saved Network
Blue

Indicates:

Plain Text
TCP Client Connected
Violet

Indicates:

Plain Text
Access Point Active
Push Button Functions
Short Press

Duration:

Plain Text
< 3 Seconds

Function:

Plain Text
Status Check

LED flashes:

Green if connected
Red if disconnected
Long Press

Duration:

Plain Text
> 3 Seconds

Function:

Plain Text
Start AP Mode
Very Long Press

Duration:

Plain Text
>= 10 Seconds

Function:

Plain Text
Forget Saved Wi-Fi
State Machine

The firmware operates using a finite state machine.

States
ST_BOOT

System startup.

ST_CHECK_NVS

Check stored Wi-Fi credentials.

ST_WIFI_CONNECTED

Connected to saved Wi-Fi.

ST_WIFI_FAILED

Failed to connect.

ST_AP_WAITING

AP active, waiting for client.

ST_CLIENT_CONNECTED

TCP client connected.

ST_TCP_CONFIGURATION

Receiving configuration commands.

ST_WIFI_CONNECTING

Testing new Wi-Fi credentials.

ST_CONFIG_SUCCESS

New Wi-Fi connected successfully.

ST_NORMAL_IDLE

Normal operation.

Time Synchronization

The system synchronizes time using HTTP requests to Google servers.

Endpoints used:

Plain Text
http://clients3.google.com/generate_204
http://www.google.com

Method:

HTTP GET request
Read HTTP "Date" header
Convert to UTC epoch time
Update ESP32 system clock

Timezone:

Plain Text
UTC +05:30 (IST)
Communication Protocol

TCP messages are line-based.

Delimiter:

Plain Text
|

Example:

Plain Text
SET_WIFI|MyWiFi|mypassword

Supported escaping:

Plain Text
\| -> Literal pipe
\\ -> Literal backslash
Libraries Used
C++
Arduino.h
WiFi.h
WiFiClient.h
WiFiServer.h
Preferences.h
HTTPClient.h
FastLED.h
time.h
sys/time.h
Purpose of Each Library
Arduino.h

Core Arduino framework.

WiFi.h

ESP32 Wi-Fi management.

WiFiClient.h

TCP client functionality.

WiFiServer.h

TCP server functionality.

Preferences.h

NVS storage management.

HTTPClient.h

HTTP requests for time sync.

FastLED.h

WS2812B LED control.

time.h

Date and time functions.

sys/time.h

System clock management.

Hardware Requirements
ESP32 Development Board
WS2812B LED Strip (8 LEDs)
Push Button
USB Power Supply
Project Workflow
Plain Text
Power ON
↓
Read Saved Credentials
↓
Start AP Mode
↓
Try Saved Wi-Fi
↓
Connected?
├─ YES → Green → Normal Operation
└─ NO → Red Retry Mode
↓
TCP Client Connects
↓
SET_WIFI Command
↓
CONNECT_WIFI Test
↓
Success?
├─ YES → SAVE_CONFIG
└─ NO → Retry