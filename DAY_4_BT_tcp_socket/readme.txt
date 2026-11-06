Task 4 – MQ-5 Gas Sensor ADC Integration
ADC Sensor Integration







escolaobompastor.com.br


Objective: Integrate an MQ-5 gas sensor with ESP32 and read its analog output through ADC Channel 6, allowing the sensor to stabilize for three minutes before accepting readings.
Components and technologies used
Component	Purpose
ESP32-WROOM-32	Main microcontroller
MQ-5 sensor	Gas sensing
GPIO34	Analog input
ADC1 Channel 6	Analog-to-digital conversion
PlatformIO	Firmware development
Arduino framework	Hardware interface
analogRead()	ADC sampling
millis()	Non-blocking timer

How it works
Step 1 – Boot and initialization
- ESP32 starts.
- Existing Task 3 firmware initializes.
- ADC1 Channel 6 is configured using GPIO34.
- A three-minute warm-up timer starts.
Step 2 – Sensor stabilization

During the first 180 seconds:
- ADC readings are sampled.
- Readings are displayed for monitoring.
- All readings are marked as IGNORED.
- They are not treated as valid measurements.
Step 3 – ADC measurement

After 180 seconds:
- Sensor state changes to WARMUP_COMPLETE.
- ADC readings become eligible for measurement.
- Raw ADC values are continuously updated.
Step 4 – Serial monitoring

The ADC value is integrated into the existing single-line serial output.

Before warm-up completion:
ADC: 1250 (IGN) | ADC_TIMER: 02:59


After warm-up completion:
ADC: 1250


Important firmware functions
- initMq5Adc() – Initializes ADC.
- handleMq5Sensor() – Handles sampling and warm-up.
- mq5WarmupElapsed() – Checks the three-minute timer.
- printSerialStatus() – Displays ADC information.
How it helps
- Prevents premature use of sensor readings.
- Allows the sensor heating and stabilization period.
- Provides continuous raw ADC monitoring.
- Keeps Wi-Fi and sensor operations running simultaneously.
- Creates a foundation for future gas detection and alarm functionality