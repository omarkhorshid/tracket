<img src="readme_images\tracket_logo.png" alt="Tracket Logo" width="100"/>

# Tracket - IoT-Enabled Smart Ping-Pong Racket

**Tracket** (a combination of "Track" and "Racket") is an IoT-enabled smart ping-pong racket designed to help players visualize key performance metrics in real-time. Whether you're practicing or playing competitively, Tracket provides valuable feedback to improve your technique and refine your shots.

## Features
- **Motion Tracking**: Uses an **MPU-6050 IMU** to measure racket orientation, acceleration, and angular velocity.
- **Impact Sensing**: A network of **piezoelectric disks** detects the precise location of ball contact on the racket.
- **Real-Time Data Processing**: An **STM32 microcontroller** handles sensor data acquisition and processing.
- **Wireless Connectivity**: An **ESP8266** enables Wi-Fi communication for data transmission and remote visualization.

<img src="readme_images\UI_screenshot1.jpg" alt="UI Screenshot" width="200"/>

## Use Cases
- Improve shot consistency by analyzing impact locations.
- Get real-time feedback on racket angles and swing dynamics.
- Track performance over time with data logging and visualization.

## Architecture
- **STM32 MCU**: Interfaces with sensors, processes data, and transmits it via UART to the ESP8266.
- **ESP8266**: Handles Wi-Fi connectivity for sending data to a mobile or web-based dashboard.
- **MPU-6050**: Provides IMU data for motion tracking.
- **Piezoelectric Disks**: Detect impact locations on the racket surface.

<img src="readme_images\architecture_diagram.png" alt="Architecutre Diagram" width="200"/>

## Schematic
<img src="readme_images\schematics.png" alt="Schematic Diagram" width="500"/>

## Measurements and Analysis
To validate the accuracy of the piezoelectric sensors, we performed measurements using an oscilloscope and the STM32 ADC. These measurements provide insight into the voltage response of the piezo sensors upon ball impact, helping to fine-tune signal processing algorithms.

<img src="readme_images\piezo_network.png" alt="Piezo elements network" width="200"/>

### Methodology
- The piezoelectric disks were connected to an oscilloscope to observe the raw voltage output upon impact.
- The STM32 ADC was used to capture digital readings of the sensor outputs for further processing and analysis.
- Different impact forces and locations were tested to evaluate sensor sensitivity and reliability.

<img src="readme_images\piezo_oscilloscope.png" alt="Piezo Oscilloscope measurements" width="300"/>

### Observations
- The oscilloscope showed voltage spikes corresponding to ball impact, with peak values depending on the force of the hit.
- The STM32 ADC successfully captured and digitized the signals, enabling real-time data processing.
- Variability in signal strength across different racket areas was noted, informing calibration and sensor placement adjustments.

<img src="readme_images\stm_adc_measurement.png" alt="STM32 ADC measurements" width="300"/>


---
