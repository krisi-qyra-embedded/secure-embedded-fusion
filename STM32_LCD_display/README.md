\# STM32F411RE — BME680 Real-Time LVGL Dashboard



\## Project Overview



This project demonstrates the integration of a BME680 environmental sensor with an ST7789 TFT LCD using an STM32F411RE microcontroller.



The system acquires environmental measurements through I²C, processes the sensor data, and displays the results using the LVGL 8.3 graphical library.



The project focuses on low-level peripheral configuration, sensor-driver integration, graphical interface development, and real-time embedded firmware.



\## Hardware Components



| Component | Description |

|---|---|

| Microcontroller | STM32F411RE Nucleo |

| Environmental sensor | Bosch BME680 |

| TFT display | ST7789, 240 × 320 pixels |

| Display interface | SPI2 |

| Sensor interface | I²C1 |

| System clock | 96 MHz |

| Graphics framework | LVGL 8.3 |



\## Development Process



\### 1. System Clock Configuration



The STM32F411RE is configured to operate at 96 MHz using the PLL and an 8 MHz HSE clock source.



The clock configuration provides:



\- SYSCLK: 96 MHz

\- AHB: 96 MHz

\- APB1: 48 MHz

\- APB2: 96 MHz



SPI2 uses the APB1 clock with a prescaler of 8, resulting in a 6 MHz SPI clock.



\### 2. ST7789 SPI Display Driver



A custom ST7789 driver handles LCD initialization, command transmission, display-area selection, and RGB565 pixel transfers.



The SPI2 pins are:



\- PB10 — SPI2 SCK

\- PB15 — SPI2 MOSI

\- PB3 — LCD chip select

\- PB4 — LCD data/command

\- PB5 — LCD reset



\### 3. LVGL 8.3 Integration



LVGL is integrated with the custom display driver using a display-buffer descriptor and a flush callback.



The rendering flow is:



1\. LVGL generates pixel data in its draw buffer.

2\. The display flush callback receives the updated region.

3\. The ST7789 driver transmits the RGB565 pixel data through SPI.

4\. The flush callback notifies LVGL when the transfer is complete.



This allows the application to render graphical interface objects without directly managing their individual pixels.



\### 4. BME680 Sensor Integration



The BME680 communicates with the STM32 through I²C1 using PB8 for SCL and PB9 for SDA.



The sensor driver performs initialization, calibration-data retrieval, measurement configuration, and sensor-data acquisition.



The BME680 provides:



\- Temperature in °C

\- Relative humidity in %

\- Atmospheric pressure in hPa

\- Gas resistance in Ω or kΩ



\### 5. Live Sensor Dashboard



The user interface uses a black background, white measurement labels, and green numerical values.



The application periodically acquires new BME680 measurements and updates the LVGL labels with the latest available data.



The main loop also services the LVGL timer handler to process graphical updates.



\### 6. Fault Handling



The firmware integrates sensor error checking and watchdog supervision. The design includes an independent watchdog mechanism for recovering from unrecoverable execution or sensor failures.



\## Hardware Demonstration



Photos of the hardware setup are stored in the `Photos/` directory.



Videos showing the display and sensor operation are stored in the `Videos/` directory.



\## Project Objective



The purpose of this project is to demonstrate practical embedded-system integration involving sensor acquisition, hardware interfaces, microcontroller clock configuration, custom display drivers, and graphical user-interface development.



\## Technologies



\*\*Embedded C · STM32 HAL · ARM Cortex-M4 · SPI · I²C · DMA · LVGL 8.3 · ST7789 · BME680\*\*

