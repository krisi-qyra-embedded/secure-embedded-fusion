\# STM32–ESP32 Complete System Runtime Demonstrations



This page documents 12 runtime demonstrations of the completed Secure STM32–ESP32 IoT Sensor Platform.



The recordings demonstrate system operation, authenticated communication, security verification, fault injection, error handling, watchdog recovery, and network integration.



\## Runtime Demonstrations



| Test | Runtime demonstration | Evidence |

|---|---|---|

| 01 | aes gcm stm32 esp32 stm32 usart6 rx tx physical fault inject | \[Watch](https://youtu.be/X7eZtsBszC8) |

| 02 | stm32 OTP provisioning | \[Watch](https://youtu.be/wdaOvRgBG60) |

| 03 | stm32 secure boot authenticity fault injection — firmware integrity fault injection | \[Watch](https://youtu.be/ile1lusTBxY) |

| 04 | stm32 multiple sensor fault injections diagnostics | \[Watch](https://youtu.be/KOrZ5UCTcxE) |

| 05 | aes gcm stm32 esp32 max31865 fault injection diagnostics | \[Watch](https://youtu.be/660VmY-hY60) |

| 06 | aes gcm stm32 esp32 replay previous complete packet (replay protection) | \[Watch](https://youtu.be/L9\_u7nxjYHs) |

| 07 | ESP32 OTA: Firmware Update Through the HTTPS Dashboard | \[Watch](https://youtu.be/FNKGb12FPp8) |

| 08 | ESP32 AWS IoT: MQTT Publish and Subscribe | \[Watch](https://youtu.be/moGgV8YiYu8) |

| 09 | STM32–ESP32 AES-GCM: Ciphertext Corruption Fault Injection | \[Watch](https://youtu.be/-JlaIp6BnVY) |

| 10 | STM32–ESP32: AWS IoT and HTTPS Dashboard Demonstration | \[Watch](https://youtu.be/Hf6foBqTHxE) |

| 11 | STM32–ESP32 HTTPS Dashboard: Live Sensor Monitoring | \[Watch](https://youtu.be/cFucC2f8QIc) |

| 12 | STM32 Secure Boot: Firmware Integrity Fault Injection | \[Watch](https://youtu.be/wribxSScLYQ) |





\## Runtime Demonstration Source Code



The runtime demonstrations are supported by three main firmware components:



| Firmware component | Description | Source code |

|---|---|---|

| STM32 Application | FreeRTOS sensor acquisition, DMA, fault diagnostics, watchdog recovery, and authenticated AES-GCM communication | \[View STM32 source](https://github.com/krisiqyra/secure-embedded-fusion/tree/main/stm32/secure_embedded_fusion_freertos_application) |

| ESP32 Application | AES-GCM communication, replay protection, HTTPS dashboard, AWS IoT MQTT, and OTA firmware updates | \[View ESP32 source](https://github.com/krisiqyra/secure-embedded-fusion/tree/main/esp32/ESP32_AES_GCM_UART_STM32_HTTPS_MQTT) |

| STM32 Secure Bootloader | Ed25519 manifest verification, firmware integrity and authenticity validation, and application handoff | \[View Secure Bootloader](https://github.com/krisiqyra/secure-embedded-fusion/tree/main/stm32/secure_bootloader) |



\## Technical Documentation



For the source-code evolution and corresponding firmware revisions, see:



\[STM32–ESP32 Firmware Development Evolution](development-evolution.md)

