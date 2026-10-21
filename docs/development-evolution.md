\# STM32–ESP32 Firmware Development Evolution



This page documents 10 selected engineering milestones

from the project's 54-commit development history.



The Git history was reconstructed from saved firmware

development snapshots.





\## Evolution Milestones



| Stage | Engineering milestone | Git revision | Video |

|---|---|---|---|

| 01 | Initial BME680 Sensor bringup DMA and diagnostics,recovery attempts | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/63b8c02641abab9251397a189f32e875769c3fc0) | \[Watch](https://youtu.be/0HvNQ7SX1Ro) |

| 02 | Final BME680 Sensor DMA,full diagnostics,recovery,watchdog escalation | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/66707ae44aeb709b5eb05302a13409eaa229712a) | \[Watch](https://youtu.be/QDfXN4FxAhw) |

| 03 | Initial mpu9250 Sensor DMA,diagnostics | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/5c9c540d3649bc815dd5cdb596b8be90cb4bade7) | \[Watch](https://youtu.be/fb4z7Y-UuZY) |

| 04 | Initial mpu9250 Sensor DMA,diagnostics | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/81b82e54887d8ce5c57b3b93fb0a9854b72d49dc) | \[Watch](https://youtu.be/QO5v6CTEE3Q) |

| 05 | MAX31865 Sensor DMA,diagnostics,retry strategies | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/4246425b5465e4be3970004fd393b6131e2302b5) | \[View screenshot](https://github.com/krisiqyra/secure-embedded-fusion/blob/main/max31865\_full\_specified\_fault\_diagnostics.png) |

| 06 |  FREERTOS(BME680 + MPU9250) compact diagnostic(no multi fault injection tests)| \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/d3c6a2e28a1394836aee9744883e3589e330e955) | \[Watch](https://youtu.be/XVHd3fkyb1o) |

| 07 |  FREERTOS(BME680 + MPU9250) Sensor full diagnostics,recovery,watchdog escalation + multi fault injection. | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/3af6b12687544f880f704f98368122a7018e510a) | \[Watch](https://youtu.be/2KtFeI2LFBU) |

| 08 |  FREERTOS(BME680 + MPU9250 + MAX31865 + pm25) Sensor full diagnostics,recovery,watchdog escalation + multi fault injection. | \[View code](https://github.com/krisiqyra/secure-embedded-fusion/commit/7c1d9bef4124942747bcb7a10b3ffeb034eeec9d) | \[Watch](https://youtu.be/dtY1UfiXqb0) |

| 09 | STM32,ESP32 aes gcm half duplex | \[View code STM32 side](https://github.com/krisiqyra/secure-embedded-fusion/commit/49675a4b89d4a34ad55f1a275712aef1c77b5e86) / \[View code ESP32 side](https://github.com/krisiqyra/secure-embedded-fusion/commit/69f39f62a039ebbd299a1c7a160291c9dcce832e) | \[Watch](https://youtu.be/tZ1dBUaURbc) |

| 10 | STM32,ESP32 aes gcm full duplex (ACK/NACK) | \[View code STM32 side](https://github.com/krisiqyra/secure-embedded-fusion/commit/c5e59a88d30e1c19142d14988f9974c528a31050) / \[View code ESP32 side](https://github.com/krisiqyra/secure-embedded-fusion/commit/8cce652a555db862ac5d9c9faf1c1f2c3e4a554e) | \[Watch](https://youtu.be/XUEpRf50c44) |







