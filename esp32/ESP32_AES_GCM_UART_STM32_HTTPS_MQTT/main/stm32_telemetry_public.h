/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef STM32_TELEMETRY_PUBLIC_H
#define STM32_TELEMETRY_PUBLIC_H

#include <stdbool.h>
#include "mbedtls_aes_gcm.h"

bool stm32_telemetry_get_latest(stm32_telemetry_packet_t *out);

#endif /* STM32_TELEMETRY_PUBLIC_H */
