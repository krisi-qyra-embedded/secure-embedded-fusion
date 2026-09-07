/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef MAIN_MBEDTLS_AES_GCM_H_
#define MAIN_MBEDTLS_AES_GCM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mbedtls/gcm.h"
#include "mbedtls_ecdh.h"

#define ESP32_AES_GCM_KEY_SIZE       32U
#define ESP32_AES_GCM_NONCE_SIZE     12U
#define ESP32_AES_GCM_TAG_SIZE       16U
#define ESP32_GCM_COUNTER_START      UINT64_C(1)

#ifndef ESP32_GCM_DEBUG_PRINT_TX_PACKET
#define ESP32_GCM_DEBUG_PRINT_TX_PACKET             0U
#endif

#ifndef ESP32_GCM_HEALTH_EVERY_N_OK
#define ESP32_GCM_HEALTH_EVERY_N_OK                 10U
#endif


#define STM32_TELEMETRY_PLAINTEXT_SIZE             169U
#define STM32_TELEMETRY_PAYLOAD_SIZE               159U
#define STM32_TELEMETRY_SEQUENCE_INDEX               5U
#define STM32_TELEMETRY_CHECKSUM_INDEX             168U

#define STM32_TELEMETRY_NAME_BUFFER_SIZE            16U
#define STM32_TELEMETRY_UNIT_BUFFER_SIZE            20U
#define STM32_TELEMETRY_IAQ_LABEL_BUFFER_SIZE       33U
#define STM32_TELEMETRY_DS18B20_COUNT                5U

#define STM32_TELEM_BME680_NAME_LEN_INDEX             9U
#define STM32_TELEM_BME680_NAME_INDEX                10U
#define STM32_TELEM_BME680_NAME_LENGTH                6U
#define STM32_TELEM_BME680_TEMP_INDEX                16U
#define STM32_TELEM_BME680_PRESSURE_INDEX            20U
#define STM32_TELEM_BME680_HUMIDITY_INDEX            24U
#define STM32_TELEM_BME680_IAQ_SCORE_INDEX           28U
#define STM32_TELEM_BME680_IAQ_LABEL_INDEX           30U
#define STM32_TELEM_BME680_IAQ_LABEL_LENGTH          32U

#define STM32_TELEM_MAX31865_NAME_LEN_INDEX          62U
#define STM32_TELEM_MAX31865_NAME_INDEX              63U
#define STM32_TELEM_MAX31865_NAME_LENGTH              8U
#define STM32_TELEM_MAX31865_TEMP_INDEX              71U

#define STM32_TELEM_DS18B20_NAME_LEN_INDEX           75U
#define STM32_TELEM_DS18B20_NAME_INDEX               76U
#define STM32_TELEM_DS18B20_NAME_LENGTH              10U
#define STM32_TELEM_DS18B20_TEMP1_INDEX              86U
#define STM32_TELEM_DS18B20_TEMP2_INDEX              90U
#define STM32_TELEM_DS18B20_TEMP3_INDEX              94U
#define STM32_TELEM_DS18B20_TEMP4_INDEX              98U
#define STM32_TELEM_DS18B20_TEMP5_INDEX             102U

#define STM32_TELEM_PM25_NAME_LEN_INDEX             106U
#define STM32_TELEM_PM25_NAME_INDEX                 107U
#define STM32_TELEM_PM25_NAME_LENGTH                  5U
#define STM32_TELEM_PM1_STD_INDEX                   112U
#define STM32_TELEM_PM25_STD_INDEX                  114U
#define STM32_TELEM_PM10_STD_INDEX                  116U
#define STM32_TELEM_PM1_ATM_INDEX                   118U
#define STM32_TELEM_PM25_ATM_INDEX                  120U
#define STM32_TELEM_PM10_ATM_INDEX                  122U
#define STM32_TELEM_PARTICLE_03_INDEX               124U
#define STM32_TELEM_PARTICLE_05_INDEX               126U
#define STM32_TELEM_PARTICLE_10_INDEX               128U
#define STM32_TELEM_PARTICLE_25_INDEX               130U
#define STM32_TELEM_PARTICLE_50_INDEX               132U
#define STM32_TELEM_PARTICLE_100_INDEX              134U

#define STM32_TELEM_MPU9250_NAME_LEN_INDEX          136U
#define STM32_TELEM_MPU9250_NAME_INDEX              137U
#define STM32_TELEM_MPU9250_NAME_LENGTH               7U
#define STM32_TELEM_MPU9250_AX_INDEX                144U
#define STM32_TELEM_MPU9250_AY_INDEX                148U
#define STM32_TELEM_MPU9250_AZ_INDEX                152U
#define STM32_TELEM_MPU9250_GX_INDEX                156U
#define STM32_TELEM_MPU9250_GY_INDEX                160U
#define STM32_TELEM_MPU9250_GZ_INDEX                164U


typedef struct
{
    char name[STM32_TELEMETRY_NAME_BUFFER_SIZE];  
    uint16_t name_index;  

    float temperature;  
    uint16_t temperature_index;  
    char temperature_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

    float pressure;  
    uint16_t pressure_index;  
    char pressure_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

    float humidity;  
    uint16_t humidity_index;  
    char humidity_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

    int16_t iaq_score;  
    uint16_t iaq_score_index;  
    char iaq_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

    char iaq_label[STM32_TELEMETRY_IAQ_LABEL_BUFFER_SIZE];  
    uint16_t iaq_label_index;  

} stm32_bme680_telemetry_t;

typedef struct
{
    char name[STM32_TELEMETRY_NAME_BUFFER_SIZE];  
    uint16_t name_index;  

    float temperature;  
    uint16_t temperature_index;  
    char temperature_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

} stm32_max31865_telemetry_t;

typedef struct
{
    char name[STM32_TELEMETRY_NAME_BUFFER_SIZE];  
    uint16_t name_index;  

    float temperature[STM32_TELEMETRY_DS18B20_COUNT];  
    uint16_t temperature_index[STM32_TELEMETRY_DS18B20_COUNT];  
    char temperature_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

} stm32_ds18b20_telemetry_t;

typedef struct
{
    char name[STM32_TELEMETRY_NAME_BUFFER_SIZE];  
    uint16_t name_index;  

    uint16_t pm1_0_standard;  
    uint16_t pm1_0_standard_index;  

    uint16_t pm2_5_standard;  
    uint16_t pm2_5_standard_index;  

    uint16_t pm10_standard;  
    uint16_t pm10_standard_index;  

    uint16_t pm1_0_atmospheric;  
    uint16_t pm1_0_atmospheric_index;  

    uint16_t pm2_5_atmospheric;  
    uint16_t pm2_5_atmospheric_index;  

    uint16_t pm10_atmospheric;  
    uint16_t pm10_atmospheric_index;  

    uint16_t particles_0_3um;  
    uint16_t particles_0_3um_index;  

    uint16_t particles_0_5um;  
    uint16_t particles_0_5um_index;  

    uint16_t particles_1_0um;  
    uint16_t particles_1_0um_index;  

    uint16_t particles_2_5um;  
    uint16_t particles_2_5um_index;  

    uint16_t particles_5_0um;  
    uint16_t particles_5_0um_index;  

    uint16_t particles_10um;  
    uint16_t particles_10um_index;  

    char concentration_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  
    char particle_count_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

} stm32_pm25_telemetry_t;

typedef struct
{
    char name[STM32_TELEMETRY_NAME_BUFFER_SIZE];  
    uint16_t name_index;  

    float ax;  
    uint16_t ax_index;  

    float ay;  
    uint16_t ay_index;  

    float az;  
    uint16_t az_index;  

    float gx;  
    uint16_t gx_index;  

    float gy;  
    uint16_t gy_index;  

    float gz;  
    uint16_t gz_index;  

    char acceleration_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  
    char gyroscope_unit[STM32_TELEMETRY_UNIT_BUFFER_SIZE];  

} stm32_mpu9250_telemetry_t;

typedef struct
{
    bool valid;  

    uint32_t sequence;  
    uint16_t sequence_index;  
    uint16_t checksum_index;  

    stm32_bme680_telemetry_t bme680;  
    stm32_max31865_telemetry_t max31865;  
    stm32_ds18b20_telemetry_t ds18b20;  
    stm32_pm25_telemetry_t pm25;  
    stm32_mpu9250_telemetry_t mpu9250;  

} stm32_telemetry_packet_t;

#ifndef ESP32_TEST_FAULT_GCM_RX_AUTH_FAILURE
#define ESP32_TEST_FAULT_GCM_RX_AUTH_FAILURE          0U
#endif

#ifndef ESP32_TEST_FAULT_FRAME_BAD_MAGIC
#define ESP32_TEST_FAULT_FRAME_BAD_MAGIC              0U
#endif

#ifndef ESP32_TEST_FAULT_FRAME_BAD_FORMAT
#define ESP32_TEST_FAULT_FRAME_BAD_FORMAT             0U
#endif

#ifndef ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM
#define ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM           0U
#endif

/* Test: classify the request as the last duplicate; runtime resends its ACK. */
#ifndef ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST
#define ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST        0U
#endif


/* Test: drop one success ACK after processing; a fresh-nonce retry is ACKed without reprocessing. */
#ifndef ESP32_TEST_DROP_ONE_SUCCESS_ACK
#define ESP32_TEST_DROP_ONE_SUCCESS_ACK          0U
#endif


#ifndef ESP32_TEST_FAULT_REPLAY_REJECT_OLD
#define ESP32_TEST_FAULT_REPLAY_REJECT_OLD            0U
#endif

#ifndef ESP32_TEST_FAULT_REPLAY_BAD_NONCE
#define ESP32_TEST_FAULT_REPLAY_BAD_NONCE             0U
#endif

#ifndef ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR
#define ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR         0U
#endif

#if ((ESP32_TEST_FAULT_GCM_RX_AUTH_FAILURE != 0U) && \
     (ESP32_TEST_FAULT_GCM_RX_AUTH_FAILURE != 1U))
#error "ESP32_TEST_FAULT_GCM_RX_AUTH_FAILURE must be 0U or 1U"
#endif

#if ((ESP32_TEST_DROP_ONE_SUCCESS_ACK != 0U) && \
     (ESP32_TEST_DROP_ONE_SUCCESS_ACK != 1U))
#error "ESP32_TEST_DROP_ONE_SUCCESS_ACK must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_FRAME_BAD_MAGIC != 0U) && \
     (ESP32_TEST_FAULT_FRAME_BAD_MAGIC != 1U))
#error "ESP32_TEST_FAULT_FRAME_BAD_MAGIC must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_FRAME_BAD_FORMAT != 0U) && \
     (ESP32_TEST_FAULT_FRAME_BAD_FORMAT != 1U))
#error "ESP32_TEST_FAULT_FRAME_BAD_FORMAT must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM != 0U) && \
     (ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM != 1U))
#error "ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST != 0U) && \
     (ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST != 1U))
#error "ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_REPLAY_REJECT_OLD != 0U) && \
     (ESP32_TEST_FAULT_REPLAY_REJECT_OLD != 1U))
#error "ESP32_TEST_FAULT_REPLAY_REJECT_OLD must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_REPLAY_BAD_NONCE != 0U) && \
     (ESP32_TEST_FAULT_REPLAY_BAD_NONCE != 1U))
#error "ESP32_TEST_FAULT_REPLAY_BAD_NONCE must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR != 0U) && \
     (ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR != 1U))
#error "ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR must be 0U or 1U"
#endif

#if ((ESP32_TEST_FAULT_FRAME_BAD_MAGIC + \
      ESP32_TEST_FAULT_FRAME_BAD_FORMAT + \
      ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM) > 1U)
#error "Enable only one ESP32 plaintext-frame fault injection at a time"
#endif

#if ((ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST + \
      ESP32_TEST_FAULT_REPLAY_REJECT_OLD + \
      ESP32_TEST_FAULT_REPLAY_BAD_NONCE + \
      ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR) > 1U)
#error "Enable only one ESP32 replay fault injection at a time"
#endif

_Static_assert(
    ESP32_AES_GCM_KEY_SIZE == ESP32_ECDH_DERIVED_KEY_SIZE,
    "AES-GCM key size must match the ECDH-derived AES key");

_Static_assert(
    ESP32_AES_GCM_NONCE_SIZE == 12U,
    "AES-GCM nonce must be 12 bytes");

_Static_assert(
    ESP32_AES_GCM_TAG_SIZE == 16U,
    "AES-GCM tag must be 16 bytes");

typedef enum
{
    ESP32_AES_GCM_SUCCESS = 0,  

    ESP32_AES_GCM_ERR_BAD_PARAM            = -1,  
    ESP32_AES_GCM_ERR_ALREADY_INIT         = -2,  
    ESP32_AES_GCM_ERR_NOT_INITIALIZED      = -3,  
    ESP32_AES_GCM_ERR_BAD_ECDH_STATE       = -4,  
    ESP32_AES_GCM_ERR_SESSION_HASH         = -5,  
    ESP32_AES_GCM_ERR_NO_SESSION           = -6,  
    ESP32_AES_GCM_ERR_COUNTER_EXHAUSTED    = -7,  
    ESP32_AES_GCM_ERR_BAD_KEY              = -8,  
    ESP32_AES_GCM_ERR_OUTPUT_TOO_SMALL     = -9,  
    ESP32_AES_GCM_ERR_SETKEY               = -10,  
    ESP32_AES_GCM_ERR_ENCRYPT              = -11,  
    ESP32_AES_GCM_ERR_DECRYPT_AUTH         = -12,  
    ESP32_AES_GCM_ERR_LENGTH               = -13,  
    ESP32_AES_GCM_ERR_SELF_TEST_MISMATCH   = -14,  
    ESP32_AES_GCM_ERR_TASK_ALREADY_RUNNING = -15,  
    ESP32_AES_GCM_ERR_TASK_CREATE          = -16,  
    ESP32_AES_GCM_ERR_NVS                  = -17,  
    ESP32_AES_GCM_ERR_REPLAY               = -18  

} esp32_aes_gcm_status_t;

typedef struct
{
    uint32_t session_id;  
    uint64_t next_counter;  
    uint64_t current_counter;  
    bool initialized;  

} esp32_gcm_nonce_state_t;

typedef struct
{
    mbedtls_gcm_context gcm_tx;  
    mbedtls_gcm_context gcm_rx;  

    esp32_gcm_nonce_state_t nonce_state;  

    uint8_t iv_gcm[
        ESP32_AES_GCM_NONCE_SIZE];  

    bool initialized;  

} esp32_aes_gcm_ctx_t;

esp32_aes_gcm_status_t esp32_aes_gcm_init(
    esp32_aes_gcm_ctx_t *ctx);

void esp32_aes_gcm_free(
    esp32_aes_gcm_ctx_t *ctx);

esp32_aes_gcm_status_t esp32_gcm_nonce_session_init(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx);

esp32_aes_gcm_status_t esp32_gcm_nonce_allocate(
    esp32_aes_gcm_ctx_t *ctx);

esp32_aes_gcm_status_t
esp32_generate_nonce_and_iv_per_transfer(
    esp32_aes_gcm_ctx_t *ctx);

esp32_aes_gcm_status_t esp32_encrypt_aes_gcm(
    esp32_aes_gcm_ctx_t *ctx,
    const uint8_t key[ESP32_AES_GCM_KEY_SIZE],
    const uint8_t *plaintext,
    size_t plaintext_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t *ciphertext_and_tag,
    size_t output_capacity,
    size_t *output_len);

esp32_aes_gcm_status_t
esp32_generate_nonce_iv_encrypt_gcm_sender(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx,
    const uint8_t *plaintext,
    size_t plaintext_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t nonce_out[ESP32_AES_GCM_NONCE_SIZE],
    uint8_t *ciphertext_and_tag,
    size_t output_capacity,
    size_t *output_len);

esp32_aes_gcm_status_t
esp32_decrypt_verify_aes_gcm_receiver(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx,
    const uint8_t nonce[ESP32_AES_GCM_NONCE_SIZE],
    const uint8_t *ciphertext_and_tag,
    size_t input_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t *plaintext,
    size_t plaintext_capacity,
    size_t *plaintext_len);

esp32_aes_gcm_status_t esp32_aes_gcm_runtime_task_start(
    esp32_aes_gcm_ctx_t *gcm_ctx,
    esp32_ecdh_ctx_t *ecdh_ctx);

void esp32_aes_gcm_runtime_task_stop(void);

bool esp32_aes_gcm_runtime_task_is_running(void);

#endif /* MAIN_MBEDTLS_AES_GCM_H_ */
