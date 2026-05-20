/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file digital_signature.h
 * @brief Secure-boot manifest, hashing, Ed25519 and protected-storage declarations.
 */
#ifndef DIGITAL_SIGNATURE_H_
#define DIGITAL_SIGNATURE_H_
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"
#include <nonce.h>
#include "stm32f4xx.h"
#include "cmox_crypto.h"
#include <entropy_personalization_nonce_gen.h>
#include "flash_protection.h"
#define VECT_TAB_BASE_ADDRESS		FLASH_BASE
#define VECT_TAB_OFFSET				0x0000
#define FIRMWARE_VERSION  		0x00010002  // v1.0.2
#define FIRMWARE_SIZE        	0x00040000UL
#define FIRMWARE_START_ADDR     0x08020000  // Application start address
#define APP_SECTOR5_START_ADDR   (0x08020000UL)
#define APP_SECTOR5_END_ADDR     (0x0805FFFFUL)
#define APP_SECTOR5_SIZE_BYTES   0x00040000UL
#define APP_SLOT_SIZE_BYTES             0x00040000UL
#define MAX_FIRMWARE_SIZE               APP_SLOT_SIZE_BYTES
#define FLASH_WORD_SIZE         4U
#define FLASH_ERASED_VALUE      0xFFU
#define ALIGN_UP(x, align)      (((x) + ((align) - 1U)) & ~((align) - 1U))
#define FIRMWARE_SIZE_PADDED    ALIGN_UP(FIRMWARE_SIZE, FLASH_WORD_SIZE)
#define FIRMWARE_PADDING_BYTES  (FIRMWARE_SIZE_PADDED - FIRMWARE_SIZE)
#define SHA256_HASH_SIZE      	32
#define HASH_CHUNK_SIZE  		1024  // Process 1KB at a time
#define SHA256_LEN   	  		32u
#define ED25519_SIGLEN 	  		64
#define MANIFEST_LEN      		(4u + 4u + SHA256_LEN + 4u)
#define ED25519_PRIVLEN  		64
#define MANIFEST_SECTOR         		SECTOR_7
#define MANIFEST_FLASH_ADDR     		0x08060000U
#define PUBLIC_KEY_FLASH_ADDR  			((uint32_t)0x08060070U)
#define MAX_EXPECTED_FIRMWARE_SIZE      APP_SLOT_SIZE_BYTES
#define SIG_KEY_STORAGE_NONE   0x00u
#define SIG_KEY_STORAGE_OTP    0x01u
#define SIG_KEY_STORAGE_FLASH  0x02u
#define APP_SLOT_START_ADDR        ((uint32_t)0x08020000UL)
#define APP_ADDRESS  				APP_SLOT_START_ADDR
#define IMAGE_READY_FLAG_ADDR      ((uint32_t)0x08060A00UL)
#define IMAGE_READY_MAGIC          ((uint32_t)0xA55A5AA5UL)
#define IMAGE_READY_STATE_VALID    ((uint32_t)0x00000001UL)
#define FW_READY_POLL_DELAY_MS     ((uint32_t)100U)
#ifndef ED25519_PRIVLEN
#define ED25519_PRIVLEN 32
#endif
#ifndef ED25519_PUBLEN
#define ED25519_PUBLEN 32
#endif
/** @brief Firmware hashing status values. */
typedef enum {
	FIRMWARE_HASH_SUCCESS           =  0,
	FIRMWARE_HASH_ERR_NULL_INPUT    = -1,
	FIRMWARE_HASH_ERR_BAD_LENGTH    = -2,
	FIRMWARE_HASH_ERR_INVALID_SIZE  = -3,
	FIRMWARE_HASH_ERR_HASH_FAILED   = -4,
	FIRMWARE_HASH_ERR_CONSTRUCT     = -5,
	FIRMWARE_HASH_ERR_INIT          = -6,
	FIRMWARE_HASH_ERR_APPEND        = -7,
	FIRMWARE_HASH_ERR_FINALIZE      = -8,
} firmware_hash_status_t;
typedef enum
{
    SECTOR5_CHECK_OK = 0,
    SECTOR5_CHECK_ALL_FF,
    SECTOR5_CHECK_ALL_00,
    SECTOR5_CHECK_DUMMY_00_FF_ONLY
} sector5_check_status_t;
typedef enum
{
    VERIFY_AUTH_NEGTEST_PASS = 0,
    VERIFY_AUTH_NEGTEST_ERR_NULL_PTR = -1,
    VERIFY_AUTH_NEGTEST_ERR_FLASH_READ = -2,
    VERIFY_AUTH_NEGTEST_ERR_NOT_PROVISIONED = -3,
    VERIFY_AUTH_NEGTEST_ERR_SIG_EMPTY = -4,
    VERIFY_AUTH_NEGTEST_ERR_PUBKEY_EMPTY = -5,
    VERIFY_AUTH_NEGTEST_ERR_SERIALIZE = -6,
    VERIFY_AUTH_NEGTEST_ERR_ECC_VERIFY = -7,
    VERIFY_AUTH_NEGTEST_ERR_FAULT_CHECK = -8,
    VERIFY_AUTH_NEGTEST_ERR_UNEXPECTED_SUCCESS = -9
} verify_auth_negtest_status_t;
typedef enum
{
    IMAGE_READY_OK = 0,
    IMAGE_READY_NOT_SET = 1,
    IMAGE_READY_INVALID = 2,
    IMAGE_READY_ERR_BAD_PARAM = -1
} image_ready_status_t;
typedef enum
{
    WAIT_IMAGE_READY_OK = 0,
    WAIT_IMAGE_READY_ERR_BAD_PARAM = -1,
    WAIT_IMAGE_READY_ERR_TIMEOUT = -2
} wait_image_ready_status_t;
enum {
    OFF_VERSION = 0u,
    OFF_FW_SIZE = OFF_VERSION + 4u,
    OFF_HASH    = OFF_FW_SIZE + 4u,
    OFF_ROLLBK  = OFF_HASH + SHA256_LEN
};
typedef struct __attribute__((packed)) {
    uint32_t version;
    uint32_t firmware_size;
    uint8_t  sha256_hash[32];
    uint32_t rollback_counter;
} firmware_manifest_t;
/**
 * @brief Create a manifest and hash the firmware with optional word padding.
 * @param manifest Manifest output structure.
 * @param use_padding Enable erased-byte padding to the word boundary.
 * @return Firmware hashing status.
 */
firmware_hash_status_t create_and_hash_manifest_with_padding(
    firmware_manifest_t *manifest,
    bool use_padding
);
typedef enum {
    SIGN_MANIFEST_SUCCESS = 0,
    SIGN_MANIFEST_ERR_NULL_PTR = -1,
    SIGN_MANIFEST_ERR_BAD_PRIVKEY_LEN = -2,
    SIGN_MANIFEST_ERR_SIGBUF_TOO_SMALL = -3,
    SIGN_MANIFEST_ERR_SERIALIZE = -4,
    SIGN_MANIFEST_ERR_SIGN_FAIL = -5,
    SIGN_MANIFEST_ERR_SIGLEN_MISMATCH = -6,
    SIGN_MANIFEST_ERR_MANIFEST_CREATE = -7,
	MANIFEST_PUBKEY_STORED_FLASH_ERR = -8
} sign_manifest_status_t;
typedef enum {
    MANIFEST_STORE_SUCCESS          =  0,
    MANIFEST_STORE_ERR_NULL_INPUT   = -1,
    MANIFEST_STORE_ERR_FLASH_WRITE  = -2,
    MANIFEST_STORE_ERR_VERIFY_FAIL  = -3,
    MANIFEST_STORE_ERR_WRP_FAIL     = -4,
    MANIFEST_STORE_ERR_ERASE_FAIL   = -5,
	MANIFEST_STORE_ERR_INVALID_MANIFEST = -6,
	MANIFEST_STORE_ERR_EMPTY = -7,
	MANIFEST_STORE_ERR_NO_SIGNATURE = -8,
	MANIFEST_STORE_ERR_FLASH_READ = -9,
} manifest_store_status_t;
typedef enum
{
    FLASH_REGION_STATE_EMPTY = 0,
    FLASH_REGION_STATE_HAS_DATA = 1,
    FLASH_REGION_STATE_ERR = -1
} flash_region_state_t;
typedef enum
{
    VERIFY_AUTH_SUCCESS = 0,
    VERIFY_AUTH_ERR_NULL_PTR = -1,
    VERIFY_AUTH_ERR_NOT_PROVISIONED = -2,
    VERIFY_AUTH_ERR_FLASH_READ = -3,
    VERIFY_AUTH_ERR_SERIALIZE = -4,
    VERIFY_AUTH_ERR_ECC_VERIFY = -5,
    VERIFY_AUTH_ERR_AUTH_FAIL = -6,
    VERIFY_AUTH_ERR_FAULT_CHECK = -7,
    VERIFY_AUTH_ERR_SIG_EMPTY = -8,
    VERIFY_AUTH_ERR_PUBKEY_EMPTY = -9
} verify_auth_status_t;
typedef enum
{
    VERIFY_INTEGRITY_SUCCESS = 0,
    VERIFY_INTEGRITY_ERR_NULL_PTR = -1,
    VERIFY_INTEGRITY_ERR_INVALID_SIZE = -2,
    VERIFY_INTEGRITY_ERR_HASH_FAILED = -3,
    VERIFY_INTEGRITY_ERR_STORED_HASH_EMPTY = -4,
    VERIFY_INTEGRITY_ERR_HASH_MISMATCH = -5
} verify_integrity_status_t;
typedef struct __attribute__((packed)) {
    firmware_manifest_t manifest;
    uint8_t signature[ED25519_SIGLEN];   // Ed25519 signature
} signed_manifest_t;
typedef struct {
    struct {
        uint8_t  private_key[ED25519_PRIVLEN];
        uint8_t  public_key[ED25519_PUBLEN];
        size_t   priv_len;
        size_t   pub_len;
        bool     provisioned;
        uint32_t storage_flags;
    } sig;
    cmox_ecc_handle_t ecc;
    cmox_ctr_drbg_handle_t drbg;
    cmox_drbg_handle_t *drbg_ctx;
    uint8_t working_buffer[2000];
    uint32_t provision_version;
    uint32_t reserved[3];
} sig_keys_ctx_t;
typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t image_addr;
    uint32_t image_addr_inv;
    uint32_t image_size;
    uint32_t image_size_inv;
    uint32_t state;
    uint32_t state_inv;
} image_ready_flag_t;
extern sig_keys_ctx_t g_sig_keys;
extern signed_manifest_t g_signed_manifest;
/**
 * @brief Hash a firmware image directly from Flash.
 * @param firmware_addr Firmware start address.
 * @param firmware_size Firmware size in bytes.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Firmware hashing status.
 */
firmware_hash_status_t hash_firmware_binary(uint32_t firmware_addr,
                                           uint32_t firmware_size,
                                           uint8_t *out_hash,
                                           size_t out_len);
/**
 * @brief Hash a firmware image in fixed-size chunks.
 * @param firmware_addr Firmware start address.
 * @param firmware_size Firmware size in bytes.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Firmware hashing status.
 */
firmware_hash_status_t hash_firmware_chunked(uint32_t firmware_addr,
                                             uint32_t firmware_size,
                                             uint8_t *out_hash,
                                             size_t out_len);
/**
 * @brief Populate the firmware manifest and calculate its firmware digest.
 * @param manifest Manifest structure to populate.
 * @return Firmware hashing status.
 */
firmware_hash_status_t create_and_hash_manifest(firmware_manifest_t *manifest);
/**
 * @brief Serialize a manifest into the canonical signed byte layout.
 * @param m Manifest to serialize.
 * @param out Destination byte buffer.
 * @param out_len Destination buffer size.
 * @return True when serialization succeeds.
 */
bool serialize_manifest(firmware_manifest_t *m,
                        uint8_t *out,
                        size_t out_len);
/**
 * @brief Write one 32-bit value in big-endian form into a bounded buffer.
 * @param buf Destination buffer.
 * @param buf_len Destination size.
 * @param offset Write offset.
 * @param v Value to encode.
 * @return True when the write fits.
 */
static inline bool write_u32_be(uint8_t *buf,
								size_t buf_len,
								size_t offset,
								uint32_t v);
/**
 * @brief Sign a firmware manifest with the Ed25519 signing key.
 * @param ecc_ctx Signing context.
 * @param privkey Private signing key.
 * @param privkey_len Private-key length.
 * @param manifest Manifest to sign.
 * @param out_sig Signature output buffer.
 * @param out_sig_len Signature length output.
 * @return Manifest signing status.
 */
sign_manifest_status_t sign_manifest_and_write_sig(sig_keys_ctx_t *ecc_ctx,
                                                   uint8_t *privkey,
                                                   size_t privkey_len,
                                                   firmware_manifest_t *manifest,
                                                   uint8_t *out_sig,
                                                   size_t *out_sig_len);
/**
 * @brief Store the signed manifest in its protected Flash region.
 * @param sm Signed manifest to store.
 * @param ctx Signing-key context.
 * @return Manifest storage status.
 */
manifest_store_status_t store_signed_manifest_protected(signed_manifest_t *sm, sig_keys_ctx_t *ctx);
/**
 * @brief Load the signed manifest from Flash.
 * @param sm_out Destination signed-manifest structure.
 * @return Manifest storage status.
 */
manifest_store_status_t load_signed_manifest(signed_manifest_t *sm_out);
/**
 * @brief Hash firmware with optional erased-byte padding to the word boundary.
 * @param firmware_addr Firmware start address.
 * @param firmware_size Firmware size in bytes.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @param use_padding Enable word padding.
 * @return Firmware hashing status.
 */
firmware_hash_status_t hash_firmware_with_padding(
    uint32_t firmware_addr,
    uint32_t firmware_size,
    uint8_t *out_hash,
    size_t out_len,
    bool use_padding
);
/**
 * @brief Provision the signing public key into OTP.
 * @param ctx Signing-key context.
 * @param lock_otp Lock OTP after successful provisioning.
 * @return OTP operation status.
 */
otp_status_t provision_public_key_to_otp(sig_keys_ctx_t *ctx, bool lock_otp);
/**
 * @brief Clear signing keys and related working material from RAM.
 * @param ctx Signing-key context to clear.
 */
void secure_zero_all_keys(sig_keys_ctx_t *ctx);
/**
 * @brief Convert a firmware hashing status to a diagnostic string.
 * @param status Firmware hashing status.
 * @return Constant status string.
 */
const char* firmware_hash_status_str(firmware_hash_status_t status);
/**
 * @brief Store the public verification key in the configured protected region.
 * @param ctx Signing-key context.
 * @return Manifest storage status.
 */
manifest_store_status_t store_publickey_protected(sig_keys_ctx_t *ctx);
/**
 * @brief Validate the application image-ready flag.
 * @param expected_image_addr Expected image address.
 * @param expected_image_size Expected image size.
 * @return Image-ready status.
 */
image_ready_status_t image_ready_flag_check(uint32_t expected_image_addr,
                                            uint32_t expected_image_size);
/**
 * @brief Poll the image-ready flag until valid or until the poll limit is reached.
 * @param expected_image_addr Expected image address.
 * @param expected_image_size Expected image size.
 * @param poll_delay_ms Delay between polls.
 * @param max_polls Maximum poll count.
 * @return Image-ready wait status.
 */
wait_image_ready_status_t wait_for_image_ready_flag(uint32_t expected_image_addr,
                                                    uint32_t expected_image_size,
                                                    uint32_t poll_delay_ms,
                                                    uint32_t max_polls);
/**
 * @brief Load and verify the signed manifest against the provisioned Ed25519 trust anchor.
 * @param ecc_ctx Verification context.
 * @param out_sm Verified signed-manifest output.
 * @return Manifest authenticity status.
 */
verify_auth_status_t verify_manifest_authenticity_from_flash(sig_keys_ctx_t *ecc_ctx,
                                                             signed_manifest_t *out_sm);
#endif
