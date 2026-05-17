/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
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

/* Define this in your build or code */
#define FIRMWARE_VERSION  		0x00010002  // v1.0.2
#define FIRMWARE_SIZE 			0xD2A4
#define FIRMWARE_START_ADDR     0x8000000  // Application start address
/* Hash output size */
#define SHA256_HASH_SIZE      	32
#define HASH_CHUNK_SIZE  		1024  // Process 1KB at a time
#define SHA256_LEN   	  		32u
#define ED25519_SIGLEN 	  		64
#define MANIFEST_LEN      		(4u + 4u + SHA256_LEN + 4u)
#define ED25519_PRIVLEN  		64   /* expected private key format: secret||public (host-side) */

/* Flash sector for signed manifest storage */
#define MANIFEST_SECTOR         SECTOR_7
#define MANIFEST_FLASH_ADDR     0x08060000U
#define MAX_EXPECTED_FIRMWARE_SIZE   (512UL * 1024UL)

/* storage flags for where the key is provisioned */
#define SIG_KEY_STORAGE_NONE   0x00u
#define SIG_KEY_STORAGE_OTP    0x01u  /* provisioned to OTP */
#define SIG_KEY_STORAGE_FLASH  0x02u  /* provisioned to protected flash/bootloader area */

/* expected macros elsewhere in your project */
#ifndef ED25519_PRIVLEN
#define ED25519_PRIVLEN 32   /* seed/private length (adjust if you use 64-byte expanded key) */
#endif
#ifndef ED25519_PUBLEN
#define ED25519_PUBLEN 32
#endif





/* Enumerated return codes for firmware hashing */
typedef enum {
    FIRMWARE_HASH_SUCCESS = 0,
    FIRMWARE_HASH_ERR_NULL_INPUT = -1,
    FIRMWARE_HASH_ERR_BAD_LENGTH = -2,
    FIRMWARE_HASH_ERR_HASH_FAILED = -3,
    FIRMWARE_HASH_ERR_INVALID_SIZE = -4
} firmware_hash_status_t;

/* ---- Offsets (no magic numbers) ---- */
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

/* Added error for manifest creation */
typedef enum {
    SIGN_MANIFEST_SUCCESS = 0,
    SIGN_MANIFEST_ERR_NULL_PTR = -1,
    SIGN_MANIFEST_ERR_BAD_PRIVKEY_LEN = -2,
    SIGN_MANIFEST_ERR_SIGBUF_TOO_SMALL = -3,
    SIGN_MANIFEST_ERR_SERIALIZE = -4,
    SIGN_MANIFEST_ERR_SIGN_FAIL = -5,
    SIGN_MANIFEST_ERR_SIGLEN_MISMATCH = -6,
    SIGN_MANIFEST_ERR_MANIFEST_CREATE = -7    /* new: create_and_hash_manifest() failed */
} sign_manifest_status_t;

/* Status codes */
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

typedef struct __attribute__((packed)) {
    firmware_manifest_t manifest;
    uint8_t signature[ED25519_SIGLEN];   // Ed25519 signature
} signed_manifest_t;

typedef struct {
    struct {
        uint8_t  private_key[ED25519_PRIVLEN]; /* private signing key / seed */
        uint8_t  public_key[ED25519_PUBLEN];   /* verification key */
        size_t   priv_len;                     /* actual length in bytes (ED25519_PRIVLEN) */
        size_t   pub_len;                      /* actual length in bytes (ED25519_PUBLEN) */
        bool     provisioned;                  /* true if key was stored to permanent area */
        uint32_t storage_flags;                /* bitfield: SIG_KEY_STORAGE_* */
    } sig; /* signature keys */
    cmox_ecc_handle_t ecc;
    cmox_ctr_drbg_handle_t drbg;
    cmox_drbg_handle_t *drbg_ctx;
    uint8_t working_buffer[2000];
    /* optional metadata: provisioning timestamp, version, or reserved for future use */
    uint32_t provision_version; /* increment on re-provision (if ever) */
    uint32_t reserved[3];
} sig_keys_ctx_t;

/* Global instance you can extern from main.c / manifest_store.c */
extern sig_keys_ctx_t g_sig_keys;


extern signed_manifest_t g_signed_manifest;

firmware_hash_status_t hash_firmware_binary(uint32_t firmware_addr,
                                           uint32_t firmware_size,
                                           uint8_t *out_hash,
                                           size_t out_len);

firmware_hash_status_t hash_firmware_chunked(uint32_t firmware_addr,
                                             uint32_t firmware_size,
                                             uint8_t *out_hash,
                                             size_t out_len);

firmware_hash_status_t create_and_hash_manifest(firmware_manifest_t *manifest);

bool serialize_manifest(firmware_manifest_t *m,
                        uint8_t *out,
                        size_t out_len);

static inline bool write_u32_be(uint8_t *buf,
								size_t buf_len,
								size_t offset,
								uint32_t v);

sign_manifest_status_t sign_manifest_and_write_sig(sig_keys_ctx_t *ecc_ctx,
                                                   uint8_t *privkey,
                                                   size_t privkey_len,
                                                   firmware_manifest_t *manifest,
                                                   uint8_t *out_sig,
                                                   size_t *out_sig_len);

/**
 * @brief  Store signed manifest to protected flash sector
 * @param  sm  Pointer to fully populated signed_manifest_t (manifest + signature)
 * @return manifest_store_status_t
 * @note   Call this AFTER manifest is populated and signature is generated
 */
manifest_store_status_t store_signed_manifest_protected( signed_manifest_t *sm);
/**
 * @brief  Read signed manifest from protected flash
 * @param  sm_out  Output buffer for signed manifest
 * @return manifest_store_status_t
 */
manifest_store_status_t load_signed_manifest(signed_manifest_t *sm_out);




#endif /* DIGITAL_SIGNATURE_H_ */
