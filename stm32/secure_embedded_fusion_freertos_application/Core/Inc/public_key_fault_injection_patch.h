/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef INC_PUBLIC_KEY_FAULT_INJECTION_PATCH_H_
#define INC_PUBLIC_KEY_FAULT_INJECTION_PATCH_H_

#include <stdbool.h>
#include <stdint.h>

#define TEST_CORRUPT_STORED_BOOT_PUBLIC_KEY    1U

#define TEST_PUBLIC_KEY_FLASH_ADDR             ((uint32_t)0x08060070U)
#define TEST_PUBLIC_KEY_LEN                    32U
#define TEST_PUBLIC_KEY_WORDS                  \
    (TEST_PUBLIC_KEY_LEN / sizeof(uint32_t))

/*
 * Corrupt the stored Ed25519 public key using only Flash-safe
 * 1 -> 0 programming transitions.
 *
 * Returns:
 *     true  = corruption succeeded or was already applied
 *     false = Flash operation/verification failed
 */
bool fault_inject_corrupt_stored_boot_public_key(void);

#endif /* INC_PUBLIC_KEY_FAULT_INJECTION_PATCH_H_ */
