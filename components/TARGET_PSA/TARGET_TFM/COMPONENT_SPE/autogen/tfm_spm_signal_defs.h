/*
 * Copyright (c) 2018, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef __TFM_SPM_SIGNAL_DEFS_H__
#define __TFM_SPM_SIGNAL_DEFS_H__

#define PSA_ITS_GET_MSK_POS (4UL)
#define PSA_ITS_GET_MSK (1UL << PSA_ITS_GET_MSK_POS)
#define PSA_ITS_SET_MSK_POS (5UL)
#define PSA_ITS_SET_MSK (1UL << PSA_ITS_SET_MSK_POS)
#define PSA_ITS_INFO_MSK_POS (6UL)
#define PSA_ITS_INFO_MSK (1UL << PSA_ITS_INFO_MSK_POS)
#define PSA_ITS_REMOVE_MSK_POS (7UL)
#define PSA_ITS_REMOVE_MSK (1UL << PSA_ITS_REMOVE_MSK_POS)
#define PSA_ITS_RESET_MSK_POS (8UL)
#define PSA_ITS_RESET_MSK (1UL << PSA_ITS_RESET_MSK_POS)

#define PSA_PLATFORM_LC_GET_MSK_POS (4UL)
#define PSA_PLATFORM_LC_GET_MSK (1UL << PSA_PLATFORM_LC_GET_MSK_POS)
#define PSA_PLATFORM_LC_SET_MSK_POS (5UL)
#define PSA_PLATFORM_LC_SET_MSK (1UL << PSA_PLATFORM_LC_SET_MSK_POS)
#define PSA_PLATFORM_SYSTEM_RESET_MSK_POS (6UL)
#define PSA_PLATFORM_SYSTEM_RESET_MSK (1UL << PSA_PLATFORM_SYSTEM_RESET_MSK_POS)

#define PSA_CRYPTO_INIT_POS (4UL)
#define PSA_CRYPTO_INIT (1UL << PSA_CRYPTO_INIT_POS)
#define PSA_MAC_POS (5UL)
#define PSA_MAC (1UL << PSA_MAC_POS)
#define PSA_HASH_POS (6UL)
#define PSA_HASH (1UL << PSA_HASH_POS)
#define PSA_ASYMMETRIC_POS (7UL)
#define PSA_ASYMMETRIC (1UL << PSA_ASYMMETRIC_POS)
#define PSA_SYMMETRIC_POS (8UL)
#define PSA_SYMMETRIC (1UL << PSA_SYMMETRIC_POS)
#define PSA_AEAD_POS (9UL)
#define PSA_AEAD (1UL << PSA_AEAD_POS)
#define PSA_KEY_MNG_POS (10UL)
#define PSA_KEY_MNG (1UL << PSA_KEY_MNG_POS)
#define PSA_RNG_POS (11UL)
#define PSA_RNG (1UL << PSA_RNG_POS)
#define PSA_CRYPTO_FREE_POS (12UL)
#define PSA_CRYPTO_FREE (1UL << PSA_CRYPTO_FREE_POS)
#define PSA_GENERATOR_POS (13UL)
#define PSA_GENERATOR (1UL << PSA_GENERATOR_POS)
#define PSA_ENTROPY_INJECT_POS (14UL)
#define PSA_ENTROPY_INJECT (1UL << PSA_ENTROPY_INJECT_POS)


#endif