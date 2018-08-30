
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_SPI_API_H
#define MBED_SPI_API_H

#include <stdint.h>
#include <stdbool.h>

#include "device.h"
#include "hal/dma_api.h"
#include "hal/buffer.h"

#if DEVICE_SPI

#ifdef __cplusplus
extern "C" {
#endif

typedef struct spi_s spi_t;

typedef struct {
    /** Minimum frequency supported must be set by target device and it will be assessed during
     *  testing.
     */
    uint32_t    minimum_frequency;
    /** Maximum frequency supported must be set by target device and it will be assessed during
     *  testing.
     */
    uint32_t    maximum_frequency;
    /** Each bit represents the corresponding word length. lsb => 1bit, msb => 32bit. */
    uint32_t    word_length;
    bool        support_slave_mode; /**< If true, the device can handle SPI slave mode. */
} spi_capabilities_t;

typedef enum _spi_mode_t {
    SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE,
    SPI_MODE_IDLE_LOW_SAMPLE_SECOND_EDGE,
    SPI_MODE_IDLE_HIGH_SAMPLE_FIRST_EDGE,
    SPI_MODE_IDLE_HIGH_SAMPLE_SECOND_EDGE,
} spi_mode_t;

typedef enum _spi_bit_ordering_t {
    SPI_BIT_ORDERING_MSB_FIRST,
    SPI_BIT_ORDERING_LSB_FIRST,
} spi_bit_ordering_t;

typedef struct _spi_async_event_t {
    // we may want to have flag for errors here
    uint32_t transfered;
} spi_async_event_t;

typedef void (*spi_async_handler_f)(spi_t *obj, void *ctx, spi_async_event_t *event);

SPIName spi_get_module(PinName MISO, PinName MOSI, PinName MCLK);
void spi_get_capabilities(SPIName name, PinName SS, spi_capabilities_t *cap);

void spi_init(spi_t *obj, bool is_slave, PinName MISO, PinName MOSI, PinName MCLK, PinName SS);
void spi_format(spi_t *obj, uint8_t bits, spi_mode_t mode, spi_bit_ordering_t bit_ordering);
uint32_t spi_frequency(spi_t *obj, uint32_t hz);
uint32_t spi_transfer(spi_t *obj, const void *tx, uint32_t tx_len, void *rx, uint32_t rx_len, const void *fill_symbol);

#ifdef DEVICE_SPI_ASYNCH
/**
 * \defgroup AsynchSPI Asynchronous SPI Hardware Abstraction Layer
 * @{
 */

bool spi_transfer_async(spi_t *obj, const void *tx, uint32_t tx_len, void *rx, uint32_t rx_len, const void *fill_symbol, spi_async_handler_f handler, void *ctx, DMAUsage hint);
void spi_transfer_async_abort(spi_t *obj);
#endif // DEVICE_SPI_ASYNCH

/**@}*/

void spi_free(spi_t *obj);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // SPI_DEVICE

#endif // MBED_SPI_API_H

/** @}*/
