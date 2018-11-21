/*
 * Copyright (c) 2018 ARM Limited
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
#ifndef DEVICE_SPI
    #error [NOT_SUPPORTED] SPI is not supported on this platform, add 'DEVICE_SPI' definition to your platform.
#endif


#include "mbed.h"
#include <ctype.h>

#include "greentea-client/test_env.h"
#include "unity.h"
#include "utest.h"

#include "configs.h"

#if DEVICE_SPI_SLAVE
    #error [NOT_SUPPORTED] This test can only be run on device that support SPI Master AND Slave.
#endif
#if SPI_COUNT < 2
    #error [NOT_SUPPORTED] This test can only be run on device with at least two SPI peripherals.
#endif

using namespace utest::v1;

/*
 * 1. Overview
 *
 * This is the SPI communication test which verifies various SPI configurations.
 * This test transfers data between SPI master and SPI slave.
 * It also requires to wire a SPI interface as master to another SPI interface
 * as slave on the same device.
 *
 * 2. What is verified by the test?
 *
 *    a) master/slave mode,
 *    b) synchronous/asynchronous modes,
 *    c) full duplex/half duplex modes,
 *    d) symbol sizes: [1, 7, 8, 9, 15, 16, 17, 31, 32]
 *    e) clock polarity/phase,
 *    f) clock frequency,
 *    g) bit ordering during transmission,
 *    h) rx count equal and different from tx count,
 *    i) undefined rx/tx buffer (NULL values),
 *    j) internal/external ss handling by master.
 *
 * 3. Test scenarii
 *
 *    a) Test for synchronous api.
 *      i)      CS is asserted high (inactive state).
 *      ii)     Configuration is validated against device capabilities
 *      iii)    If master or slave can not handle this configuration the test case is skipped.
 *      iv)     Format configuration is set for both master and slave.
 *      v)      Frequency configuration is set on the master peripheral.
 *      vi)     Reception buffers and semaphores are reinitialized.
 *      vii)    A thread is started for the slave side.
 *      viii)   A thread is started for the master side.
 *      ix)     The master thread asserts CS to 0 (active state), performs the transfer and asserts CS back to 1.
 *      x)      The test thread (main) waits until either the semaphore is given twice or we it reaches a timeout.
 *      xi)     Master & slave rx buffers are respectively compared to slave & master tx buffers.
 *      xii)    If a buffer do not match, then the test fails.
 *      xiii)   Both SPI peripheral are freed and the next test starts.
 *    b) Test for synchronous api.
 *      If the device supports SPI_ASYNCH the following scenario is ran :
 *
 *      i)      CS is asserted high (inactive state).
 *      ii)     Configuration is validated against device capabilities
 *      iii)    If master or slave can not handle this configuration the test case is skipped.
 *      iv)     Format configuration is set for both master and slave.
 *      v)      Frequency configuration is set on the master peripheral.
 *      vi)     Reception buffers and semaphores are reinitialized.
 *      vii)    The spi_transfer_async function is called for the slave peripheral.
 *      viii)   The CS is asserted to 0 (active state)
 *      ix)     spi_trasnfer_async is called for the master peripheral.
 *      x)      The CS is asserted back to 1 (inative state).
 *      xi)     Master & slave rx buffers are respectively compared to slave & master tx buffers.
 *      xii)    If a buffer do not match, then the test fails.
 *      xiii)   Both SPI peripheral are freed and the next test starts.
 *
 * 4. Setup required to run the tests.
 *    In order to run these tests you need to wire the 4 pins for each peripheral as declared in the corresponding header file.
 *
 */

#define FREQ_200KHZ (200000)
#define FREQ_1MHZ   (1000000)
#define FREQ_2MHZ   (2000000)
#define FREQ_MIN    (0)
#define FREQ_MAX    (0xFFFFFFFF)

#define TEST_SYM_CNT 5

typedef enum
{
    FULL_DUPLEX, HALF_DUPLEX_MOSI, HALF_DUPLEX_MISO
} duplex_t;

typedef struct {
    const void *tx;
    uint32_t tx_len;
    void *rx;
    uint32_t rx_len;
    void *fill_symbol;
    DigitalOut *ss;
    uint8_t bits;
    volatile uint32_t count; // test output
    volatile bool has_error;
} test_context_t;

/* SPI test configuration. */
typedef struct
{
    uint8_t symbol_size;
    spi_mode_t mode;
    spi_bit_ordering_t bit_ordering;
    uint32_t freq_hz;
    uint32_t master_tx_cnt;
    uint32_t master_rx_cnt;
    uint32_t slave_tx_cnt;
    uint32_t slave_rx_cnt;
    bool master_tx_defined;
    bool master_rx_defined;
    bool slave_tx_defined;
    bool slave_rx_defined;
    bool auto_ss;
    duplex_t duplex;
    bool sync;
} config_test_case_t;

Semaphore join(0, 2);
Timer t;

spi_t slave = {0};
spi_t master = {0};

const uint8_t tx_master[] = "0123456789abcdefghijklmnopqrstuvwxyz=+/-*!?";
const uint8_t tx_slave[]  = "zyxwvutsrqponmlkjihgfedcba9876543210=+/-*!?";

uint8_t rx_slave[sizeof(tx_master)+1] = {0};
uint8_t rx_master[sizeof(tx_slave)+1] = {0};

/* Array witch test cases which represents different SPI configurations for testing. */
static config_test_case_t test_cases[] = {
        /* default config: 8 bit symbol\sync mode\full duplex\clock idle low\sample on the first clock edge\MSB first\100 KHz clock\automatic SS handling */
/* 00 */{8, SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE    , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* symbol size testing */
/* 01 */{1  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 02 */{7  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 03 */{9  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 04 */{15 , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 05 */{16 , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 06 */{17 , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 07 */{31 , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 08 */{32 , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* mode testing */
/* 09 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_SECOND_EDGE , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 10 */{8  , SPI_MODE_IDLE_HIGH_SAMPLE_FIRST_EDGE , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 11 */{8  , SPI_MODE_IDLE_HIGH_SAMPLE_SECOND_EDGE, SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* bit ordering testing */
/* 12 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_LSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* freq testing */
/* 13 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ, TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 14 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_2MHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 15 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_MIN   , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
/* 16 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_MAX   , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* master: TX > RX */
/* 17 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT-2, TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* master: TX < RX */
/* 18 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT-2 , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* slave: TX > RX */
/* 19 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT-2, true , true , true , true , false , FULL_DUPLEX  },
        /* slave: TX < RX */
/* 20 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT-2, TEST_SYM_CNT  , true , true , true , true , false , FULL_DUPLEX  },
        /* master tx buffer undefined */
/* 21 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , false, true , true , true , false , FULL_DUPLEX  },
        /* master rx buffer undefined */
/* 22 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , false, true , true , false , FULL_DUPLEX  },
        /* slave tx buffer undefined */
/* 23 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , false, true , false , FULL_DUPLEX  },
        /* slave rx buffer undefined */
/* 24 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , false, false , FULL_DUPLEX  },
        /* manual ss hadling by master */
/* 25 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false, FULL_DUPLEX  },
        /* half duplex mode  */
/* 26 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , HALF_DUPLEX_MISO },
/* 27 */{8  , SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , SPI_BIT_ORDERING_MSB_FIRST, FREQ_200KHZ  , TEST_SYM_CNT   , TEST_SYM_CNT  , TEST_SYM_CNT  , TEST_SYM_CNT  , true , true , true , true , false , HALF_DUPLEX_MOSI },
};

/* Function returns true if configuration is consistent with the capabilities of
 * the SPI peripheral, false otherwise. */
static bool check_capabilities(spi_capabilities_t *p_cabs, uint32_t symbol_size, bool slave, bool half_duplex)
{
    if (!(p_cabs->word_length & (1 << (symbol_size - 1))) ||
        (slave && !p_cabs->support_slave_mode) ||
        (half_duplex && !p_cabs->half_duplex)) {
        return false;
    }
    return true;
}

void toggle_ss(DigitalOut *pin, bool value) {
    if (pin != NULL) {
        *pin = value;
    }
}

void fn_slave(void *vctx) {
    test_context_t *ctx = (test_context_t *)vctx;
    //printf("fn_slave: %08x %u %08x %u\n", ctx->tx, ctx->tx_len, ctx->rx, ctx->rx_len);
    ctx->count = 0;
#if 0
    for (uint32_t i = 0; i < sizeof(tx_slave); i++) {
        ctx->count += spi_transfer(&slave, &tx_slave[i], 1, &rx_slave[i], 1, &fill_);
    }
#else 
    ctx->count = spi_transfer(&slave, ctx->tx, ctx->tx_len, ctx->rx, ctx->rx_len, ctx->fill_symbol);
#endif
    join.release();
}

void fn_master(void *vctx) {
    test_context_t *ctx = (test_context_t *)vctx;
    uint32_t factor = 1;
    if (ctx->bits > 16) {
        factor = 4;
    } else if (ctx->bits > 8) {
        factor = 2;
    }

    const uint8_t *tx = (const uint8_t *)ctx->tx;
    uint8_t *rx = (uint8_t *)ctx->rx;
    uint8_t *rx_ptr = NULL;
    uint32_t rx_len = 0;
    ctx->count = 0;

    toggle_ss(ctx->ss, false);
    for (uint32_t i = 0; i < ctx->tx_len; i++) {
        if (i < ctx->rx_len) {
            rx_ptr = &rx[i*factor];
            rx_len = 1;
        } else {
            rx_ptr = NULL;
            rx_len = 0;
        }
        ctx->count += spi_transfer(&master, &tx[i*factor], 1, rx_ptr, rx_len, ctx->fill_symbol);
        wait_ms(3);
    }
    if (ctx->tx_len < ctx->rx_len) {
        for (uint32_t i = ctx->tx_len; i < ctx->rx_len; i++) {
            ctx->count += spi_transfer(&master, NULL, 0, &rx[i*factor], 1, ctx->fill_symbol);
            wait_ms(3);
        }
    }
    toggle_ss(ctx->ss, true);
    
    join.release();
}

void slave_handler(spi_t *obj, void *vctx, spi_async_event_t *event) {
    test_context_t *ctx = (test_context_t*)vctx;
    ctx->count = event->transfered;
    ctx->has_error = event->error;
    join.release();
}
void master_handler(spi_t *obj, void *vctx, spi_async_event_t *event) {
    test_context_t *ctx = (test_context_t*)vctx;
    ctx->count = event->transfered;
    ctx->has_error = event->error;
    join.release();
}

bool assert_buffer_ok(const uint8_t *a, const uint8_t *b, uint32_t used, uint32_t len) {
    bool res = memcmp(a, b, used) == 0;
    if (!res) {
        for (uint32_t i = 0; (i < (len - used)) && !res; i += 1) {
            res &= a[used + i] == 0;
        }
    }
    return res;
}

template<typename T> bool check_buffer(const T*a, const T*b, T fill, uint32_t a_len, uint32_t b_len) {
    bool res = memcmp(a, b, a_len * sizeof(T)) == 0;
    for (uint32_t i = a_len; (i < b_len) && res; i++) {
        res &= b[i] == fill;
    }
    return res;
}

/* Function which perform transfer using specified config on the master side. */
template<typename T, const uint32_t tc_id, bool sync>
void test_transfer_master()
{
    DigitalOut *mcs_pin = NULL;
    PinName mcs = MBED_SPI_MASTER_SSEL;
    PinName mmclk = MBED_SPI_MASTER_MCLK;
    PinName mmiso = MBED_SPI_MASTER_MISO;
    PinName mmosi = MBED_SPI_MASTER_MOSI;
    PinName scs = MBED_SPI_SLAVE_SSEL;
    PinName smclk = MBED_SPI_SLAVE_MCLK;
    PinName smiso = MBED_SPI_SLAVE_MISO;
    PinName smosi = MBED_SPI_SLAVE_MOSI;
    uint32_t freq_hz = test_cases[tc_id].freq_hz;
    
    uint32_t master_symbols_clocked = 0;
    uint32_t slave_symbols_clocked = 0;
    
    spi_capabilities_t master_capab = { 0 };
    spi_capabilities_t slave_capab = { 0 };
    
    Thread *tmaster;
    Thread *tslave;
    test_context_t master_ctx = { 0 };
    test_context_t slave_ctx = { 0 };

#ifndef MAX
        // quick & dirty max impl
#define MAX(a, b) ((a>b)?(a):(b))
#endif

    // randomize tx buffers ?
    
    T *p_tx_master = (T *)tx_master;
    T *p_rx_master = (T *)rx_master;
    T *p_tx_slave = (T *)tx_slave;
    T *p_rx_slave = (T *)rx_slave;
    T master_fill = (T)0x4D4D4D4D;
    T slave_fill = (T)0x73737373;

    memset(rx_slave, 0, sizeof(rx_slave));
    memset(rx_master, 0, sizeof(rx_master));
    
    if (!test_cases[tc_id].master_tx_defined) {
        p_tx_master = NULL;
    }
    if (!test_cases[tc_id].master_rx_defined) {
        p_rx_master = NULL;
    }

    switch (test_cases[tc_id].duplex) {
        case HALF_DUPLEX_MISO:
            mmiso = NC;
            smiso = NC;
            break;
        case HALF_DUPLEX_MOSI:
            mmosi = NC;
            smosi = NC;
            break;
        default:
            
            break;
    }

    spi_get_capabilities(
            spi_get_module(mmosi, mmiso, mmclk),
            NC,
            &master_capab
    );
    spi_get_capabilities(
            spi_get_module(smosi, smiso, smclk),
            NC,
            &slave_capab
    );
    if (check_capabilities(&master_capab, test_cases[tc_id].symbol_size, false, test_cases[tc_id].duplex) &&
        check_capabilities(&slave_capab, test_cases[tc_id].symbol_size, false, test_cases[tc_id].duplex)) {
        /* Adapt min/max frequency for testing based of capabilities. */
        switch (freq_hz)
        {
            case FREQ_MIN:
                freq_hz = master_capab.minimum_frequency;
                break;
            case FREQ_MAX:
                freq_hz = master_capab.maximum_frequency;
                break;
            default:
                break;
        }
    } else {
        TEST_SKIP_MESSAGE("Configuration not supported. Skipping. \n");
        return;
    }
      
    if (test_cases[tc_id].duplex == FULL_DUPLEX) {
        master_symbols_clocked = MAX(test_cases[tc_id].master_tx_cnt, test_cases[tc_id].master_rx_cnt);
        slave_symbols_clocked = MAX(test_cases[tc_id].slave_tx_cnt, test_cases[tc_id].slave_rx_cnt);
    } else {
        master_symbols_clocked = test_cases[tc_id].master_tx_cnt + test_cases[tc_id].master_rx_cnt;
        slave_symbols_clocked = test_cases[tc_id].slave_tx_cnt + test_cases[tc_id].slave_rx_cnt;
    }
    
    /* Adapt manual/auto SS handling by master. */
    if (!test_cases[tc_id].auto_ss) {
        mcs_pin = new DigitalOut(MBED_SPI_MASTER_SSEL);
        mcs = NC;
        *mcs_pin = 1;
    }
    spi_init(&slave, true, smosi, smiso, smclk, scs);// enable slave
    spi_init(&master, false, mmosi, mmiso, mmclk, mcs); // enable master
    
    spi_format(&slave, test_cases[tc_id].symbol_size, test_cases[tc_id].mode, test_cases[tc_id].bit_ordering);
    spi_format(&master, test_cases[tc_id].symbol_size, test_cases[tc_id].mode, test_cases[tc_id].bit_ordering);

    spi_frequency(&master, test_cases[tc_id].freq_hz);

    t.reset();
    t.start();

    slave_ctx = (test_context_t){
        .tx = p_tx_slave,
        .tx_len = test_cases[tc_id].slave_tx_cnt,
        .rx = p_rx_slave,
        .rx_len = test_cases[tc_id].slave_rx_cnt,
        .fill_symbol = &slave_fill,
        .ss = NULL,
        .bits = test_cases[tc_id].symbol_size,
    };
    master_ctx = (test_context_t){
        .tx = p_tx_master,
        .tx_len = test_cases[tc_id].master_tx_cnt,
        .rx = p_rx_master,
        .rx_len = test_cases[tc_id].master_rx_cnt,
        .fill_symbol = &master_fill,
        .ss = mcs_pin,
        .bits = test_cases[tc_id].symbol_size,
    };

    if (sync) {    
        tmaster = new Thread();
        MBED_ASSERT(tmaster != NULL);
        tslave = new Thread();
        MBED_ASSERT(tslave != NULL);
        // thread 1 listen
        tslave->start(callback(fn_slave, &slave_ctx));
        // thread 2 receive
        tmaster->start(callback(fn_master, &master_ctx));
    } else { 
        spi_transfer_async(&slave,
            p_tx_slave, slave_ctx.tx_len,
            p_rx_slave, slave_ctx.rx_len,
            &slave_fill, slave_handler, &slave_ctx, DMA_USAGE_OPPORTUNISTIC); // prepare listen
        
        toggle_ss(mcs_pin, 0);
        spi_transfer_async(&master,
            p_tx_master, master_ctx.tx_len,
            p_rx_master, master_ctx.rx_len,
            &master_fill, master_handler, &master_ctx, DMA_USAGE_OPPORTUNISTIC); // prepare send
    }
    uint32_t i = 0;
    uint64_t cnt = 0;
    while ((i < 2) && (t.read_ms() < 2000)) {
        if (join.wait(0) != 0) {
            i += 1;
        }
        cnt += 1;
    }

    if (sync) {
        tmaster->terminate();
        delete tmaster;
        tslave->terminate();
        delete tslave;
    }

    t.stop();
    toggle_ss(mcs_pin, 1); // make sure ssel is cleared
    spi_free(&master); // master
    spi_free(&slave); // slave

    printf("Test context:\n");
    printf("Master: tx_len: %u tx:%.*s rx_len: %u\n", master_ctx.tx_len, master_ctx.tx_len, master_ctx.tx, master_ctx.rx_len);
    printf("Slave: tx_len: %u tx:%.*s rx_len: %u\n", slave_ctx.tx_len, slave_ctx.tx_len, slave_ctx.tx, slave_ctx.rx_len);
    printf("Ran in %d (cnt = %llu) : %f\n", t.read_ms(), cnt, (float)cnt / (float)t.read_ms());
    printf("s sent  : %.*s\n", slave_ctx.tx_len, tx_slave);
    printf("m recved: %.*s\n", master_ctx.rx_len, rx_master); // should used a "received" value here
    printf("m sent  : %.*s\n", master_ctx.tx_len, tx_master);
    printf("s recved: %.*s\n", slave_ctx.rx_len, rx_slave); // should used a "received" value here
    
    TEST_ASSERT(!master_ctx.has_error);
    TEST_ASSERT(!slave_ctx.has_error);
    TEST_ASSERT_EQUAL(master_symbols_clocked, master_ctx.count);
    TEST_ASSERT_EQUAL(slave_symbols_clocked, slave_ctx.count);

#ifndef MIN
#define MIN(a, b) (((a)>(b))?(b):(a))
#endif
       
    if ((p_tx_slave != NULL) && (p_rx_master != NULL)) {
        uint32_t n = MIN(slave_ctx.tx_len, master_ctx.rx_len);
        TEST_ASSERT_EQUAL(1, check_buffer((T*)slave_ctx.tx, p_rx_master, slave_fill, n, master_ctx.rx_len));
    }
    if ((p_tx_master != NULL) && (p_rx_slave != NULL)) {
        uint32_t n = MIN(slave_ctx.rx_len, master_ctx.tx_len);
        TEST_ASSERT_EQUAL(1, check_buffer((T*)master_ctx.tx, p_rx_slave, master_fill, n, slave_ctx.rx_len));
    }
    printf("---\n");
    if (mcs_pin != NULL) {
        delete mcs_pin;
    }
}

utest::v1::status_t test_setup(const size_t number_of_cases)
{
    GREENTEA_SETUP(40, "default_auto");
    return verbose_test_setup_handler(number_of_cases);
}

Case cases[] = {
    Case("SPI master-slave sync com - default config", test_transfer_master<uint8_t, 0, true>),
    Case("SPI master-slave sync com - symbol size: 1", test_transfer_master<uint8_t, 1, true>),
    Case("SPI master-slave sync com - symbol size: 7", test_transfer_master<uint8_t, 2, true>),
    Case("SPI master-slave sync com - symbol size: 9", test_transfer_master<uint16_t, 3, true>),
    Case("SPI master-slave sync com - symbol size: 15", test_transfer_master<uint16_t, 4, true>),
    Case("SPI master-slave sync com - symbol size: 16", test_transfer_master<uint16_t, 5, true>),
    Case("SPI master-slave sync com - symbol size: 17", test_transfer_master<uint32_t, 6, true>),
    Case("SPI master-slave sync com - symbol size: 31", test_transfer_master<uint32_t, 7, true>),
    Case("SPI master-slave sync com - symbol size: 32", test_transfer_master<uint32_t, 8, true>),
    Case("SPI master-slave sync com - mode: idle low, sample second edge", test_transfer_master<uint8_t, 9, true>),
    Case("SPI master-slave sync com - mode: idle high, sample first edge", test_transfer_master<uint8_t, 10, true>),
    Case("SPI master-slave sync com - mode: idle high, sample second edge", test_transfer_master<uint8_t, 11, true>),
    Case("SPI master-slave sync com - bit ordering: LSB first", test_transfer_master<uint8_t, 12, true>),
    Case("SPI master-slave sync com - freq testing: 200 KHz", test_transfer_master<uint8_t, 13, true>),
    Case("SPI master-slave sync com - freq testing: 2 MHz", test_transfer_master<uint8_t, 14, true>),
    Case("SPI master-slave sync com - freq testing: min defined", test_transfer_master<uint8_t, 15, true>),
    Case("SPI master-slave sync com - freq testing: max defined", test_transfer_master<uint8_t, 16, true>),
    Case("SPI master-slave sync com - master: TX > RX", test_transfer_master<uint8_t, 17, true>),
    Case("SPI master-slave sync com - master: TX < RX", test_transfer_master<uint8_t, 18, true>),
    Case("SPI master-slave sync com - slave: TX > RX", test_transfer_master<uint8_t, 19, true>),
    Case("SPI master-slave sync com - slave: TX < RX", test_transfer_master<uint8_t, 20, true>),
    Case("SPI master-slave sync com - master: TX undefined", test_transfer_master<uint8_t, 21, true>),
    Case("SPI master-slave sync com - master: RX undefined", test_transfer_master<uint8_t, 22, true>),
    Case("SPI master-slave sync com - slave: TX undefined", test_transfer_master<uint8_t, 23, true>),
    Case("SPI master-slave sync com - slave: RX undefined", test_transfer_master<uint8_t, 24, true>),
    Case("SPI master-slave sync com - master: manual ss", test_transfer_master<uint8_t, 25, true>),
//    Case("SPI master-slave sync com - half duplex (MOSI)", test_transfer_master<uint8_t, 26, true>),
//    Case("SPI master-slave sync com - half duplex (MISO)", test_transfer_master<uint8_t, 27, true>),
#ifdef DEVICE_SPI_ASYNCH
    Case("SPI master-slave async com - default config", test_transfer_master<uint8_t, 0, false>),
    Case("SPI master-slave async com - symbol size: 1", test_transfer_master<uint8_t, 1, false>),
    Case("SPI master-slave async com - symbol size: 7", test_transfer_master<uint8_t, 2, false>),
    Case("SPI master-slave async com - symbol size: 9", test_transfer_master<uint16_t, 3, false>),
    Case("SPI master-slave async com - symbol size: 15", test_transfer_master<uint16_t, 4, false>),
    Case("SPI master-slave async com - symbol size: 16", test_transfer_master<uint16_t, 5, false>),
    Case("SPI master-slave async com - symbol size: 17", test_transfer_master<uint32_t, 6, false>),
    Case("SPI master-slave async com - symbol size: 31", test_transfer_master<uint32_t, 7, false>),
    Case("SPI master-slave async com - symbol size: 32", test_transfer_master<uint32_t, 8, false>),
    Case("SPI master-slave async com - mode: idle low, sample second edge", test_transfer_master<uint8_t, 9, false>),
    Case("SPI master-slave async com - mode: idle high, sample first edge", test_transfer_master<uint8_t, 10, false>),
    Case("SPI master-slave async com - mode: idle high, sample second edge", test_transfer_master<uint8_t, 11, false>),
    Case("SPI master-slave async com - bit ordering: LSB first", test_transfer_master<uint8_t, 12, false>),
    Case("SPI master-slave async com - freq testing: 200 KHz", test_transfer_master<uint8_t, 13, false>),
    Case("SPI master-slave async com - freq testing: 2 MHz", test_transfer_master<uint8_t, 14, false>),
    Case("SPI master-slave async com - freq testing: min defined", test_transfer_master<uint8_t, 15, false>),
//    Case("SPI master-slave async com - freq testing: max defined", test_transfer_master<uint8_t, 16, false>),
    Case("SPI master-slave async com - master: TX > RX", test_transfer_master<uint8_t, 17, false>),
    Case("SPI master-slave async com - master: TX < RX", test_transfer_master<uint8_t, 18, false>),
    Case("SPI master-slave async com - slave: TX > RX", test_transfer_master<uint8_t, 19, false>),
    Case("SPI master-slave async com - slave: TX < RX", test_transfer_master<uint8_t, 20, false>),
    Case("SPI master-slave async com - master: TX undefined", test_transfer_master<uint8_t, 21, false>),
    Case("SPI master-slave async com - master: RX undefined", test_transfer_master<uint8_t, 22, false>),
    Case("SPI master-slave async com - slave: TX undefined", test_transfer_master<uint8_t, 23, false>),
    Case("SPI master-slave async com - slave: RX undefined", test_transfer_master<uint8_t, 24, false>),
    Case("SPI master-slave async com - master: manual ss", test_transfer_master<uint8_t, 25, false>),
//    Case("SPI master-slave async com - half duplex (MOSI)", test_transfer_master<uint8_t, 26, false>),
//    Case("SPI master-slave async com - half duplex (MISO)", test_transfer_master<uint8_t, 27, false>),
#endif
};

Specification specification(test_setup, cases);

int main() {
    return !Harness::run(specification);
}
