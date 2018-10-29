/*
 * Copyright (c) 2018 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define DEFAULT_TEST_SYM_CNT 5
#define SHORTER_TEST_SYM_CNT 3
#define SHORTEST_TEST_SYM_CNT 1
#define LONG_TEST_SYM_CNT 64

#define FREQ_200KHZ (200000)
#define FREQ_1MHZ   (1000000)
#define FREQ_2MHZ   (2000000)
#define FREQ_MIN    (0)
#define FREQ_MAX    (-1)
#define US_PER_MS   (1000)
#define US_PER_S    (1000000)

#define MASTER_SYNC_BIT_MASK 1
#define SLAVE_SYNC_BIT_MASK 2

#if IS_MASTER
#define SPI_MISO MBED_CONF_APP_SPI_MASTER_MISO
#define SPI_MOSI MBED_CONF_APP_SPI_MASTER_MOSI
#define SPI_CLK MBED_CONF_APP_SPI_MASTER_CLK
#define SPI_SS MBED_CONF_APP_SPI_MASTER_CS
#define TRANSMISSION_DELAY_MS MBED_CONF_APP_SPI_MASTER_DELAY
#define TRANSMISSION_BUTTON MBED_CONF_APP_SPI_MASTER_TRANSMISSION_START_BTN
#define TRANSMISSION_LED MBED_CONF_APP_SPI_MASTER_TRANSMISSION_START_LED
#define DEBUG MBED_CONF_APP_SPI_MASTER_DEBUG
#else
#define SPI_MISO MBED_CONF_APP_SPI_SLAVE_MISO
#define SPI_MOSI MBED_CONF_APP_SPI_SLAVE_MOSI
#define SPI_CLK MBED_CONF_APP_SPI_SLAVE_CLK
#define SPI_SS MBED_CONF_APP_SPI_SLAVE_CS
#define TRANSMISSION_DELAY_MS MBED_CONF_APP_SPI_SLAVE_DELAY
#define TRANSMISSION_DELAY_MASTER_MS MBED_CONF_APP_SPI_MASTER_DELAY
#define TRANSMISSION_BUTTON MBED_CONF_APP_SPI_SLAVE_TRANSMISSION_START_BTN
#define TRANSMISSION_LED MBED_CONF_APP_SPI_SLAVE_TRANSMISSION_START_LED
#define DEBUG MBED_CONF_APP_SPI_SLAVE_DEBUG
#endif

/* Duplex modes for testing. */
typedef enum
{
    FULL_DUPLEX,
    HALF_DUPLEX_MOSI,
    HALF_DUPLEX_MISO
} duplex_t;

/* Buffer cases for testing. */
typedef enum
{
    SPI_BUFFERS_EQUAL,
    SPI_BUFFERS_MASTER_TX_GT_RX,
    SPI_BUFFERS_MASTER_TX_LT_RX,
    SPI_BUFFERS_SLAVE_TX_GT_RX,
    SPI_BUFFERS_SLAVE_TX_LT_RX,
    SPI_BUFFERS_SHORTEST,
    SPI_BUFFERS_LONG
} buffers_t;

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
    bool master_sync;
    bool slave_sync;
} config_test_case_t;

/* Transmission thread parameters. */
typedef struct
{
    spi_t * obj;
    config_test_case_t *config;
    DigitalOut *ss;
    int status;
} trans_thread_params_t;

template<typename T>
void init_transmission_buffers(config_test_case_t *config, T *p_tx_pattern, T *p_rx1_pattern, T *p_rx2_pattern,
        T *p_tx_buff, T *p_rx_buff, T *p_fill_symbol);

/* Total number of transmitted symbols - provided by the configuration. */
static uint32_t sym_count = 0;

/* Flag which indicates if spi transfer test has been finished. */
static volatile bool transfer_finished;

/* Semaphore used to delay main task and execute transfer thread. */
static Semaphore transm_sem(0, 1);

/* Timeout handler for the transfer thread. */
static void transm_timeout_handler()
{
    transm_sem.release();
}

/* Function inserts symbol string into the message. */
static void msg_insert_symbol(uint8_t *buffer, uint8_t *symbol_str)
{
    sprintf((char*) buffer, " %s", (char*) symbol_str);
}

/* Function builds message based on array with symbols. */
template<typename T>
static void mgs_init(uint8_t *buffer, T *symbols, const char *dut, const char *buf_name)
{
    uint8_t *p_buffer = buffer;
    uint8_t symbol[32];

    sprintf((char*) buffer, "%s %s", dut, buf_name);
    p_buffer += strlen((const char*) buffer);

    for (uint32_t i = 0; i < sym_count; i++) {
        sprintf((char*) symbol, "0x%X", symbols[i]);

        msg_insert_symbol(p_buffer, symbol);
        p_buffer += (strlen((const char*) symbol) + 1);
    }

    sprintf((char*) p_buffer, "%s", "\r\n");
}

/* Debug function to dump tx/rx buffers. */
template<typename T>
static void dump_buffers(T *tx_pattern, T *rx1_pattern, T *rx2_pattern, T *tx_buff, T *rx_buff)
{
    uint8_t buffer[1024];

#if DEBUG
    typedef unsigned int u32;
#if IS_MASTER
    /* Wait a while so the messages from master and slave are not mixed. */
    wait_ms(100);
    const char * dut = "master";
#else
    const char * dut = "slave ";
#endif

    printf("%s - buffers dump: \r\n", dut);
    mgs_init<T>(buffer, rx1_pattern, dut, "rx1 pattern:");
    printf((const char*)buffer);
    mgs_init<T>(buffer, rx2_pattern, dut, "rx2 pattern:");
    printf((const char*)buffer);
    mgs_init<T>(buffer, tx_buff, dut, "tx buffer:  ");
    printf((const char*)buffer);
    mgs_init<T>(buffer, rx_buff, dut, "rx buffer:  ");
    printf((const char*)buffer);
#endif
}

/* Debug function to print configuration details. */
void dump_config(config_test_case_t *config)
{
#if DEBUG
    printf("TEST CONFIGURATION\r\n");
    printf("symbol_size:       %lu\r\n", (uint32_t) config->symbol_size);
    printf("spi_mode:          %lu\r\n", (uint32_t) config->mode);
    printf("bit_ordering:      %lu\r\n", (uint32_t) config->bit_ordering);
    printf("freq:              %lu\r\n", (uint32_t) config->freq_hz);
    printf("master tx cnt:     %lu\r\n", (uint32_t) config->master_tx_cnt);
    printf("master rx cnt:     %lu\r\n", (uint32_t) config->master_rx_cnt);
    printf("slave tx cnt:      %lu\r\n", (uint32_t) config->slave_tx_cnt);
    printf("slave rx cnt:      %lu\r\n", (uint32_t) config->slave_rx_cnt);
    printf("master tx defined: %lu\r\n", (uint32_t) config->master_tx_defined);
    printf("master rx defined: %lu\r\n", (uint32_t) config->master_rx_defined);
    printf("slave tx defined:  %lu\r\n", (uint32_t) config->slave_tx_defined);
    printf("slave rx defined:  %lu\r\n", (uint32_t) config->slave_rx_defined);
    printf("auto ss:           %lu\r\n", (uint32_t) config->auto_ss);
    printf("duplex:            %lu\r\n", (uint32_t) config->duplex);
    printf("master sync mode:  %lu\r\n", (uint32_t) config->master_sync);
    printf("slave sync mode:   %lu\r\n", (uint32_t) config->slave_sync);
#endif
}

/* Function inits specified buffer with given pattern. */
static void set_buffer(void * addr, uint32_t size, uint8_t val)
{
    char *p_char = (char*) addr;

    for (uint32_t i = 0; i < size; i++) {
        p_char[i] = val;
    }
}

/* Function inits array of symbols with given pattern.
 *
 * Example:
 * 12 bytes buffer filled with 16 bit (2 bytes) symbols and pattern 0xAA 0xBB 0xCC
 * will be build as follows:
 * 0xAAAA 0cBBBB 0xCCCC 0xAAAA 0xBBBB 0xCCCC
 *
 */
static void init_buffer_with_pattern(void * p_buffer, uint32_t sym_count, uint32_t sym_size, uint8_t * p_pattern,
        uint32_t pattern_sym_count)
{
    uint32_t pattern_idx = 0;

    while (sym_count) {
        set_buffer(p_buffer, sym_size, p_pattern[pattern_idx]);

        p_buffer += sym_size;

        pattern_idx++;
        if (pattern_idx == pattern_sym_count) {
            pattern_idx = 0;
        }

        sym_count--;
    }
}

/* Function handles ss line if ss is specified. */
static void handle_ss(DigitalOut * ss, bool select)
{
    if (ss) {
        if (select) {
            *ss = 0;
        } else {
            *ss = 1;
        }
    }
}

#ifdef DEVICE_SPI_ASYNCH
/* Callback function for SPI async transfers. */
static uint32_t context;
static volatile bool async_transm_done;
static volatile uint32_t async_transfered;
void spi_async_callback(spi_t *obj, void *ctx, spi_async_event_t *event) {
    async_transm_done = true;
    async_transfered = event->transfered;
}
#endif

/* Function returns true if configuration is consistent with the capabilities of
 * the SPI peripheral, false otherwise. */
static int check_capabilities(uint32_t symbol_size, bool slave, bool half_duplex, bool sync_mode)
{
    spi_capabilities_t capabilities = { 0 };
#if IS_MASTER
    spi_get_capabilities(spi_get_module(SPI_MOSI, SPI_MISO, SPI_CLK), NC, &capabilities);
#else
    spi_get_capabilities(spi_get_module(SPI_MOSI, SPI_MISO, SPI_CLK), SPI_SS, &capabilities);
#endif

    if (!(capabilities.word_length & (1 << (symbol_size - 1))) || (slave && !capabilities.support_slave_mode)
            || (half_duplex && !capabilities.half_duplex)
#ifndef DEVICE_SPI_ASYNCH
            || (!sync_mode)
#endif
            ) {
#if IS_MASTER
        printf("SKIP: Configuration not supported by master.\r\n");
#else
        printf("SKIP: Configuration not supported by slave.\r\n");
#endif
    }

    return CMDLINE_RETCODE_SUCCESS;
}

/* Function used to perform transmission using sync or async modes. */
static uint32_t sync_async_transfer(spi_t *obj, const void *tx, uint32_t tx_len, void *rx, uint32_t rx_len,
        const void *fill_symbol, bool sync_mode)
{
    uint32_t count = 0;

    if (sync_mode) {
        count = spi_transfer(obj, tx, tx_len, rx, rx_len, fill_symbol);
    }
#ifdef DEVICE_SPI_ASYNCH
    else {
        async_transm_done = false;
        async_transfered = 0;

        spi_transfer_async(obj, tx, tx_len, rx, rx_len, fill_symbol, spi_async_callback, &context, DMA_USAGE_NEVER);

        /* Wait here for the end of the transmission. */
        while(!async_transm_done) {
        }

        count = async_transfered;
    }
#endif

    return count;
}

/* Function waits before the transmission.
 * It takes params from the app configuration. If TRANSMISSION_DELAY_MS is 0 and
 * TRANSMISSION_BUTTON is specified then it is used to trigger the transmission
 * (wait until button is pressed).
 * TRANSMISSION_LED can be specified to indicate waiting. This features might be useful while
 * driver development and debugging.
 */
static void wait_before_transmission()
{
    DigitalOut *tr_led = NULL;

    if (TRANSMISSION_LED != NC) {
        tr_led = new DigitalOut(TRANSMISSION_LED);
        *tr_led = 0;
    }

    if (TRANSMISSION_DELAY_MS) {
        wait_ms (TRANSMISSION_DELAY_MS);
    } else {
        DigitalIn button(TRANSMISSION_BUTTON);

        while (button.read() == 1)
            ;
        while (button.read() == 0)
            ;
    }

    if (tr_led) {
        *tr_led = 1;
    }
}

/* Function compares given buffers and returns true when equal, false otherwise.
 * In case when buffer is undefined (NULL) function returns true.
 */
static bool check_buffers(void *p_pattern, void *p_buffer, uint32_t size)
{
    const char * p_byte_pattern = (const char*) p_pattern;
    const char * p_byte_buffer = (const char*) p_buffer;

    if (p_buffer == NULL || p_pattern == NULL) {
        return true;
    }

    while (size) {
        if (*p_byte_pattern != *p_byte_buffer) {
            return false;
        }
        p_byte_pattern++;
        p_byte_buffer++;
        size--;
    }

    return true;
}

/* Function which perform transfer using specified config.
 *
 * Note:
 *  SS for slave is NULL.
 *  SS for master must is NULL when ss is handled automatically or defined in case of
 *  manual handling.
 *
 */
template<typename T>
static int perform_transfer(spi_t *obj, config_test_case_t *config, DigitalOut *ss)
{
    uint32_t count;
    bool test_passed;
    bool sync;
    int status = CMDLINE_RETCODE_SUCCESS;

    uint32_t clocked_symbols_1 = sym_count;
    uint32_t clocked_symbols_2 = sym_count;

    if (config->duplex != FULL_DUPLEX) {
#if IS_MASTER
        clocked_symbols_1 = (config->master_tx_cnt + config->master_rx_cnt);
#else
        clocked_symbols_1 = (config->slave_tx_cnt + config->slave_rx_cnt);
#endif
        clocked_symbols_2 = (sym_count + sym_count);
    }

    T tx_pattern[sym_count];
    T rx1_pattern[sym_count];
    T rx2_pattern[sym_count];
    T tx_buff[sym_count];
    T rx_buff[sym_count];
    T fill_symbol;

    void *p_tx_buff = tx_buff;
    void *p_rx_buff = rx_buff;

#if IS_MASTER
    if (!config->master_tx_defined) {
        p_tx_buff = NULL;
    }

    if (!config->master_rx_defined) {
        p_rx_buff = NULL;
    }
    sync = config->master_sync;
#else
    if (!config->slave_tx_defined) {
        p_tx_buff = NULL;
    }

    if (!config->slave_rx_defined) {
        p_rx_buff = NULL;
    }
    sync = config->slave_sync;
#endif

    init_transmission_buffers<T>(config, &tx_pattern[0], &rx1_pattern[0], &rx2_pattern[0], &tx_buff[0], &rx_buff[0],
            &fill_symbol);

    test_passed = true;
    wait_before_transmission();
    handle_ss(ss, true);
#if IS_MASTER
    count = sync_async_transfer(obj, p_tx_buff, config->master_tx_cnt, p_rx_buff, config->master_rx_cnt, (void*) &fill_symbol, sync);
#else
    count = sync_async_transfer(obj, p_tx_buff, config->slave_tx_cnt, p_rx_buff, config->slave_rx_cnt,
            (void*) &fill_symbol, sync);
#endif
    handle_ss(ss, false);

    if (!check_buffers(rx1_pattern, p_rx_buff, sizeof(T) * sym_count)) {
#if IS_MASTER
        printf("ERROR (T1): Master RX buffer invalid. \r\n ");
#else
        printf("ERROR (T1): Slave RX buffer invalid. \r\n ");
#endif
        test_passed = false;
    }

    if (!check_buffers(tx_pattern, p_tx_buff, sizeof(T) * sym_count)) {
#if IS_MASTER
        printf("ERROR (T1): Master TX buffer invalid. \r\n ");
#else
        printf("ERROR (T1): Slave TX buffer invalid. \r\n ");
#endif
        test_passed = false;
    }

    if (clocked_symbols_1 != count) {
#if IS_MASTER
        printf("ERROR (T1): Master Clocked symbol count invalid (expected: %d actual: %d). \r\n ", clocked_symbols_1, count);
#else
        printf("ERROR (T1): Slave Clocked symbol count invalid (expected: %d actual: %d). \r\n ", clocked_symbols_1,
                count);
#endif
        test_passed = false;
    }

    if (!test_passed) {
        dump_buffers<T>(tx_pattern, rx1_pattern, rx2_pattern, tx_buff, rx_buff);
    }

    /* Init TX buffer with data received from slave if possible. */

    if (p_tx_buff && p_rx_buff) {
        memcpy(p_tx_buff, p_rx_buff, sizeof(tx_buff));
        memcpy(tx_pattern, p_rx_buff, sizeof(tx_buff));
    }

    set_buffer(rx_buff, sizeof(rx_buff), 0x00);

    test_passed = true;
    wait_before_transmission();
    handle_ss(ss, true);
    count = sync_async_transfer(obj, p_tx_buff, sym_count, p_rx_buff, sym_count, (void*) &fill_symbol, sync);
    handle_ss(ss, false);

    if (!check_buffers(rx2_pattern, p_rx_buff, sizeof(T) * sym_count)) {
#if IS_MASTER
        printf("ERROR (T2): Master RX buffer invalid. \r\n ");
#else
        printf("ERROR (T2): Slave RX buffer invalid. \r\n ");
#endif
        test_passed = false;
    }

    if (!check_buffers(tx_pattern, p_tx_buff, sizeof(T) * sym_count)) {
#if IS_MASTER
        printf("ERROR (T2): Master TX buffer invalid. \r\n ");
#else
        printf("ERROR (T2): Slave TX buffer invalid. \r\n ");
#endif
        test_passed = false;
    }

    if (clocked_symbols_2 != count) {
#if IS_MASTER
        printf("ERROR (T2): Master Clocked symbol count invalid (expected: %lu actual: %lu). \r\n ", clocked_symbols_2, count);
#else
        printf("ERROR (T2): Slave Clocked symbol count invalid (expected: %lu actual: %lu). \r\n ", clocked_symbols_2,
                count);
#endif
        test_passed = false;
    }

    if (!test_passed) {
        dump_buffers<T>(tx_pattern, rx1_pattern, rx2_pattern, tx_buff, rx_buff);
    }

    return status;
}

/* Auxiliary thread which is used to handle transmission.
 * In case when transmission is not finished after specified timeout, then
 * this thread is terminated and test failure/timeout is indicated.
 */
void transfer_thread(void * thread_params)
{
    int status = CMDLINE_RETCODE_SUCCESS;
    trans_thread_params_t *params = (trans_thread_params_t*) thread_params;

    if (params->config->symbol_size <= 8) {
        status = perform_transfer<uint8_t>(params->obj, params->config, params->ss);
    } else if (params->config->symbol_size <= 16) {
        status = perform_transfer<uint16_t>(params->obj, params->config, params->ss);
    } else {
        status = perform_transfer<uint32_t>(params->obj, params->config, params->ss);
    }

    transfer_finished = true;

    params->status = status;

    transm_sem.release();
}
