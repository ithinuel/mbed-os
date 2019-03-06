/*
 * Copyright (c) 2018, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define WIFI 2
#if !defined(MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE) || \
    (MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE == WIFI && !defined(MBED_CONF_NSAPI_DEFAULT_WIFI_SSID))
#error [NOT_SUPPORTED] No network configuration found for this target.
#endif
#ifndef MBED_CONF_APP_ECHO_SERVER_ADDR
#error [NOT_SUPPORTED] Requires parameters from mbed_app.json
#endif

#include "mbed.h"
#include "mbed_trace.h"
#include "greentea_serial.h"
#include "greentea-client/test_env.h"
#include "unity/unity.h"
#include "utest.h"
#include "utest/utest_stack_trace.h"
#include "udp_tests.h"

using namespace utest::v1;

namespace {
Timer tc_bucket; // Timer to limit a test cases run time
}

#if MBED_CONF_NSAPI_SOCKET_STATS_ENABLE
mbed_stats_socket_t udp_stats[MBED_CONF_NSAPI_SOCKET_STATS_MAX_COUNT];
#endif

void drop_bad_packets(UDPSocket &sock, int orig_timeout)
{
    nsapi_error_t err;
    sock.set_timeout(0);
    while (true) {
        err = sock.recv(NULL, 0);
        if (err == NSAPI_ERROR_WOULD_BLOCK) {
            break;
        }
    }
    sock.set_timeout(orig_timeout);
}
static void _ifup()
{
    NetworkInterface *net = NetworkInterface::get_default_instance();
    nsapi_error_t err = net->connect();
    TEST_ASSERT_EQUAL(NSAPI_ERROR_OK, err);
    greentea_serial->printf("MBED: UDPClient IP address is '%s'\n", net->get_ip_address());
}

static void _ifdown()
{
    NetworkInterface::get_default_instance()->disconnect();
    greentea_serial->printf("MBED: ifdown\n");
}


nsapi_version_t get_ip_version()
{
    SocketAddress test;
    if (!test.set_ip_address(NetworkInterface::get_default_instance()->get_ip_address())) {
        return NSAPI_UNSPEC;
    }
    return test.get_ip_version();
}

void fill_tx_buffer_ascii(char *buff, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        buff[i] = (rand() % 43) + '0';
    }
}

int split2half_rmng_udp_test_time()
{
    return (udp_global::TESTS_TIMEOUT - tc_bucket.read()) / 2;
}

#if MBED_CONF_NSAPI_SOCKET_STATS_ENABLE
int fetch_stats()
{
    return SocketStats::mbed_stats_socket_get_each(&udp_stats[0], MBED_CONF_NSAPI_SOCKET_STATS_MAX_COUNT);
}
#endif

// Test setup
utest::v1::status_t greentea_setup(const size_t number_of_cases)
{
    GREENTEA_SETUP(udp_global::TESTS_TIMEOUT, "default_auto");
    _ifup();
    tc_bucket.start();
    return greentea_test_setup_handler(number_of_cases);
}

void greentea_teardown(const size_t passed, const size_t failed, const failure_t failure)
{
    tc_bucket.stop();
    _ifdown();
    return greentea_test_teardown_handler(passed, failed, failure);
}

Case cases[] = {
    Case("UDPSOCKET_OPEN_CLOSE_REPEAT", UDPSOCKET_OPEN_CLOSE_REPEAT),
    Case("UDPSOCKET_OPEN_LIMIT", UDPSOCKET_OPEN_LIMIT),
    Case("UDPSOCKET_RECV_TIMEOUT", UDPSOCKET_RECV_TIMEOUT),
    Case("UDPSOCKET_SENDTO_TIMEOUT", UDPSOCKET_SENDTO_TIMEOUT),
    Case("UDPSOCKET_OPEN_DESTRUCT", UDPSOCKET_OPEN_DESTRUCT),
    Case("UDPSOCKET_OPEN_TWICE", UDPSOCKET_OPEN_TWICE),
    Case("UDPSOCKET_BIND_PORT", UDPSOCKET_BIND_PORT),
    Case("UDPSOCKET_BIND_PORT_FAIL", UDPSOCKET_BIND_PORT_FAIL),
    Case("UDPSOCKET_BIND_ADDRESS_PORT", UDPSOCKET_BIND_ADDRESS_PORT),
    Case("UDPSOCKET_BIND_ADDRESS_NULL", UDPSOCKET_BIND_ADDRESS_NULL),
    Case("UDPSOCKET_BIND_ADDRESS_INVALID", UDPSOCKET_BIND_ADDRESS_INVALID),
    Case("UDPSOCKET_BIND_ADDRESS", UDPSOCKET_BIND_ADDRESS),
    Case("UDPSOCKET_BIND_WRONG_TYPE", UDPSOCKET_BIND_WRONG_TYPE),
    Case("UDPSOCKET_BIND_UNOPENED", UDPSOCKET_BIND_UNOPENED),
    Case("UDPSOCKET_SENDTO_INVALID", UDPSOCKET_SENDTO_INVALID),
    Case("UDPSOCKET_ECHOTEST_NONBLOCK", UDPSOCKET_ECHOTEST_NONBLOCK),
    Case("UDPSOCKET_ECHOTEST_BURST_NONBLOCK", UDPSOCKET_ECHOTEST_BURST_NONBLOCK),
    Case("UDPSOCKET_SENDTO_REPEAT", UDPSOCKET_SENDTO_REPEAT),
    Case("UDPSOCKET_ECHOTEST", UDPSOCKET_ECHOTEST),
    Case("UDPSOCKET_ECHOTEST_BURST", UDPSOCKET_ECHOTEST_BURST),
};

Specification specification(greentea_setup, cases, greentea_teardown, greentea_continue_handlers);

static void my_mutex_wait() {
    greentea_serial->lock();
}
static void my_mutex_release() {
    greentea_serial->unlock();
}

int main() {
    mbed_trace_mutex_wait_function_set(my_mutex_wait);
    mbed_trace_mutex_release_function_set(my_mutex_release);
    mbed_trace_init();
    return !Harness::run(specification);
}
