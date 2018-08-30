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
#include "drivers/SPI.h"
#include "platform/mbed_critical.h"

#if DEVICE_SPI_ASYNCH
#include "platform/mbed_power_mgmt.h"
#endif

#if DEVICE_SPI

namespace mbed {

SPI::spi_peripheral_s SPI::_peripherals[SPI_COUNT];

SPI::SPI(PinName mosi, PinName miso, PinName sclk, PinName ssel) :
    _id(),
    _self(),
#if DEVICE_SPI_ASYNCH
    _usage(DMA_USAGE_NEVER),
    _deep_sleep_locked(false),
#endif
    _bits(8),
    _mode(SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE),
    _fmt_need_update(true),
    _msb_first(true),
    _hz(1000000),
    _write_fill(0xFF)
{
    // No lock needed in the constructor
    // get_module
   
    SPIName name = spi_get_module(mosi, miso, sclk);

    core_util_critical_section_enter();
    // lookup in a critical section if we already have it else initialize it
    for (; _id < SPI_COUNT; _id++) {
        if ((_peripherals[_id].name == name) ||
            (_peripherals[_id].name == 0)) {
            _self = &_peripherals[_id];
            break;
        }
    }
    if (_self->name == 0) {
        _self->name = name;
        // XXX: we may want to ensure that it was previously initialized with the same mosi/miso/sclk/ss pins
        spi_init(&_self->spi, false, mosi, miso, sclk, NC);
    }
    core_util_critical_section_exit();
}

void SPI::format(int bits, int mode)
{
    format(bits, (spi_mode_t)mode, true);
}

void SPI::format(uint8_t bits, spi_mode_t mode, bool msb_first)
{
    lock();
    _bits = bits;
    _mode = mode;
    _msb_first = msb_first;
    _fmt_need_update = true;
    unlock();
}

void SPI::frequency(int hz) {
    frequency((uint32_t)hz);
}
uint32_t SPI::frequency(uint32_t hz)
{
    uint32_t actual_hz;
    lock();
    _hz = hz;
    // If changing format while you are the owner then just
    // update frequency, but if owner is changed then even frequency should be
    // updated which is done by acquire.
    if (_self->owner == this) {
        actual_hz = spi_frequency(&_self->spi, _hz);
    } else {
        actual_hz = _acquire();
        _release(); // we don't actually need the spi right now.
    }
    unlock();
    return actual_hz;
}

void SPI::acquire()
{
    lock();
    _acquire();
    unlock();
}

void SPI::release()
{
    _release();
}

// Note: Private function with no locking
uint32_t SPI::_acquire()
{
    uint32_t actual_hz = 0;
    _self->busy.wait();
    if ((_self->owner != this) || _fmt_need_update) {
        spi_format(&_self->spi, _bits, _mode, _msb_first?SPI_BIT_ORDERING_MSB_FIRST:SPI_BIT_ORDERING_LSB_FIRST);
        _fmt_need_update = false;
        actual_hz = spi_frequency(&_self->spi, _hz);
        _self->owner = this;
    }
    return actual_hz;
}

void SPI::_release() {
    _self->busy.release();
}

int SPI::write(int value)
{
    lock();
    _acquire();
    uint32_t ret = 0;
    spi_transfer(&_self->spi, &value, _bits/8, &ret, _bits/8, NULL);
    _release();
    unlock();
    return ret;
}

int SPI::write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length)
{
    lock();
    _acquire();
    int ret = spi_transfer(&_self->spi, tx_buffer, tx_length, rx_buffer, rx_length, &_write_fill);
    _release();
    unlock();
    return ret;
}

void SPI::lock()
{
    _self->mutex.lock();
}

void SPI::unlock()
{
    _self->mutex.unlock();
}

void SPI::set_default_write_value(uint32_t data) {
    lock();
    _write_fill = (uint32_t)data;
    unlock();
}

#if DEVICE_SPI_ASYNCH
// this one is actually protected but it's part of the template resolution.
int SPI::transfer(const void *tx_buffer, int tx_length, void *rx_buffer, int rx_length, unsigned char bit_width, const event_callback_t &callback, int event)
{
    // the mutex will be unlocked when the last transaction of the queue finishes.
    if (_self->busy.wait(0) == 0) {
        return queue_transfer(tx_buffer, tx_length, rx_buffer, rx_length, bit_width, callback, event);
    }
    
    start_transfer(tx_buffer, tx_length, rx_buffer, rx_length, bit_width, callback, event);
    return 0;
}

void SPI::abort_transfer()
{
    spi_transfer_async_abort(&_self->spi);
}

void SPI::clear_transfer_buffer()
{
#if TRANSACTION_QUEUE_SIZE_SPI
    _self->transaction_buffer.reset();
#endif
}

void SPI::abort_all_transfers()
{
    clear_transfer_buffer();
    abort_transfer();
}

int SPI::set_dma_usage(DMAUsage usage)
{
    if (_self->busy.wait(0) == 0) {
        return -1;
    }
    _usage = usage;
    unlock();
    return  0;
}

// protected methods

int SPI::queue_transfer(const void *tx_buffer, int tx_length, void *rx_buffer, int rx_length, unsigned char bit_width, const event_callback_t &callback, int event)
{
#if TRANSACTION_QUEUE_SIZE_SPI
    transaction_t t;

    t.tx_buffer = const_cast<void *>(tx_buffer);
    t.tx_length = tx_length;
    t.rx_buffer = rx_buffer;
    t.rx_length = rx_length;
    t.event = event;
    t.callback = callback;
    t.width = bit_width;
    Transaction<SPI> transaction(this, t);
    if (_self->transaction_buffer.full()) {
        return -1; // the buffer is full
    } else {
        core_util_critical_section_enter();
        _self->transaction_buffer.push(transaction);
        if (!_is_active) {
            dequeue_transaction();
        }
        core_util_critical_section_exit();
        return 0;
    }
#else
    return -1;
#endif
}

void SPI::_irq_handler(spi_t *spi, void *pthat, spi_async_event_t *event) {
    SPI *that = (SPI *)that;
    
    MBED_ASSERT(&that->_self.spi == spi);
    
    if (that->_callback) {
        that->unlock_deep_sleep();
        that->_callback.call(SPI_EVENT_ALL);
        that->lock_deep_sleep();
    }
    
    that->unlock_deep_sleep();
    that->_release();
    // a thread cannot take over here as we are in an ISR
#if TRANSACTION_QUEUE_SIZE_SPI
    if (that->_self.transaction_buffer.len() != 0) {
        that->dequeue_transaction();
    }
#endif
}

void SPI::start_transfer(const void *tx_buffer, int tx_length, void *rx_buffer, int rx_length, unsigned char bit_width, const event_callback_t &callback, int event)
{
    lock_deep_sleep();
    _acquire();
    _callback = callback;
    spi_transfer_async(&_self->spi, tx_buffer, tx_length, rx_buffer, rx_length, &_write_fill, &SPI::_irq_handler, this, _usage);
}

void SPI::lock_deep_sleep()
{
    if (!core_util_atomic_exchange_bool(&_deep_sleep_locked, true)) {
        sleep_manager_lock_deep_sleep();
    }
}

void SPI::unlock_deep_sleep()
{
    if (core_util_atomic_exchange_bool(&_deep_sleep_locked, false)) {
        sleep_manager_unlock_deep_sleep();
    }
}

#if TRANSACTION_QUEUE_SIZE_SPI

void SPI::dequeue_transaction()
{
    Transaction<SPI> t;
    if (_self->transaction_buffer.pop(t)) {
        SPI *obj = t.get_object();
        transaction_t *data = t.get_transaction();
        obj->start_transfer(data->tx_buffer, data->tx_length, data->rx_buffer, data->rx_length, data->width, data->callback, data->event);
    }
}

#endif

void SPI::irq_handler_asynch(void)
{
    // XXX: deprecated ! do not use anymore ! see ??????
}

#endif

} // namespace mbed

#endif
