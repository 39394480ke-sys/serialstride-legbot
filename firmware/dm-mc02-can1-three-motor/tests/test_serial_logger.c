#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "serial_logger.h"
#include "usbd_cdc_if.h"

static uint8_t transmit_result;
static uint32_t transmit_calls;
static uint16_t transmitted_length;

uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length)
{
    assert(buffer != NULL);
    ++transmit_calls;
    transmitted_length = length;
    return transmit_result;
}

static void test_busy_usb_retries_without_dropping_record(void)
{
    SerialLogger logger;

    serial_logger_init(&logger);
    assert(serial_logger_write(&logger, "first\r\n"));
    transmit_result = USBD_BUSY;
    serial_logger_service(&logger);
    assert(transmit_calls == 1u);
    assert(logger.queue.count == 1u);
    assert(serial_logger_dropped(&logger) == 0u);

    transmit_result = USBD_OK;
    serial_logger_service(&logger);
    assert(transmit_calls == 2u);
    assert(transmitted_length == strlen("first\r\n"));
    assert(logger.queue.count == 0u);
    assert(serial_logger_dropped(&logger) == 0u);
}

static void test_real_usb_failure_drops_record(void)
{
    SerialLogger logger;

    serial_logger_init(&logger);
    assert(serial_logger_write(&logger, "failed\r\n"));
    transmit_result = USBD_FAIL;
    serial_logger_service(&logger);
    assert(logger.queue.count == 0u);
    assert(serial_logger_dropped(&logger) == 1u);
}

int main(void)
{
    test_busy_usb_retries_without_dropping_record();
    test_real_usb_failure_drops_record();
    return 0;
}
