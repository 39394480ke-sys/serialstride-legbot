#include "serial_logger.h"

#include <stddef.h>
#include <string.h>

#include "usbd_cdc_if.h"

void serial_logger_init(SerialLogger *logger)
{
    if (logger == NULL) return;
    memset(logger, 0, sizeof(*logger));
    motion_log_queue_init(&logger->queue);
}

bool serial_logger_write(SerialLogger *logger, const char *record)
{
    if (logger == NULL || record == NULL) return false;
    if (!motion_log_queue_push(&logger->queue, record, strlen(record))) {
        logger->dropped++;
        return false;
    }
    return true;
}

void serial_logger_service(SerialLogger *logger)
{
    const uint8_t *record;
    uint16_t length;
    uint8_t result;

    if (logger == NULL ||
        !motion_log_queue_peek(&logger->queue, &record, &length))
        return;
    memcpy(logger->tx_buffers[logger->tx_buffer_index], record, length);
    result = CDC_Transmit_HS(
        logger->tx_buffers[logger->tx_buffer_index], length);
    if (result == USBD_BUSY) return;
    motion_log_queue_finish_attempt(&logger->queue, result == USBD_OK,
                                    &logger->dropped);
    if (result == USBD_OK) logger->tx_buffer_index ^= 1u;
}

uint32_t serial_logger_dropped(const SerialLogger *logger)
{
    return logger != NULL ? logger->dropped : 0u;
}
