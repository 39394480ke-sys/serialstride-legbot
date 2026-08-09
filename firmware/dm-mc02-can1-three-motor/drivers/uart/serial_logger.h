#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_io.h"

typedef struct {
    MotionLogQueue queue;
    uint32_t dropped;
    uint8_t tx_buffers[2][MOTION_LOG_RECORD_CAPACITY];
    uint8_t tx_buffer_index;
} SerialLogger;

void serial_logger_init(SerialLogger *logger);
bool serial_logger_write(SerialLogger *logger, const char *record);
void serial_logger_service(SerialLogger *logger);
uint32_t serial_logger_dropped(const SerialLogger *logger);

#endif
