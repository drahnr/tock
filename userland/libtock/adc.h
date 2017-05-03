#pragma once

#include <stdint.h>

#include "tock.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVER_NUM_ADC 7

int adc_set_callback(subscribe_cb callback, void* callback_args);


uint16_t adc_single_sample(uint8_t channel);
//int adc_sample_buffer(uint8_t channel, uint8_t* buffer, uint32_t size);

#ifdef __cplusplus
}
#endif
