#pragma once

#include <stdint.h>

#include "tock.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVER_NUM_ADC 7

typedef enum {
  SingleSample = 0,
  MultipleSample = 1,
  ContinuousSample = 2,
} ADCMode_t;

// system call interface
int adc_set_callback(subscribe_cb callback, void* callback_args);
int adc_set_buffer(uint16_t* buffer, uint32_t len);
int adc_set_double_buffer(uint16_t* buffer, uint32_t len);
bool adc_is_present(void);
int adc_channel_count(void);
int adc_single_sample(uint8_t channel);
int adc_multiple_sample(uint8_t channel, uint32_t frequency);
int adc_continuous_sample(uint8_t channel, uint32_t frequency);
int adc_stop_sampling(void);

// synchronous calls
int adc_sample_sync(uint8_t channel, uint16_t* sample);
int adc_sample_buffer_sync(uint8_t channel, uint32_t frequency, uint16_t* buffer, uint32_t* length);

#ifdef __cplusplus
}
#endif
