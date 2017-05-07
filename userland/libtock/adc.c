#include <stdint.h>
#include <stdio.h>

#include "tock.h"
#include "adc.h"

typedef struct {
  bool fired;
  uint8_t channel;
  uint16_t sample;
  uint32_t length;
  uint16_t* buffer;
  int error;
} adc_data_t;

static void adc_cb(int callback_type,
                   int arg1,
                   int arg2,
                   void* userdata) {

  adc_data_t* result = (adc_data_t*)userdata;

  switch (callback_type) {
    case SingleSample:
      result->error = SUCCESS;
      result->channel = arg1;
      result->sample = arg2;
      break;

    case MultipleSample:
      result->error = SUCCESS;
      result->channel = (arg1 & 0xFF);
      result->length = ((arg1 >> 8) & 0xFFFFFF);
      result->buffer = (uint16_t*)arg2;
      break;

    case ContinuousSample:
      result->error = SUCCESS;
      result->channel = (arg1 & 0xFF);
      result->length = ((arg1 >> 8) & 0xFFFFFF);
      result->buffer = (uint16_t*)arg2;
      break;

    default:
      result->error = FAIL;
      break;
  }

  result->fired = true;
}

int adc_set_callback(subscribe_cb callback, void* callback_args) {
  return subscribe(DRIVER_NUM_ADC, 0, callback, callback_args);
}

int adc_set_buffer(uint16_t* buffer, uint32_t len) {
  // we "allow" byte arrays, so this is actually twice as long
  return allow(DRIVER_NUM_ADC, 0, (void*)buffer, len*2);
}

int adc_set_double_buffer(uint16_t* buffer, uint32_t len) {
  // we "allow" byte arrays, so this is actually twice as long
  return allow(DRIVER_NUM_ADC, 1, (void*)buffer, len*2);
}

bool adc_is_present(void) {
  return (command(DRIVER_NUM_ADC, 0, 0) >= 0);
}

int adc_channel_count(void) {
  return command(DRIVER_NUM_ADC, 0, 0);
}

int adc_single_sample(uint8_t channel) {
  return command(DRIVER_NUM_ADC, 1, channel);
}

int adc_multiple_sample(uint8_t channel, uint32_t frequency) {
  uint32_t chan_freq = (frequency << 8) | (channel & 0xFF);
  return command(DRIVER_NUM_ADC, 2, chan_freq);
}

int adc_continuous_sample(uint8_t channel, uint32_t frequency) {
  uint32_t chan_freq = (frequency << 8) | (channel & 0xFF);
  return command(DRIVER_NUM_ADC, 3, chan_freq);
}

int adc_stop_sampling(void) {
  return command(DRIVER_NUM_ADC, 4, 0);
}

int adc_sample_sync(uint8_t channel, uint16_t* sample) {
  int err;
  adc_data_t result = {0};
  result.fired = false;
  result.error = SUCCESS;

  err = adc_set_callback(adc_cb, (void*) &result);
  if (err < SUCCESS) return err;

  err = adc_single_sample(channel);
  if (err < SUCCESS) return err;

  // wait for callback
  yield_for(&result.fired);

  // copy over result
  *sample = result.sample;

  return result.error;
}

int adc_sample_buffer_sync(uint8_t channel, uint32_t frequency, uint16_t* buffer, uint32_t* length) {
  int err;
  adc_data_t result = {0};
  result.fired = false;
  result.error = SUCCESS;

  err = adc_set_callback(adc_cb, (void*) &result);
  if (err < SUCCESS) return err;

  err = adc_set_buffer(buffer, *length);
  if (err < SUCCESS) return err;

  err = adc_multiple_sample(channel, frequency);
  if (err < SUCCESS) return err;

  // wait for callback
  yield_for(&result.fired);

  // copy over result
  if (result.buffer != buffer) {
    return FAIL;
  }
  *length = result.length;

  return result.error;
}

