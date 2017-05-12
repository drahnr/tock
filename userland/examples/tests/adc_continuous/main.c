#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <tock.h>
#include <console.h>
#include <timer.h>
#include <adc.h>

// Sample the first channel. On Hail, this is external pin A0 (AD0)
#define ADC_CHANNEL 0

// Sampling frequencies
#define ADC_LOWSPEED_FREQUENCY 10
#define ADC_HIGHSPEED_FREQUENCY 44100

// Buffer size
// Given a sampling frequency, we will receive callbacks every
// BUF_SIZE/FREQ seconds. At 44100 Hz and 4410 samples, this is a callback
// every 100 ms
#define BUF_SIZE 4410

// data buffers
static uint16_t sample_buffer1[BUF_SIZE] = {0};
static uint16_t sample_buffer2[BUF_SIZE] = {0};

// state
static uint8_t counter = 0;

static void adc_cb(int callback_type,
    int arg1,
    int arg2,
    __attribute__ ((unused)) void* callback_args) {

  if (callback_type == ContinuousSample) {
    // single ADC sample is ready
    uint8_t channel = arg1 & 0xFF;
    uint16_t sample = (uint16_t)(arg2 * 3300 / 4095);

    printf("Channel: %u\tValue: %u\n", channel, sample);

  } else if (callback_type == ContinuousBuffer) {
    // buffer of ADC samples is ready

    // parse out arguments
    uint8_t channel = arg1 & 0xFF;
    uint32_t length = (arg1 >> 8) & 0xFFFFFF;
    uint16_t* buf_ptr = (uint16_t*)arg2;

    // calculate and print statistics about the data
    uint32_t sum = 0;
    uint16_t min = 0xFFFF;
    uint16_t max = 0;
    for (uint32_t i=0; i<length; i++) {
      uint16_t sample = (buf_ptr[i] * 3300 / 4095);
      sum += sample;
      if (sample < min) {
        min = sample;
      }
      if (sample > max) {
        max = sample;
      }
    }
    printf("Channel: %u\tCount: %d\tAvg: %lu\tMin: %u\tMax: %u\n",
        channel, BUF_SIZE, sum/BUF_SIZE, min, max);

  } else {
    printf("Bad callback type\n");
  }

  // switch between single and buffered sampling
  counter++;
  if (counter == 10) {
      // stop single sampling
      int err = adc_stop_sampling();
      if (err < SUCCESS) {
        printf("Failed to stop sampling: %d\n", err);
        return;
      }

      // start buffered sampling
      printf("Beginning buffered sampling on channel %d at %d Hz\n",
          ADC_CHANNEL, ADC_HIGHSPEED_FREQUENCY);
      err = adc_continuous_buffered_sample(ADC_CHANNEL, ADC_HIGHSPEED_FREQUENCY);
      if (err < SUCCESS) {
        printf("continuous buffered sample error: %d\n", err);
        return;
      }

  } else if (counter == 20) {
    // stop buffered sampling
    counter = 0;
    int err = adc_stop_sampling();

    // start single sampling
    printf("Beginning continuous sampling on channel %d at %d Hz\n",
        ADC_CHANNEL, ADC_LOWSPEED_FREQUENCY);
    err = adc_continuous_sample(ADC_CHANNEL, ADC_LOWSPEED_FREQUENCY);
    if (err < SUCCESS) {
      printf("continuous sample error: %d\n", err);
      return;
    }
  }
}

int main(void) {
  int err;
  putstr("[Tock] ADC Continuous Test\n");

  // check if ADC driver exists
  if (!adc_is_present()) {
    printf("No ADC driver!\n");
    return -1;
  }
  printf("ADC driver exists with %d channels\n", adc_channel_count());

  // set ADC callback
  err = adc_set_callback(adc_cb, NULL);
  if (err < SUCCESS) {
    printf("set callback error: %d\n", err);
    return -1;
  }

  // set main buffer for ADC samples
  err = adc_set_buffer(sample_buffer1, BUF_SIZE);
  if (err < SUCCESS) {
    printf("set buffer error: %d\n", err);
    return -1;
  }

  // set secondary buffer for ADC samples. In continuous mode, the ADC will
  // automatically switch between the two each callback
  err = adc_set_double_buffer(sample_buffer2, BUF_SIZE);
  if (err < SUCCESS) {
    printf("set double buffer error: %d\n", err);
    return -1;
  }

  // begin continuous sampling
  printf("Beginning continuous sampling on channel %d at %d Hz\n",
      ADC_CHANNEL, ADC_LOWSPEED_FREQUENCY);
  err = adc_continuous_sample(ADC_CHANNEL, ADC_LOWSPEED_FREQUENCY);
  if (err < SUCCESS) {
    printf("continuous sample error: %d\n", err);
    return -1;
  }

  // return successfully. The system automatically calls `yield` continuously
  // for us
  return 0;
}

