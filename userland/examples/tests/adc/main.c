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

// List of frequencies to sample at
const uint32_t FREQS[10] = {25, 100, 500, 1000, 5000, 10000, 44100, 100000, 150000, 175000};

static void test_single_samples(void) {
  uint16_t sample;

  int err = adc_sample_sync(ADC_CHANNEL, &sample);
  if (err < 0) {
    printf("Error sampling ADC: %d\n", err);

  } else {
    // 12 bit, reference = VCC/2, gain = 0.5
    // millivolts = ((sample * 2) / (2^12 - 1)) * (3.3 V / 2) * 1000
    int millivolts = (sample * 3300) / 4095;
    printf("ADC Reading: %d mV (raw: 0x%04x)\n", millivolts, sample);
  }
}

static void test_sampling_buffer(int index) {
  uint32_t length = 16;
  uint16_t buf[length];
  memset(buf, 0, length);

  printf("%lu ADC samples at %lu Hz\n", length, FREQS[index]);
  int err = adc_sample_buffer_sync(ADC_CHANNEL, FREQS[index], buf, &length);
  if (err < 0) {
    printf("Error sampling ADC: %d\n", err);

  } else {
    printf("\t[ ");
    for (uint32_t i=0; i<length; i++) {
      // convert to millivolts
      printf("%u ", (buf[i] * 3300) / 4095);
    }
    printf("]\n");
  }
}

int main(void) {
  putstr("[Tock] ADC Test\n");

  // check if ADC driver exists
  if (!adc_is_present()) {
    printf("No ADC driver!\n");
    return -1;
  }
  printf("ADC driver exists with %d channels\n", adc_channel_count());

  while (1) {

    printf("\nSingle Samples\n");
    for (uint32_t i=0; i<10; i++) {
      test_single_samples();
      delay_ms(100);
    }

    printf("\nBuffered Samples\n");
    for (uint32_t i=0; i<10; i++) {
      test_sampling_buffer(i);
      delay_ms(100);
    }
  }

  return 0;
}

