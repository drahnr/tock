#include <stdint.h>

#include "tock.h"
#include "adc.h"

struct adc_data {
  uint16_t reading;
  bool fired;
};

static struct adc_data result = { .fired = false };

// Internal callback for faking synchronous reads
static void adc_cb(__attribute__ ((unused)) int callback_type,
                   __attribute__ ((unused)) int channel,
                   int reading,
                   void* ud) {
  struct adc_data* data = (struct adc_data*) ud;
  data->reading = reading;
  data->fired = true;
}

int adc_set_callback(subscribe_cb callback, void* callback_args) {
    return subscribe(DRIVER_NUM_ADC, 0, callback, callback_args);
}

uint16_t adc_single_sample(uint8_t channel) {
  int err;

  result.fired = false;
  err = adc_set_callback(adc_cb, (void*) &result);
  if (err < 0) {
    return err;
  }

  err = command(DRIVER_NUM_ADC, 1, channel);
  if (err < 0) {
    return err;
  }

  // Wait for the ADC callback.
  yield_for(&result.fired);

  return result.reading;
}

