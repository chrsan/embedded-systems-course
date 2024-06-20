#include "util.h"

#include <stdint.h>

#include "esp_timer.h"

#define NOP() asm volatile("nop")

void spin_delay(uint64_t us) {
  uint64_t m = (uint64_t)esp_timer_get_time();
  uint64_t e = (m + us);
  if (m > e) {
    while ((uint64_t)esp_timer_get_time() > e) {
      NOP();
    }
  }

  while ((uint64_t)esp_timer_get_time() < e) {
    NOP();
  }
}
