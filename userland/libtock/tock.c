#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "tock.h"

void yield_for(bool *cond) {
  while(!*cond) {
    yield();
  }
}

void yield(void) {
  // Note: A process stops yielding when there is a callback ready to run,
  // which the kernel executes by modifying the stack frame pushed by the
  // hardware. The kernel copies the PC value from the stack frame to the
  // LR field, and sets the PC value to callback to run. When this frame is
  // unstacked during the interrupt return, the effectively clobbers the LR
  // register.
  //
  // At this point, the callback function is now executing, which may itself
  // clobber any of the other caller-saved registers. Thus we mark this inline
  // assembly as conservatively clobbering all caller-saved registers, forcing
  // yield to save any live registers.
  //
  // Upon direct observation of this function, the LR is the only register that
  // is live across the SVC invocation, however, if the yield call is inlined,
  // it is possible that the LR won't be live at all (commonly seen for the
  // `while (1) { yield(); }` idiom) or that other registers are live, thus it
  // is important to let the compiler do the work here.
  //
  // According to the AAPCS:
  //   A subroutine must preserve the contents of the registers r4-r8, r10, r11
  //   and SP (and r9 in PCS variants that designate r9 as v6)
  // As our compilation flags mark r9 as the PIC base register, it does not need
  // to be saved. Thus we must clobber r0-3, r12, and LR
  asm volatile(
      "svc 0       \n"
      :
      :
      : "memory", "r0", "r1", "r2", "r3", "r12", "lr"
      );
}

int subscribe(uint32_t driver, uint32_t subscribe,
              subscribe_cb cb, void* userdata) {
  register uint32_t r0 asm("r0") = driver;
  register uint32_t r1 asm("r1") = subscribe;
  register void*    r2 asm("r2") = cb;
  register void*    r3 asm("r3") = userdata;
  register int ret asm ("r0");
  asm volatile(
      "svc 1"
      : "=r" (ret)
      : "r" (r0), "r" (r1), "r" (r2), "r" (r3)
      : "memory");
  return ret;
}


int command(uint32_t driver, uint32_t command, int data) {
  register uint32_t r0 asm("r0") = driver;
  register uint32_t r1 asm("r1") = command;
  register uint32_t r2 asm("r2") = data;
  register int ret asm ("r0");
  asm volatile(
      "svc 2"
      : "=r" (ret)
      : "r" (r0), "r" (r1), "r" (r2)
      : "memory"
      );
  return ret;
}

int allow(uint32_t driver, uint32_t allow, void* ptr, size_t size) {
  register uint32_t r0 asm("r0") = driver;
  register uint32_t r1 asm("r1") = allow;
  register void*    r2 asm("r2") = ptr;
  register size_t   r3 asm("r3") = size;
  register int ret asm ("r0");
  asm volatile(
      "svc 3"
      : "=r" (ret)
      : "r" (r0), "r" (r1), "r" (r2), "r" (r3)
      : "memory"
      );
  return ret;
}

void* memop(uint32_t op_type, int arg1) {
  register uint32_t r0 asm("r0") = op_type;
  register int      r1 asm("r1") = arg1;
  register void*   ret asm("r0");
  asm volatile(
      "svc 4"
      : "=r" (ret)
      : "r" (r0), "r" (r1)
      : "memory"
      );
  return ret;
}

bool driver_exists(uint32_t driver) {
  int ret = command(driver, 0, 0);
  return ret >= 0;
}
