/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::IHOLD_IRUN_t r{0}; r.sr = IHOLD_IRUN(); return r.SETTING
#define SET_REG(SETTING) IHOLD_IRUN_register.SETTING = B; write(IHOLD_IRUN_register.address, IHOLD_IRUN_register.sr)

uint32_t TMC2240Stepper::IHOLD_IRUN() { return IHOLD_IRUN_register.sr; }
void TMC2240Stepper::IHOLD_IRUN(uint32_t data) {
	IHOLD_IRUN_register.sr = data;
	write(IHOLD_IRUN_register.address, IHOLD_IRUN_register.sr);
}

void TMC2240Stepper::ihold(uint8_t B) { SET_REG(ihold); }
void TMC2240Stepper::run(uint8_t B) { SET_REG(run); }
void TMC2240Stepper::iholddelay(uint8_t B) { SET_REG(iholddelay); }
void TMC2240Stepper::irundelay(uint8_t B) { SET_REG(irundelay); }

uint8_t TMC2240Stepper::ihold() { GET_REG(ihold); }
uint8_t TMC2240Stepper::run() { GET_REG(run); }
uint8_t TMC2240Stepper::iholddelay() { GET_REG(iholddelay); }
uint8_t TMC2240Stepper::irundelay() { GET_REG(irundelay); }

#endif // TMC2240_DEV
