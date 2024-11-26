/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::TPOWERDOWN_t r{0}; r.sr = TPOWERDOWN(); return r.SETTING
#define SET_REG(SETTING) TPOWERDOWN_register.SETTING = B; write(TPOWERDOWN_register.address, TPOWERDOWN_register.sr)

uint32_t TMC2240Stepper::TPOWERDOWN() { return TPOWERDOWN_register.sr; }
void TMC2240Stepper::TPOWERDOWN(uint32_t data) {
	TPOWERDOWN_register.sr = data;
	write(TPOWERDOWN_register.address, TPOWERDOWN_register.sr);
}

#endif // TMC2240_DEV
