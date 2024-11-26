/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::DEVCONF_t r{0}; r.sr = DEVCONF(); return r.SETTING
#define SET_REG(SETTING) DEVCONF_register.SETTING = B; write(DEVCONF_register.address, DEVCONF_register.sr)

uint32_t TMC2240Stepper::DEVCONF() { return DEVCONF_register.sr; }
void TMC2240Stepper::DEVCONF(uint32_t data) {
	DEVCONF_register.sr = data;
	write(DEVCONF_register.address, DEVCONF_register.sr);
}

void TMC2240Stepper::cur_range(uint8_t B) { SET_REG(cur_range); }
void TMC2240Stepper::slope_control(uint8_t B) { SET_REG(slope_control); }

uint8_t TMC2240Stepper::cur_range() { GET_REG(cur_range); }
uint8_t TMC2240Stepper::slope_control() { GET_REG(slope_control); }

#endif // TMC2240_DEV
