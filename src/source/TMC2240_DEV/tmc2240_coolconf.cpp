/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::COOLCONF_t r{0}; r.sr = COOLCONF(); return r.SETTING
#define SET_REG(SETTING) COOLCONF_register.SETTING = B; write(COOLCONF_register.address, COOLCONF_register.sr)

uint32_t TMC2240Stepper::COOLCONF() { return COOLCONF_register.sr; }
void TMC2240Stepper::COOLCONF(uint32_t data) {
	COOLCONF_register.sr = data;
	write(COOLCONF_register.address, COOLCONF_register.sr);
}

void TMC2240Stepper::sgt(uint8_t B) { SET_REG(sgt); }
void TMC2240Stepper::semin(uint8_t B) { SET_REG(semin); }
void TMC2240Stepper::seup(uint8_t B) { SET_REG(seup); }
void TMC2240Stepper::semax(uint8_t B) { SET_REG(semax); }
void TMC2240Stepper::sedn(uint8_t B) { SET_REG(sedn); }
void TMC2240Stepper::seimin(bool B) { SET_REG(seimin); }
void TMC2240Stepper::sfilt(bool B) { SET_REG(sfilt); }

uint8_t TMC2240Stepper::sgt() { GET_REG(sgt); }
uint8_t TMC2240Stepper::semin() { GET_REG(semin); }
uint8_t TMC2240Stepper::seup() { GET_REG(seup); }
uint8_t TMC2240Stepper::semax() { GET_REG(semax); }
uint8_t TMC2240Stepper::sedn() { GET_REG(sedn); }
bool TMC2240Stepper::seimin() { GET_REG(seimin); }
bool TMC2240Stepper::sfilt() { GET_REG(sfilt); }

#endif // TMC2240_DEV
