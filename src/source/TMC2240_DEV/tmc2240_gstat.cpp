/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::GSTAT_t r{0}; r.sr = GSTAT(); return r.SETTING
#define SET_REG(SETTING) GSTAT_register.SETTING = B; write(GSTAT_register.address, GSTAT_register.sr)

uint32_t TMC2240Stepper::GSTAT() { return GSTAT_register.sr; }
void TMC2240Stepper::GSTAT(uint32_t data) {
	GSTAT_register.sr = data;
	write(GSTAT_register.address, GSTAT_register.sr);
}

void TMC2240Stepper::reset(bool B) { SET_REG(reset); }
void TMC2240Stepper::drv_err(bool B) { SET_REG(drv_err); }
void TMC2240Stepper::uv_cp(bool B) { SET_REG(uv_cp); }
void TMC2240Stepper::register_reset(bool B) { SET_REG(register_reset); }
void TMC2240Stepper::vm_uvlo(bool B) { SET_REG(vm_uvlo); }

bool TMC2240Stepper::reset() { GET_REG(reset); }
bool TMC2240Stepper::drv_err() { GET_REG(drv_err); }
bool TMC2240Stepper::uv_cp() { GET_REG(uv_cp); }
bool TMC2240Stepper::register_reset() { GET_REG(register_reset); }
bool TMC2240Stepper::vm_uvlo() { GET_REG(vm_uvlo); }

#endif // TMC2240_DEV
