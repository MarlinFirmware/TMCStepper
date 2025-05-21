/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::TCOOLTHRS_t r{0}; r.sr = TCOOLTHRS(); return r.SETTING
#define SET_REG(SETTING) TCOOLTHRS_register.SETTING = B; write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr)

uint32_t TMC2240Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2240Stepper::TCOOLTHRS(uint32_t data) {
	TCOOLTHRS_register.sr = data;
	write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}

void TMC2240Stepper::tcoolthrs(uint8_t B)	{ SET_REG(tcoolthrs); 	}
uint8_t TMC2240Stepper::tcoolthrs()			{ GET_REG(tcoolthrs); 	}

#endif // TMC2240_DEV
