/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::SG4_THRS_t r{0}; r.sr = SG4_THRS(); return r.SETTING
#define SET_REG(SETTING) SG4_THRS_register.SETTING = B; write(SG4_THRS_register.address, SG4_THRS_register.sr)

uint32_t TMC2240Stepper::SG4_THRS() { return SG4_THRS_register.sr; }
void TMC2240Stepper::SG4_THRS(uint32_t data) {
	SG4_THRS_register.sr = data;
	write(SG4_THRS_register.address, SG4_THRS_register.sr);
}

void TMC2240Stepper::sg4_thrsb(uint8_t B) { SET_REG(sg4_thrsb); }
void TMC2240Stepper::sg4_filt_en(bool B) { SET_REG(sg4_filt_en); }
void TMC2240Stepper::sg_angle_offset(bool B) { SET_REG(sg_angle_offset); }

uint8_t TMC2240Stepper::sg4_thrsb() { GET_REG(sg4_thrsb); }
bool TMC2240Stepper::sg4_filt_en() { GET_REG(sg4_filt_en); }
bool TMC2240Stepper::sg_angle_offset() { GET_REG(sg_angle_offset); }

#endif // TMC2240_DEV
