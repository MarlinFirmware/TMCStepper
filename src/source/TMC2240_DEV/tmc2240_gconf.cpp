/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::GCONF_t r{0}; r.sr = GCONF(); return r.SETTING
#define SET_REG(SETTING) GCONF_register.SETTING = B; write(GCONF_register.address, GCONF_register.sr)

uint32_t TMC2240Stepper::GCONF() { return GCONF_register.sr; }
void TMC2240Stepper::GCONF(uint32_t data) {
	GCONF_register.sr = data;
	write(GCONF_register.address, GCONF_register.sr);
}

void TMC2240Stepper::diag1_stall(uint8_t B) { SET_REG(diag1_stall); }
//void TMC2240Stepper::slope_control(uint8_t B) { SET_REG(slope_control); }

uint8_t TMC2240Stepper::diag1_stall() { GET_REG(diag1_stall); }

bool TMC2240Stepper::fast_standstill() { GET_REG(fast_standstill); }
bool TMC2240Stepper::en_pwm_mode() { GET_REG(en_pwm_mode); }
bool TMC2240Stepper::multistep_filt() { GET_REG(multistep_filt); }
bool TMC2240Stepper::shaft() { GET_REG(shaft); }
bool TMC2240Stepper::diag0_error () { GET_REG(diag0_error); }
bool TMC2240Stepper::diag0_otpw() { GET_REG(diag0_otpw); }
bool TMC2240Stepper::diag0_stall() { GET_REG(diag0_stall); }
bool TMC2240Stepper::diag1_index() { GET_REG(diag1_index); }
bool TMC2240Stepper::diag1_onstate() { GET_REG(diag1_onstate); }
bool TMC2240Stepper::diag0_pushpull() { GET_REG(diag0_pushpull); }
bool TMC2240Stepper::diag1_pushpull () { GET_REG(diag1_pushpull); }
bool TMC2240Stepper::small_hysteresis() { GET_REG(small_hysteresis); }
bool TMC2240Stepper::stop_enable(){ GET_REG(stop_enable); }
bool TMC2240Stepper::direct_mode(){ GET_REG(direct_mode); }

void TMC2240Stepper::fast_standstill(bool B) { SET_REG(fast_standstill); }
void TMC2240Stepper::en_pwm_mode(bool B) { SET_REG(en_pwm_mode); }
void TMC2240Stepper::multistep_filt(bool B) { SET_REG(multistep_filt); }
void TMC2240Stepper::shaft(bool B) { SET_REG(shaft); }
void TMC2240Stepper::diag0_error(bool B) { SET_REG(diag0_error); }
void TMC2240Stepper::diag0_otpw(bool B) { SET_REG(diag0_otpw); }
void TMC2240Stepper::diag0_stall(bool B) { SET_REG(diag0_stall); }
void TMC2240Stepper::diag1_index(bool B) { SET_REG(diag1_index); }
void TMC2240Stepper::diag1_onstate(bool B) { SET_REG(diag1_onstate); }
void TMC2240Stepper::diag0_pushpull(bool B) { SET_REG(diag0_pushpull); }
void TMC2240Stepper::diag1_pushpull(bool B) { SET_REG(diag1_pushpull); }
void TMC2240Stepper::small_hysteresis(bool B) { SET_REG(small_hysteresis); }
void TMC2240Stepper::stop_enable(bool B) { SET_REG(stop_enable); }
void TMC2240Stepper::direct_mode(bool B) { SET_REG(direct_mode); }

#endif // TMC2240_DEV
