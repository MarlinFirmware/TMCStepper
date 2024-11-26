/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.SETTING
#define SET_REG(SETTING) PWMCONF_register.SETTING = B; write(PWMCONF_register.address, PWMCONF_register.sr)

uint32_t TMC2240Stepper::PWMCONF() { return PWMCONF_register.sr; }
void TMC2240Stepper::PWMCONF(uint32_t data) {
	PWMCONF_register.sr = data;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2240Stepper::pwm_ofs(uint8_t B) { SET_REG(pwm_ofs); }
void TMC2240Stepper::pwm_grad(uint8_t B) { SET_REG(pwm_grad); }
void TMC2240Stepper::pwm_freq(uint8_t B) { SET_REG(pwm_freq); }
void TMC2240Stepper::pwm_autoscale(bool B) { SET_REG(pwm_autoscale); }
void TMC2240Stepper::pwm_autograd(bool B) { SET_REG(pwm_autograd); }
void TMC2240Stepper::freewheel(uint8_t B) { SET_REG(freewheel); }
void TMC2240Stepper::pwm_meas_sd_enable(bool B) { SET_REG(pwm_meas_sd_enable); }
void TMC2240Stepper::pwm_dis_reg_stst(bool B) { SET_REG(pwm_dis_reg_stst); }
void TMC2240Stepper::pwm_reg(uint8_t B) { SET_REG(pwm_reg); }
void TMC2240Stepper::pwm_lim(uint8_t B) { SET_REG(pwm_lim); }

uint8_t TMC2240Stepper::pwm_ofs() { GET_REG(pwm_ofs); }
uint8_t TMC2240Stepper::pwm_grad() { GET_REG(pwm_grad); }
uint8_t TMC2240Stepper::pwm_freq() { GET_REG(pwm_freq); }
bool TMC2240Stepper::pwm_autoscale() { GET_REG(pwm_autoscale); }
bool TMC2240Stepper::pwm_autograd() { GET_REG(pwm_autograd); }
uint8_t TMC2240Stepper::freewheel() { GET_REG(freewheel); }
bool TMC2240Stepper::pwm_meas_sd_enable() { GET_REG(pwm_meas_sd_enable); }
bool TMC2240Stepper::pwm_dis_reg_stst() { GET_REG(pwm_dis_reg_stst); }
uint8_t TMC2240Stepper::pwm_reg() { GET_REG(pwm_reg); }
uint8_t TMC2240Stepper::pwm_lim() { GET_REG(pwm_lim); }

#endif // TMC2240_DEV
