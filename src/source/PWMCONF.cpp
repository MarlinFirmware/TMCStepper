/**
 * TMCStepper library by @teemuatlut
 * PWMCONF.cpp - PWM Configuration
 * TMC2130 (TMC2160, TMC5130, TMC5160, TMC5161), TMC2160 (TMC5130, TMC5160, TMC5161), TMC2208 (TMC2209, TMC2224)
 */
#include "TMCStepper.h"
#include "TMC_MACROS.h"

#define SET_REG(SETTING) PWMCONF_register.SETTING = B; write(PWMCONF_register.address, PWMCONF_register.sr)
#define GET_REG(SETTING) return PWMCONF_register.SETTING

//
// TMC2130 (TMC2160, TMC5130, TMC5160, TMC5161)
//

uint32_t TMC2130Stepper::PWMCONF() { return PWMCONF_register.sr; }
void TMC2130Stepper::PWMCONF(uint32_t input) {
	PWMCONF_register.sr = input;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2130Stepper::pwm_ampl(		uint8_t B )	{ SET_REG(pwm_ampl);		}
void TMC2130Stepper::pwm_grad(		uint8_t B )	{ SET_REG(pwm_grad);		}
void TMC2130Stepper::pwm_freq(		uint8_t B )	{ SET_REG(pwm_freq);		}
void TMC2130Stepper::pwm_autoscale(	bool 	B )	{ SET_REG(pwm_autoscale);	}
void TMC2130Stepper::pwm_symmetric(	bool 	B )	{ SET_REG(pwm_symmetric);	}
void TMC2130Stepper::freewheel(		uint8_t B )	{ SET_REG(freewheel);		}

uint8_t TMC2130Stepper::pwm_ampl()		{ GET_REG(pwm_ampl);		}
uint8_t TMC2130Stepper::pwm_grad()		{ GET_REG(pwm_grad);		}
uint8_t TMC2130Stepper::pwm_freq()		{ GET_REG(pwm_freq);		}
bool 	TMC2130Stepper::pwm_autoscale()	{ GET_REG(pwm_autoscale);	}
bool 	TMC2130Stepper::pwm_symmetric()	{ GET_REG(pwm_symmetric);	}
uint8_t TMC2130Stepper::freewheel()		{ GET_REG(freewheel);		}

//
// TMC2160 (TMC5130, TMC5160, TMC5161)
//

uint32_t TMC2160Stepper::PWMCONF() {
	return PWMCONF_register.sr;
}
void TMC2160Stepper::PWMCONF(uint32_t input) {
	PWMCONF_register.sr = input;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2160Stepper::pwm_ofs		( uint8_t B ) { PWMCONF_register.pwm_ofs = B; 		write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::pwm_grad		( uint8_t B ) { PWMCONF_register.pwm_grad = B; 		write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::pwm_freq		( uint8_t B ) { PWMCONF_register.pwm_freq = B; 		write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::pwm_autoscale	( bool 	  B ) { PWMCONF_register.pwm_autoscale = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::pwm_autograd	( bool    B ) { PWMCONF_register.pwm_autograd = B; 	write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::freewheel		( uint8_t B ) { PWMCONF_register.freewheel = B; 	write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::pwm_reg		( uint8_t B ) { PWMCONF_register.pwm_reg = B; 		write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2160Stepper::pwm_lim		( uint8_t B ) { PWMCONF_register.pwm_lim = B; 		write(PWMCONF_register.address, PWMCONF_register.sr); }

uint8_t TMC2160Stepper::pwm_ofs()		{ return PWMCONF_register.pwm_ofs;		}
uint8_t TMC2160Stepper::pwm_grad()		{ return PWMCONF_register.pwm_grad;		}
uint8_t TMC2160Stepper::pwm_freq()		{ return PWMCONF_register.pwm_freq;		}
bool 	TMC2160Stepper::pwm_autoscale()	{ return PWMCONF_register.pwm_autoscale;}
bool 	TMC2160Stepper::pwm_autograd()	{ return PWMCONF_register.pwm_autograd;	}
uint8_t TMC2160Stepper::freewheel()		{ return PWMCONF_register.freewheel;	}
uint8_t TMC2160Stepper::pwm_reg()		{ return PWMCONF_register.pwm_reg;		}
uint8_t TMC2160Stepper::pwm_lim()		{ return PWMCONF_register.pwm_lim;		}

//
// TMC2208 (TMC2209, TMC2224)
//

uint32_t TMC2208Stepper::PWMCONF() {
	return read(PWMCONF_register.address);
}
void TMC2208Stepper::PWMCONF(uint32_t input) {
	PWMCONF_register.sr = input;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2208Stepper::pwm_ofs		( uint8_t B ) { PWMCONF_register.pwm_ofs = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_grad		( uint8_t B ) { PWMCONF_register.pwm_grad = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_freq		( uint8_t B ) { PWMCONF_register.pwm_freq = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_autoscale	( bool 	  B ) { PWMCONF_register.pwm_autoscale = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_autograd	( bool    B ) { PWMCONF_register.pwm_autograd = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::freewheel		( uint8_t B ) { PWMCONF_register.freewheel = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_reg		( uint8_t B ) { PWMCONF_register.pwm_reg = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_lim		( uint8_t B ) { PWMCONF_register.pwm_lim = B; write(PWMCONF_register.address, PWMCONF_register.sr); }

uint8_t TMC2208Stepper::pwm_ofs()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_ofs;		}
uint8_t TMC2208Stepper::pwm_grad()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_grad;		}
uint8_t TMC2208Stepper::pwm_freq()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_freq;		}
bool 	TMC2208Stepper::pwm_autoscale()	{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autoscale;	}
bool 	TMC2208Stepper::pwm_autograd()	{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autograd;	}
uint8_t TMC2208Stepper::freewheel()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.freewheel;		}
uint8_t TMC2208Stepper::pwm_reg()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_reg;		}
uint8_t TMC2208Stepper::pwm_lim()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_lim;		}

// PWMCONF
uint32_t TMC2240Stepper::PWMCONF() { return PWMCONF_register.sr; }
void TMC2240Stepper::PWMCONF(uint32_t input) {
	PWMCONF_register.sr = input;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2240Stepper::pwm_ofs			( uint8_t B ) { PWMCONF_register.pwm_ofs = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_grad			( uint8_t B ) { PWMCONF_register.pwm_grad = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_freq			( uint8_t B ) { PWMCONF_register.pwm_freq = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_autoscale		( bool 	  B ) { PWMCONF_register.pwm_autoscale = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_autograd		( bool    B ) { PWMCONF_register.pwm_autograd = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::freewheel			( uint8_t B ) { PWMCONF_register.freewheel = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_meas_sd_enable	( bool 	  B ) { PWMCONF_register.pwm_autoscale = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_dis_reg_stst	( bool    B ) { PWMCONF_register.pwm_autograd = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_reg			( uint8_t B ) { PWMCONF_register.pwm_reg = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2240Stepper::pwm_lim			( uint8_t B ) { PWMCONF_register.pwm_lim = B; write(PWMCONF_register.address, PWMCONF_register.sr); }

uint8_t TMC2240Stepper::pwm_ofs()				{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_ofs;		}
uint8_t TMC2240Stepper::pwm_grad()				{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_grad;		}
uint8_t TMC2240Stepper::pwm_freq()				{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_freq;		}
bool 	TMC2240Stepper::pwm_autoscale()			{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autoscale;	}
bool 	TMC2240Stepper::pwm_autograd()			{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autograd;	}
uint8_t TMC2240Stepper::freewheel()				{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.freewheel;		}
bool 	TMC2240Stepper::pwm_meas_sd_enable()	{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autoscale;	}
bool 	TMC2240Stepper::pwm_dis_reg_stst()		{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autograd;	}
uint8_t TMC2240Stepper::pwm_reg()				{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_reg;		}
uint8_t TMC2240Stepper::pwm_lim()				{ TMC2240_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_lim;		}
