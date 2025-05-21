/**
 * TMCStepper library by @teemuatlut
 */
#ifdef TMC2240_DEV

#include "../TMC_MACROS.h"
#include "../../TMCStepper.h"

#define GET_REG(SETTING) TMC2240_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.SETTING
#define SET_REG(SETTING) CHOPCONF_register.SETTING = B; write(CHOPCONF_register.address, CHOPCONF_register.sr)

#define GET_REG_2240(SETTING) return CHOPCONF_register.SETTING

uint32_t TMC2240Stepper::CHOPCONF() { return CHOPCONF_register.sr; }
void TMC2240Stepper::CHOPCONF(uint32_t data) {
	CHOPCONF_register.sr = data;
	write(CHOPCONF_register.address, CHOPCONF_register.sr);
}

void TMC2240Stepper::toff(uint8_t B) {
	SET_REG(toff);
	if (B>0) _savedToff = B;
}

void TMC2240Stepper::hstrt(uint8_t B) { SET_REG(hstrt); }
void TMC2240Stepper::hend(uint8_t B) { SET_REG(hend); }
void TMC2240Stepper::fd3(bool B) { SET_REG(fd3); }
void TMC2240Stepper::disfdcc(bool B) { SET_REG(disfdcc); }
void TMC2240Stepper::chm(bool B) { SET_REG(chm); }
void TMC2240Stepper::tbl(uint8_t B) { SET_REG(tbl); }
void TMC2240Stepper::vhighfs(bool B) { SET_REG(vhighfs); }
void TMC2240Stepper::vhighchm(bool B) { SET_REG(vhighchm); }
void TMC2240Stepper::tpfd(uint8_t B) { SET_REG(tpfd); }
void TMC2240Stepper::mres(uint8_t B) { SET_REG(mres); }
void TMC2240Stepper::intpol(bool B) { SET_REG(intpol); }
void TMC2240Stepper::dedge(bool B) { SET_REG(dedge); }
void TMC2240Stepper::diss2g(bool B) { SET_REG(diss2g); }
void TMC2240Stepper::diss2vs(bool B) { SET_REG(diss2vs); }

uint8_t TMC2240Stepper::toff() { GET_REG_2240(toff); }
uint8_t TMC2240Stepper::hstrt() { GET_REG(hstrt); }
uint8_t TMC2240Stepper::hend() { GET_REG(hend); }
bool TMC2240Stepper::fd3() { GET_REG(fd3); }
bool TMC2240Stepper::disfdcc() { GET_REG(disfdcc); }
bool TMC2240Stepper::chm() { GET_REG(chm); }
uint8_t TMC2240Stepper::tbl() { GET_REG(tbl); }
bool TMC2240Stepper::vhighfs() { GET_REG(vhighfs); }
bool TMC2240Stepper::vhighchm() { GET_REG(vhighchm); }
uint8_t TMC2240Stepper::tpfd() { GET_REG(tpfd); }
uint8_t TMC2240Stepper::mres() { GET_REG(mres); }
bool TMC2240Stepper::intpol() { GET_REG(intpol); }
bool TMC2240Stepper::dedge() { GET_REG(dedge); }
bool TMC2240Stepper::diss2g() { GET_REG(diss2g); }
bool TMC2240Stepper::diss2vs() { GET_REG(diss2vs); }

#endif // TMC2240_DEV
