/**
 * TMCStepper library by @teemuatlut
 * TMC2208Stepper.cpp
 * Implementing methods for TMC2208 (TMC2209, TMC2224)
 */
#include "../TMCStepper.h"
#include "TMC_MACROS.h"
#include "SERIAL_SWITCH.h"

#ifdef ESP_PLATFORM
	#include <esp32/rom/ets_sys.h>
	#include <esp_timer.h>
	uint32_t millis() {
		return esp_timer_get_time() / 1000;
	}
#endif

// Protected
// addr needed for TMC2209
#ifdef ESP_PLATFORM
TMC2208Stepper::TMC2208Stepper(ESP32_Serial * SerialPort, float RS, uint8_t addr) :
#else
TMC2208Stepper::TMC2208Stepper(Stream * SerialPort, float RS, uint8_t addr) :
#endif
	TMCStepper(RS),
	slave_address(addr)
	{
		HWSerial = SerialPort;
		defaults();
	}

#ifdef ESP_PLATFORM
TMC2208Stepper::TMC2208Stepper( ESP32_Serial * SerialPort, float RS, uint8_t addr, uint16_t mul_pin1, uint16_t mul_pin2) :
#else
TMC2208Stepper::TMC2208Stepper(Stream * SerialPort, float RS, uint8_t addr, uint16_t mul_pin1, uint16_t mul_pin2) :
#endif
	TMC2208Stepper(SerialPort, RS)
	{
		SSwitch *SMulObj = new SSwitch(mul_pin1, mul_pin2, addr);
		sswitch = SMulObj;
	}

#if TMCSTEPPER_SW_SERIAL
	// Protected
	// addr needed for TMC2209
	TMC2208Stepper::TMC2208Stepper(uint16_t SW_RX_pin, uint16_t SW_TX_pin, float RS, uint8_t addr) :
		TMCStepper(RS),
		#if HAS_HALF_DUPLEX_MODE
			RXTX_pin(SW_RX_pin == SW_TX_pin ? SW_RX_pin : 0),
		#endif
		slave_address(addr)
		{
			SoftwareSerial *SWSerialObj = new SoftwareSerial(SW_RX_pin, SW_TX_pin);
			SWSerial = SWSerialObj;
			defaults();
		}

	void TMC2208Stepper::beginSerial(uint32_t baudrate) {
		if (SWSerial != nullptr)
		{
			SWSerial->begin(baudrate);
			SWSerial->end();
		}
		#if HAS_HALF_DUPLEX_MODE
			if (RXTX_pin > 0) {
				digitalWrite(RXTX_pin, HIGH);
				pinMode(RXTX_pin, OUTPUT);
			}
		#endif
	}
#endif

void TMC2208Stepper::begin() {
	#if TMCSTEPPER_SW_SERIAL
		#ifndef TMC2208_BAUDRATE
			#define TMC2208_BAUDRATE 115200
		#endif
		beginSerial(TMC2208_BAUDRATE);
	#endif
	pdn_disable(true);
	mstep_reg_select(true);
}

void TMC2208Stepper::defaults() {
	GCONF_register.i_scale_analog = 1;
	GCONF_register.internal_rsense = 0; // OTP
	GCONF_register.en_spreadcycle = 0; // OTP
	GCONF_register.multistep_filt = 1; // OTP

	IHOLD_IRUN_register.iholddelay = 1; // OTP

	TPOWERDOWN_register.sr = 20;

	//CHOPCONF_register.sr = 0x10000053;
	CHOPCONF_register.toff		= 3;
	CHOPCONF_register.hstrt		= 5;
	CHOPCONF_register.hend		= 0;
	CHOPCONF_register.tbl		= 0;
	CHOPCONF_register.vsense	= false;
	CHOPCONF_register.mres		= 0;
	CHOPCONF_register.intpol	= true;
	CHOPCONF_register.dedge		= false;
	CHOPCONF_register.diss2g	= false;
	CHOPCONF_register.diss2vs	= false;

	//PWMCONF_register.sr = 0xC10D0024;
    PWMCONF_register.pwm_ofs		= 36;
    PWMCONF_register.pwm_grad		= 0;
    PWMCONF_register.pwm_freq		= 1;
    PWMCONF_register.pwm_autoscale	= true;
    PWMCONF_register.pwm_autograd	= true;
    PWMCONF_register.freewheel		= 0;
    PWMCONF_register.pwm_reg		= 1;
    PWMCONF_register.pwm_lim		= 12;

	//MSLUT0_register.sr = ???;
	//MSLUT1_register.sr = ???;
	//MSLUT2_register.sr = ???;
	//MSLUT3_register.sr = ???;
	//MSLUT4_register.sr = ???;
	//MSLUT5_register.sr = ???;
	//MSLUT6_register.sr = ???;
	//MSLUT7_register.sr = ???;
	//MSLUTSTART_register.start_sin90 = 247;
}

void TMC2208Stepper::push() {
	GCONF(GCONF_register.sr);
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
	SLAVECONF(SLAVECONF_register.sr);
	TPOWERDOWN(TPOWERDOWN_register.sr);
	TPWMTHRS(TPWMTHRS_register.sr);
	VACTUAL(VACTUAL_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
}

bool TMC2208Stepper::isEnabled() { return !enn() && toff(); }

uint8_t TMC2208Stepper::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}

__attribute__((weak))
int TMC2208Stepper::available() {
	int out = 0;
	#if TMCSTEPPER_SW_SERIAL
		if (SWSerial != nullptr) {
			out = SWSerial->available();
		} else
	#endif
		if (HWSerial != nullptr) {
			out = HWSerial->available();
		}

	return out;
}

__attribute__((weak))
void TMC2208Stepper::preWriteCommunication() {
	if (HWSerial != nullptr) {
		if (sswitch != nullptr)
			sswitch->active();
	}
}

__attribute__((weak))
void TMC2208Stepper::preReadCommunication() {
	#if TMCSTEPPER_SW_SERIAL
		if (SWSerial != nullptr) {
			SWSerial->listen();
		} else
	#endif
		if (HWSerial != nullptr) {
			if (sswitch != nullptr)
				sswitch->active();
		}
}

__attribute__((weak))
int16_t TMC2208Stepper::serial_read() {
	int16_t out = 0;
	#if TMCSTEPPER_SW_SERIAL
		if (SWSerial != nullptr) {
			out = SWSerial->read();
		} else
	#endif
		if (HWSerial != nullptr) {
			out = HWSerial->read();
		}

	return out;
}

__attribute__((weak))
uint8_t TMC2208Stepper::serial_write(const uint8_t data) {
	int out = 0;;
	#if TMCSTEPPER_SW_SERIAL
		if (SWSerial != nullptr) {
			return SWSerial->write(data);
		} else
	#endif
		if (HWSerial != nullptr) {
			return HWSerial->write(data);
		}

	return out;
}

__attribute__((weak))
void TMC2208Stepper::postWriteCommunication() {}

__attribute__((weak))
void TMC2208Stepper::postReadCommunication() {
	#if TMCSTEPPER_SW_SERIAL
		if (SWSerial != nullptr) {
			SWSerial->end();
		}
	#endif
}

void TMC2208Stepper::write(uint8_t addr, uint32_t regVal) {
	uint8_t len = 7;
	addr |= TMC_WRITE;
	uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, (uint8_t)(regVal>>24), (uint8_t)(regVal>>16), (uint8_t)(regVal>>8), (uint8_t)(regVal>>0), 0x00};

	datagram[len] = calcCRC(datagram, len);

	preWriteCommunication();

	for(uint8_t i=0; i<=len; i++) {
		bytesWritten += serial_write(datagram[i]);
	}
	postWriteCommunication();

	#ifdef ESP_PLATFORM
		ets_delay_us( replyDelay * 1000 );
	#else
		delay(replyDelay);
	#endif
}

uint64_t TMC2208Stepper::_sendDatagram(uint8_t datagram[], const uint8_t len, uint16_t timeout) {
	while (available() > 0) serial_read(); // Flush

	#if HAS_HALF_DUPLEX_MODE
		if (RXTX_pin > 0) {
			digitalWrite(RXTX_pin, HIGH);
			pinMode(RXTX_pin, OUTPUT);
		}
	#endif

	for(int i=0; i<=len; i++) serial_write(datagram[i]);

	#if HAS_HALF_DUPLEX_MODE
		if (RXTX_pin > 0) {
			pinMode(RXTX_pin, INPUT_PULLUP);
		}
	#endif

	#ifdef ESP_PLATFORM
		ets_delay_us( replyDelay * 1000 );
	#else
		delay(this->replyDelay);
	#endif

	// scan for the rx frame and read it
	uint32_t ms = millis();
	uint32_t sync_target = (static_cast<uint32_t>(datagram[0])<<16) | 0xFF00 | datagram[2];
	uint32_t sync = 0;

	do {
		uint32_t ms2 = millis();
		if (ms2 != ms) {
			// 1ms tick
			ms = ms2;
			timeout--;
		}
		if (!timeout) return 0;

		int16_t res = serial_read();
		if (res < 0) continue;

		sync <<= 8;
		sync |= res & 0xFF;
		sync &= 0xFFFFFF;

	} while (sync != sync_target);

	uint64_t out = sync;
	ms = millis();
	timeout = this->abort_window;

	for(uint8_t i=0; i<5;) {
		uint32_t ms2 = millis();
		if (ms2 != ms) {
			// 1ms tick
			ms = ms2;
			timeout--;
		}
		if (!timeout) return 0;

		int16_t res = serial_read();
		if (res < 0) continue;

		out <<= 8;
		out |= res & 0xFF;

		i++;
	}

	#if HAS_HALF_DUPLEX_MODE
		if (RXTX_pin > 0) {
			digitalWrite(RXTX_pin, HIGH);
			pinMode(RXTX_pin, OUTPUT);
		}
	#endif

	while (available() > 0) serial_read(); // Flush

	return out;
}

uint32_t TMC2208Stepper::read(uint8_t addr) {
	constexpr uint8_t len = 3;
	addr |= TMC_READ;
	uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, 0x00};
	datagram[len] = calcCRC(datagram, len);
	uint64_t out = 0x00000000UL;

	for (uint8_t i = 0; i < max_retries; i++) {
		preReadCommunication();
		delay(3);
		out = _sendDatagram(datagram, len, abort_window);
		postReadCommunication();

		#ifdef ESP_PLATFORM
			ets_delay_us( replyDelay * 1000 );
		#else
			delay(replyDelay);
		#endif

		CRCerror = false;
		uint8_t out_datagram[] = {
			static_cast<uint8_t>(out>>56),
			static_cast<uint8_t>(out>>48),
			static_cast<uint8_t>(out>>40),
			static_cast<uint8_t>(out>>32),
			static_cast<uint8_t>(out>>24),
			static_cast<uint8_t>(out>>16),
			static_cast<uint8_t>(out>> 8),
			static_cast<uint8_t>(out>> 0)
		};
		uint8_t crc = calcCRC(out_datagram, 7);
		if ((crc != static_cast<uint8_t>(out)) || crc == 0 ) {
			CRCerror = true;
			out = 0;
		} else {
			break;
		}
	}

	return out>>8;
}

uint8_t TMC2208Stepper::IFCNT() { return read(IFCNT_t::address); }

void TMC2208Stepper::OTP_PROG(uint16_t input) {
	write(OTP_PROG_t::address, input);
}

uint32_t TMC2208Stepper::OTP_READ() { return read(OTP_READ_t::address); }

uint16_t TMC2208Stepper::FACTORY_CONF() { return read(FACTORY_CONF_register.address); }
void TMC2208Stepper::FACTORY_CONF(uint16_t input) {
	FACTORY_CONF_register.sr = input;
	write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr);
}
void TMC2208Stepper::fclktrim(uint8_t B){ FACTORY_CONF_register.fclktrim = B; write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr); }
void TMC2208Stepper::ottrim(uint8_t B)	{ FACTORY_CONF_register.ottrim = B; write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr); }
uint8_t TMC2208Stepper::fclktrim()		{ FACTORY_CONF_t r{}; r.sr = FACTORY_CONF(); return r.fclktrim; }
uint8_t TMC2208Stepper::ottrim()		{ FACTORY_CONF_t r{}; r.sr = FACTORY_CONF(); return r.ottrim; }

void TMC2208Stepper::VACTUAL(uint32_t input) {
	VACTUAL_register.sr = input;
	write(VACTUAL_register.address, VACTUAL_register.sr);
}
uint32_t TMC2208Stepper::VACTUAL() {
	return VACTUAL_register.sr;
}

uint32_t TMC2208Stepper::PWM_SCALE() { return read(TMC2208_n::PWM_SCALE_t::address); }
uint8_t TMC2208Stepper::pwm_scale_sum() {
	TMC2208_n::PWM_SCALE_t r{};
	r.sr = PWM_SCALE();
	return r.pwm_scale_sum;
}

int16_t TMC2208Stepper::pwm_scale_auto() {
	TMC2208_n::PWM_SCALE_t r{};
	r.sr = PWM_SCALE();
	return r.pwm_scale_auto;
	// Not two's complement? 9nth bit determines sign
	/*
	uint32_t d = PWM_SCALE();
	int16_t response = (d>>PWM_SCALE_AUTO_bp)&0xFF;
	if (((d&PWM_SCALE_AUTO_bm) >> 24) & 0x1) return -response;
	else return response;
	*/
}

// R: PWM_AUTO
uint32_t TMC2208Stepper::PWM_AUTO() { return read(PWM_AUTO_t::address); }
uint8_t TMC2208Stepper::pwm_ofs_auto()  { PWM_AUTO_t r{}; r.sr = PWM_AUTO(); return r.pwm_ofs_auto; }
uint8_t TMC2208Stepper::pwm_grad_auto() { PWM_AUTO_t r{}; r.sr = PWM_AUTO(); return r.pwm_grad_auto; }

// R: MSCURACT
uint32_t TMC2208Stepper::MSCURACT() { return read(TMC2208_n::MSCURACT_t::address); }
int16_t TMC2208Stepper::cur_a() {
	TMC2208_n::MSCURACT_t r{};
	r.sr = MSCURACT();
	int16_t value = r.cur_a;
	if (value > 255) value -= 512;
	return value;
}
int16_t TMC2208Stepper::cur_b() {
	TMC2208_n::MSCURACT_t r{};
	r.sr = MSCURACT();
	int16_t value = r.cur_b;
	if (value > 255) value -= 512;
	return value;
}
