/*
 * Pcf8574t.cpp
 *
 *  Created on: 20 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER

#include <Pcf8574t.h>

namespace pcf {

Pcf8574t::Pcf8574t(uint8_t addr, pcf_func_out f_out, pcf_func_in f_in) {
	this->f_in = f_in;
	this->f_out = f_out;
	this->addr = addr;
	this->packet = {0,0,0,0,0,0,0,0};
}

Pcf8574t::~Pcf8574t() {

}

void Pcf8574t::send_half_byte(const uint8_t half_bt){
	LL_mDelay(1);
	this->__send_half_byte_with_strob(half_bt & 0x0f);
}

void Pcf8574t::send_full_byte(const uint8_t bt){
	this->send_half_byte(bt >> 4);
	this->send_half_byte(bt);
}

void Pcf8574t::__send_half_byte_with_strob(const uint8_t half_bt){
	this->packet.byte &= 0b1111;
	this->packet.byte |= half_bt << 4;
	this->packet.bits.EN = 1;
	this->f_out(this->addr, this->packet.byte);
	LL_mDelay(1);
	this->packet.bits.EN = 0;
	this->f_out(this->addr, this->packet.byte);
}

void Pcf8574t::__set_pin(pcf_packet *p){
	this->f_out(this->addr, p->byte);
}

void Pcf8574t::enable_led(const bool state){
	if (state)
		this->packet.bits.LED = 1;
	else
		this->packet.bits.LED = 0;
	this->__set_pin(&this->packet);
}

} /* namespace lcd */

#endif // USE_FULL_LL_DRIVER2
