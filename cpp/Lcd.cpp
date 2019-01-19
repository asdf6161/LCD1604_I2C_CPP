/*
 * Lcd_i2c.cpp
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER2
#include <Lcd.h>

namespace lcd {

Lcd_i2c::Lcd_i2c(uint8_t addr, func_out fo, func_in fi) {
	/*this->addres = addr;
	this->transmit8 = fo;
	this->recive8 = fi;
	data = {0, 0, 0, 0, 0, 0, 0, 0};*/
}

Lcd_i2c::~Lcd_i2c() {

}

void Lcd_i2c::__send_full_bt(const uint8_t bt){
	/*uint8_t rw = this->data.bits.RW;
	this->data.bits.RW = 0;
	// first high
	this->__send_half_bt(bt >> 4);
	LL_mDelay(1);
	this->__send_half_bt(bt);
	LL_mDelay(1);
	this->data.bits.RW = rw;*/
}

void Lcd_i2c::__send_half_bt(const uint8_t half_byte){
	/*uint8_t bt = half_byte & 0b1111;
	this->data.byte |= (bt << 4);
	this->data.byte &= ~(bt << 4);
	this->__send_with_strob(bt);*/
}

bool Lcd_i2c::__read_busy_flag(){
	// Todo
/*	this->data.bits.RW = 1;
	this->__send_half_bt(0b0000);
	this->data.bits.RW = 0;
	uint8_t ans = this->recive8(this->addres);
	return (ans >> 4) & 0b1;
	*/
	return false;
}

void Lcd_i2c::enable_light(bool state){
//	this->data.bits.LED = state;
}

void Lcd_i2c::enable_cursor(bool state){
	if (state)
		display_on_off |= (1 << BIT_C);
	else
		display_on_off &= ~(1 << BIT_C);
	this->__send_full_bt(display_on_off);
}

void Lcd_i2c::enable_blink(bool state){
	if (state)
		display_on_off |= (1 << BIT_B);
	else
		display_on_off &= ~(1 << BIT_B);
	this->__send_full_bt(display_on_off);
}

void Lcd_i2c::enable_display(bool state){
	if (state)
		display_on_off |= (1 << BIT_D);
	else
		display_on_off &= ~(1 << BIT_D);
	this->__send_full_bt(display_on_off);
}

void Lcd_i2c::clear_display(){
	this->__send_full_bt(display_clear);
}

void Lcd_i2c::__set_4_bit_interface(){
	LL_mDelay(15);
	__send_half_bt(0b11);
	LL_mDelay(5);
	__send_half_bt(0b11);
	LL_mDelay(1);
	__send_half_bt(0b11);
	LL_mDelay(1);
	__send_half_bt(0b10);

}

void Lcd_i2c::init(){
	this->enable_light(true);
	LL_mDelay(1);
	this->__set_4_bit_interface();
	LL_mDelay(1);
	this->__send_full_bt(function_set);
	LL_mDelay(1);
	this->enable_display(false);
	LL_mDelay(1);
	this->__send_full_bt(entry_mode_set);
	LL_mDelay(1);
	this->__send_full_bt(display_on_off);
}

void Lcd_i2c::__send_with_strob(const uint8_t half_byte){

}

} /* namespace lcd */

#endif // USE_FULL_LL_DRIVER
