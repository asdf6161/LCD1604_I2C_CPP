/*
 * Lcd_i2c.cpp
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#include <Lcd_i2c.h>

namespace lcd {

Lcd_i2c::Lcd_i2c(uint8_t addr, func_out fo, func_in fi) {
	this->addres = addr;
	this->transmit8 = fo;
	this->recive8 = fi;
	data = {0, 0, 0, 0, 0, 0, 0, 0};
}

Lcd_i2c::~Lcd_i2c() {

}

void Lcd_i2c::__send_full_bt(uint8_t byte){
	this->__send_half_bt(byte);
	this->__send_half_bt(byte >> 4);
}

void Lcd_i2c::__send_half_bt(uint8_t half_byte){
	half_byte &= 0b1111;
	this->data.byte |= (half_byte << 4);
	this->__send_with_strob();
	this->data.byte &= ~(half_byte << 4);
	this->__send_with_strob();
}

bool Lcd_i2c::__read_busy_flag(){
	this->data.bits.RW = 1;
	this->__send_half_bt(0b0000);
	this->data.bits.RW = 0;
	uint8_t ans = this->recive8(this->addres);
	return (ans >> 4) & 0b1;
}

void Lcd_i2c::enable_light(bool state){
	this->data.bits.LED = state;
}

void Lcd_i2c::enable_cursor(bool state){

}



void Lcd_i2c::init(){
	this->enable_light(true);
	LL_mDelay(15);
	__send_half_bt(0b11);
	this->__send_with_strob();
//	while(!this->__read_busy_flag()){}
	LL_mDelay(5);
	this->__send_with_strob();
//	while(this->__read_busy_flag()){}
	LL_mDelay(1);
	this->__send_with_strob();

	LL_mDelay(1);
//	while(this->__read_busy_flag()){}
	__send_half_bt(0b10);
	this->__send_with_strob();

	LL_mDelay(1);
//	while(this->__read_busy_flag()){}
	this->__send_with_strob();

	LL_mDelay(1);
//	while(this->__read_busy_flag()){}
	__send_half_bt(0b1000);
	this->__send_with_strob();

	__send_half_bt(0);
	this->__send_with_strob();
	__send_half_bt(0b1000);
	this->__send_with_strob();


	__send_half_bt(0);
	this->__send_with_strob();
	__send_half_bt(0b1000);
	this->__send_with_strob();


	__send_half_bt(0);
	this->__send_with_strob();
	__send_half_bt(0b110);
	this->__send_with_strob();

	__send_half_bt(0);
	this->__send_with_strob();
	__send_half_bt(0b1111);
	this->__send_with_strob();

	LL_mDelay(10);
}

void Lcd_i2c::__send_with_strob(){
	this->data.bits.EN = 1;
	this->transmit8(this->addres, this->data.byte);
	this->data.bits.EN = 0;
	this->transmit8(this->addres, this->data.byte);
}

} /* namespace lcd */

#endif // USE_FULL_LL_DRIVER
