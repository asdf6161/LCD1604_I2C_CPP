/*
 * Lcd_i2c.cpp
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#include <Lcd.h>

namespace lcd {

Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender, uint8_t size_x, uint8_t size_y) {
	this->sender = sender;
	this->init();
	this->size_x = size_x;
	this->size_y = size_y;
	this->curr_pos = 0x0;
}

Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender) {
	this->sender = sender;
	this->init();
	this->size_x = 16;
	this->size_y = 2;
	this->curr_pos = 0x0;
}

Lcd_i2c::~Lcd_i2c() {

}

bool Lcd_i2c::__read_busy_flag(){
	// Todo
	return false;
}

void Lcd_i2c::enable_light(bool state){
	sender->enable_led(state);
}

void Lcd_i2c::enable_cursor(bool state){
	if (state)
		display_on_off |= (1 << BIT_C);
	else
		display_on_off &= ~(1 << BIT_C);
	sender->send_full_byte(display_on_off);
}

void Lcd_i2c::enable_blink(bool state){
	if (state)
		display_on_off |= (1 << BIT_B);
	else
		display_on_off &= ~(1 << BIT_B);
	sender->send_full_byte(display_on_off);
}

void Lcd_i2c::enable_display(bool state){
	if (state)
		display_on_off |= (1 << BIT_D);
	else
		display_on_off &= ~(1 << BIT_D);
	sender->send_full_byte(display_on_off);
}

void write_symbol(const uint8_t sym){
	// Todo
}

void set_cursor_pos(uint8_t x, uint8_t y){

}

void Lcd_i2c::clear_display(){
	sender->send_full_byte(display_clear);
}

void Lcd_i2c::__set_4_bit_interface(){
	LL_mDelay(15);
	sender->send_half_byte(0b11);
	LL_mDelay(5);
	sender->send_half_byte(0b11);
	LL_mDelay(1);
	sender->send_half_byte(0b11);
	LL_mDelay(1);
	sender->send_half_byte(0b10);
}

void Lcd_i2c::init(){
	this->enable_light(true);
	LL_mDelay(1);
	this->__set_4_bit_interface();

	/* setting */
	LL_mDelay(1);
	sender->send_full_byte(function_set);

	/* Display off */
	LL_mDelay(1);
	this->enable_display(0b00001000);

	/* dipsplay clear */
	LL_mDelay(1);
	this->clear_display();

	/* setting mode */
	LL_mDelay(1);
	sender->send_full_byte(entry_mode_set);
}


} /* namespace lcd */

#endif // USE_FULL_LL_DRIVER
