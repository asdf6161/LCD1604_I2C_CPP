/*
 * Lcd_i2c.cpp
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#include <Lcd.h>

namespace lcd {
//
//Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender, uint8_t size_x, uint8_t size_y) {
//	this->sender = sender;
//	this->init();
//	this->size_x = size_x;
//	this->size_y = size_y;
//	this->curr_pos = 0x0;
//}

Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender) {
	this->sender = sender;
	this->size_x = 16;
	this->size_y = 2;
	this->curr_pos = 0x0;
	this->init();
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

void Lcd_i2c::write_symbol(const uint8_t sym){
	this->sender->write_data(sym);
}

void Lcd_i2c::write_string(uint8_t *sym){
	while((*sym) != '\0'){
		this->write_symbol((*sym++));
	}
}

void Lcd_i2c::set_cursor_pos(uint8_t x, uint8_t y){
	if (x > 0x3f){
		x = 0x3f;
	}
	if (y > 1){
		y = 1;
	}
	this->curr_pos = x | (y << 6);
	this->__set_DDRAM_addr(this->curr_pos);
}

void Lcd_i2c::write_user_symbol(const uint8_t *arr, const uint8_t addres){
	this->__set_CGRAM_addr(addres * 8);
	for (uint8_t i = 0; i < 8; ++i) {
		this->sender->write_data(arr[i]);
	}
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

void Lcd_i2c::__set_DDRAM_addr(uint8_t addr){
	this->DDRAM_addres &= 0b10000000;
	this->DDRAM_addres |= addr & 0b01111111;
	this->sender->send_full_byte(DDRAM_addres);
}

void Lcd_i2c::__set_CGRAM_addr(uint8_t addr){
	this->sender->send_full_byte(CGRAM_addres | (addr & 0x1f));
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
