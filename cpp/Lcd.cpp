/*
 * Lcd_i2c.cpp
 *
 *  Created on: 18 янв. 2019 г.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#include <Lcd.h>

/*
 * Сделать так чтобы переменная curr_pos всегда соответствовала действительности
 * Сделать так чтобы все измененые и установленные параметры дисплея хранились в классе
 * */

namespace lcd {
//
//Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender, uint8_t size_x, uint8_t size_y) {
//	this->sender = sender;
//	this->init();
//	this->size_x = size_x;
//	this->size_y = size_y;
//	this->curr_pos = 0x0;
//}

Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender, display_line_cnt line_cnt) {
	this->sender = sender;
	this->size_x = 16;
	this->size_y = 2;
	this->curr_pos = 0x0;
	this->line_cnt = line_cnt;
	this->set_numbers_disp_line(this->line_cnt);
	this->init();
}

Lcd_i2c::Lcd_i2c(Lcd_sender_abstract *sender) {
	this->sender = sender;
	this->size_x = 16;
	this->size_y = 2;
	this->curr_pos = 0x0;
	this->line_cnt = DISP_2_LINE;
	this->set_numbers_disp_line(this->line_cnt);
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
	sender->send_byte(display_on_off);
}

void Lcd_i2c::enable_blink(bool state){
	if (state)
		display_on_off |= (1 << BIT_B);
	else
		display_on_off &= ~(1 << BIT_B);
	sender->send_byte(display_on_off);
}

void Lcd_i2c::enable_display(bool state){
	if (state)
		display_on_off |= (1 << BIT_D);
	else
		display_on_off &= ~(1 << BIT_D);
	sender->send_byte(display_on_off);
}

void Lcd_i2c::write_symbol(const uint8_t sym){
	this->sender->write_data(sym);
}

void Lcd_i2c::write_string(uint8_t *sym){
	while((*sym) != '\0'){
		this->write_symbol((*sym++));
	}
}

void Lcd_i2c::set_numbers_disp_line(display_line_cnt cnt){
	switch (cnt) {
		case DISP_1_LINE:{
			function_set &= ~(1 << BIT_N);
			break;
		}
		case DISP_2_LINE:{
			function_set |= (1 << BIT_N);
			break;
		}
		default:
			break;
	}
}

void Lcd_i2c::shift_display(shift_direction dir, uint8_t cnt){
	for (uint8_t i = 0; i < cnt; i++) {
		switch (dir) {
			case SHIFT_LEFT:{
				this->sender->send_byte(cursor_display_shift | (1 << BIT_S_C) | (1 << BIT_R_L));
				break;
			}
			case SHIFT_RIGHT:{
				this->sender->send_byte(cursor_display_shift | (1 << BIT_S_C));
				break;
			}
			default:
				break;
		}
	}
}

void Lcd_i2c::shift_cursor(shift_direction dir, uint8_t cnt){
	for (uint8_t i = 0; i < cnt; i++) {
		switch (dir) {
			case SHIFT_LEFT:{
				this->sender->send_byte(cursor_display_shift);
				break;
			}
			case SHIFT_RIGHT:{
				this->sender->send_byte(cursor_display_shift | (1 << BIT_R_L));
				break;
			}
			default:
				break;
		}
	}
	this->__delay_ms(2);
}

void Lcd_i2c::cursor_set_pos(uint8_t x, uint8_t y){
	if (x > 0x3f){
		x = 0x3f;
	}
	if (y > 1){
		y = 1;
	}
	this->curr_pos = x | (y << 6);
	this->__set_DDRAM_addr(this->curr_pos);
}

void Lcd_i2c::cursor_move_home(){
	this->sender->send_byte(this->return_home);
}

void Lcd_i2c::write_user_symbol(const uint8_t *arr, const uint8_t addres){
	this->__set_CGRAM_addr(addres * 8);
	for (uint8_t i = 0; i < 8; ++i) {
		this->sender->write_data(arr[i]);
	}
}

void Lcd_i2c::clear_display(){
	sender->send_byte(display_clear);
}

void Lcd_i2c::__set_4_bit_interface(){
	this->__delay_ms(15);
	sender->send_byte(0b11);
	this->__delay_ms(5);
	sender->send_byte(0b11);
	this->__delay_ms(1);
	sender->send_byte(0b11);
	this->__delay_ms(1);
	sender->send_byte(0b10);
}

void Lcd_i2c::__set_DDRAM_addr(uint8_t addr){
	this->DDRAM_addres &= 0b10000000;
	this->DDRAM_addres |= addr & 0b01111111;
	this->sender->send_byte(DDRAM_addres);
}

void Lcd_i2c::__set_CGRAM_addr(uint8_t addr){
	this->sender->send_byte(CGRAM_addres | (addr & 0x1f));
}

void Lcd_i2c::__delay_ms(uint32_t ms){
	LL_mDelay(ms);
}

void Lcd_i2c::init(){
	this->enable_light(true);
	this->__delay_ms(1);
	this->__set_4_bit_interface();

	/* setting */
	this->__delay_ms(1);
	sender->send_byte(function_set);

	/* Display off */
	this->__delay_ms(1);
	this->enable_display(0b00001000);

	/* dipsplay clear */
	this->__delay_ms(1);
	this->clear_display();

	/* setting mode */
	this->__delay_ms(1);
	sender->send_byte(entry_mode_set);
}


} /* namespace lcd */

#endif // USE_FULL_LL_DRIVER
