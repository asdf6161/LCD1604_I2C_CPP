/*
 * Lcd_i2c.cpp
 *
 *  Created on: 18 янв. 2019 г.
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

Lcd::Lcd(Lcd_sender_abstract *sender, display_line_cnt line_cnt) {
	this->sender = sender;
	this->curr_pos = 0x0;
	this->line_cnt = line_cnt;
	this->init();
}

Lcd::Lcd(Lcd_sender_abstract *sender) {
	this->sender = sender;
	this->curr_pos = 0x0;
	this->line_cnt = DISP_2_LINE;
	this->init();
}

Lcd::~Lcd() {

}

bool Lcd::__read_busy_flag(){
	// Todo
	return false;
}

void Lcd::enable_light(bool state){
	sender->enable_led(state);
}

void Lcd::enable_cursor(bool state){
	if (state)
		display_on_off |= (1 << BIT_C);
	else
		display_on_off &= ~(1 << BIT_C);
	sender->send_byte(display_on_off);
}

void Lcd::enable_blink(bool state){
	if (state)
		display_on_off |= (1 << BIT_B);
	else
		display_on_off &= ~(1 << BIT_B);
	sender->send_byte(display_on_off);
}

void Lcd::enable_display(bool state){
	if (state){
		display_on_off |= (1 << BIT_D);
	}
	else{
		display_on_off &= ~(1 << BIT_D);
	}
	sender->send_byte(display_on_off);
}

void Lcd::enable_display_shift(bool state){
	if (state){
		entry_mode_set |= 1 << BIT_SH;
	} else {
		entry_mode_set &= ~(1 << BIT_SH);
	}
	sender->send_byte(entry_mode_set);
}

void Lcd::cursor_set_autoshift(shift_direction dir){
	switch (dir) {
		case SHIFT_RIGHT:{
			entry_mode_set |= 1 << BIT_I_D;
			break;
		}
		case SHIFT_LEFT:{
			entry_mode_set &= ~(1 << BIT_I_D);
			break;
		}
		default:
			break;
	}
	sender->send_byte(entry_mode_set);
}

void Lcd::write_symbol(const uint8_t sym){
	this->sender->write_data(sym);
	this->curr_pos++;
}

void Lcd::write_string(uint8_t *sym){
	while((*sym) != '\0'){
		this->write_symbol((*sym++));
	}
}

void Lcd::set_numbers_disp_line(display_line_cnt cnt){
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
	this->sender->send_byte(function_set);
}

void Lcd::__set_interface_data_length(interface_data_length cnt){
	switch (cnt) {
		case INTERFACE_4_BIT:{
			function_set &= ~(1 << BIT_D_L);
			break;
		}
		case INTERFACE_8_BIT:{
			function_set |= (1 << BIT_D_L);
			break;
		}
		default:
			break;
	}
	this->sender->send_byte(function_set);
}

void Lcd::set_display_font_type(font_type type){
	switch (type) {
		case FONT_5X11:{
			function_set &= ~(1 << BIT_F);
			break;
		}
		case FONT_5X8:{
			function_set |= (1 << BIT_F);
			break;
		}
		default:
			break;
	}
	this->sender->send_byte(function_set);
}

void Lcd::set_display_shift(shift_direction dir, uint8_t cnt){
	for (uint8_t i = 0; i < cnt; i++) {
		switch (dir) {
			case SHIFT_LEFT:{
				cursor_display_shift |= (1 << BIT_S_C);
				cursor_display_shift &= ~(1 << BIT_R_L);
				break;
			}
			case SHIFT_RIGHT:{
				cursor_display_shift |= (1 << BIT_S_C) | (1 << BIT_R_L);
				break;
			}
			default:
				break;
		}
		this->sender->send_byte(cursor_display_shift);
	}
}

void Lcd::set_cursor_shift(shift_direction dir, uint8_t cnt){
	for (uint8_t i = 0; i < cnt; i++) {
		switch (dir) {
			case SHIFT_LEFT:{
				cursor_display_shift &= ~(1 << BIT_R_L);
				this->curr_pos--;
				break;
			}
			case SHIFT_RIGHT:{
				cursor_display_shift |= (1 << BIT_R_L);
				this->curr_pos++;
				break;
			}
			default:
				break;
		}
		this->sender->send_byte(cursor_display_shift);
	}
}

void Lcd::cursor_set_pos(uint8_t x, uint8_t y){
#ifdef LCD1602
	if (x > 0x3f){
		x = 0x3f;
	}
	if (y > 1){
		y = 1;
	}
	this->curr_pos = x | (y << 6);
	this->__set_DDRAM_addr(this->curr_pos);
#endif
#ifdef LCD1604
	if (x > 0x0f){
		x = 0x0f;
	}
	if (y > 3){
		y = 3;
	}
	if (y == 0) y = 0;
	if (y == 1) y = 0x40;
	if (y == 2) y = 0x10;
	if (y == 3) y = 0x50;
	this->curr_pos = x | y;
	this->__set_DDRAM_addr(this->curr_pos);
#endif
}

void Lcd::cursor_return_home(){
	this->sender->send_byte(this->return_home);
	this->curr_pos = 0;
}

void Lcd::write_user_symbol(const uint8_t *arr, const uint8_t addres){
	this->__set_CGRAM_addr((addres%8) * 8);
	for (uint8_t i = 0; i < 8; ++i) {
		this->sender->write_data(arr[i]);
	}
	this->__set_DDRAM_addr(curr_pos); // Возвращяем курсор
}

void Lcd::clear_display(){
	sender->send_byte(display_clear);
}

uint8_t Lcd::get_cursor_pos_x(){
	return (this->curr_pos % MAX_HORISONTAL_CELL);
}

uint8_t Lcd::get_cursor_pos_y(){
	return (this->curr_pos / MAX_HORISONTAL_CELL);
}

Lcd_sender_abstract *Lcd::get_sender(){
	return this->sender;
}

void Lcd::__set_DDRAM_addr(uint8_t addr){
	this->DDRAM_addres &= 0b10000000;
	this->DDRAM_addres |= addr & 0b01111111;
	this->sender->send_byte(DDRAM_addres);
}

void Lcd::__set_CGRAM_addr(uint8_t addr){
	CGRAM_addres &= ~(0x3f);
	CGRAM_addres |= (addr & 0x3f);
	this->sender->send_byte(CGRAM_addres);
}

void Lcd::__delay_ms(uint32_t ms){
	LL_mDelay(ms);
}

void Lcd::__delay_us(uint32_t us){
	if (us < 1000)
		this->__delay_ms(1);
	else
		this->__delay_ms(us / 1000);
}

void Lcd::init(){
//	this->sender->send_byte(DDRAM_addres);
//	this->sender->send_byte(CGRAM_addres);
//	this->sender->send_byte(function_set);
//	this->sender->send_byte(cursor_display_shift);
//	this->sender->send_byte(display_on_off);
//	this->sender->send_byte(entry_mode_set);
//	this->sender->send_byte(return_home);
//	this->sender->send_byte(display_clear);

//	for (uint8_t var = 0; var < 2; ++var) {  // Todo - Fix
		this->__delay_ms(40);
		this->enable_light(true);
#ifdef LCD_4_BIT
		this->sender->send_half_byte(0b10);
		this->sender->send_byte(0b101000);
		this->sender->send_byte(0b101000);
#endif
#ifndef LCD_4_BIT
		this->sender->send_byte(0b00111000);
		this->sender->send_byte(0b00111000);
#endif
		this->sender->send_byte(0b00001111);
		this->sender->send_byte(1);
		this->sender->send_byte(0b110);
//	}
	// fix param
	function_set |= 0b101000;
	display_on_off |= 0b00001111;
	entry_mode_set |= 0b110;
}


} /* namespace lcd */

#endif // USE_FULL_LL_DRIVER
