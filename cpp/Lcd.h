/*
 * Lcd_i2c.h
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#define MAX_HORISONTAL_CELL 40  // DEC non HEX
#define BIT_I_D 1
#define BIT_SH  0
#define BIT_S_C 3
#define BIT_R_L 2
#define BIT_D_L 4
#define BIT_N   3
#define BIT_F   2
#define BIT_D   2
#define BIT_C   1
#define BIT_B   0

#include "stdint.h"
#include "Pcf8574t.h"
#include "Lcd_sender_abstract.h"

extern "C" {
#ifdef STM32F303xC
#include "stm32f3xx_ll_utils.h"
#endif
}

namespace lcd {

enum shift_direction {
	SHIFT_RIGHT,
	SHIFT_LEFT,
//	SHIFT_UP, // Todo
//	SHIFT_DOWN, // Todo
};

enum display_line_cnt {
	DISP_2_LINE,
	DISP_1_LINE,
};

enum interface_data_length {
	INTERFACE_8_BIT,
	INTERFACE_4_BIT,
};

enum font_type {
	FONT_5X11,
	FONT_5X8,
};

class Lcd {

public:
	/*
	 * sender - class realize method for send from Lcd_sender_abstract
	 * size_x - number of symbols on the display on a horizontal
	 * size_y - number of symbols on the display on a vertical
	 * line_cnt - quantity of the displayed lines
	 * */
//	Lcd_i2c(Lcd_sender_abstract *sender, uint8_t size_x, uint8_t size_y);
	Lcd(Lcd_sender_abstract *sender, display_line_cnt line_cnt);
	Lcd(Lcd_sender_abstract *sender);
	virtual ~Lcd();

// pb Methods
public:
	void init();
	void enable_light(bool state);
	void enable_cursor(bool state);
	void enable_blink(bool state);
	void enable_display(bool state);
	void enable_display_shift(bool state);

	void cursor_set_pos(uint8_t x, uint8_t y);
	void cursor_set_autoshift(shift_direction dir);
	void cursor_return_home();

	/* arr have size 8, contain 5bit constant, address 0-7 */
	void write_user_symbol(const uint8_t *arr, const uint8_t addres);
	void write_symbol(const uint8_t sym);
	void write_string(uint8_t *sym);

	void set_numbers_disp_line(display_line_cnt cnt);
	void set_interface_data_length(interface_data_length cnt);
	void set_display_font_type(font_type type);
	void set_display_shift(shift_direction dir, uint8_t cnt);
	void set_cursor_shift(shift_direction dir, uint8_t cnt);

	bool read_busy();
	uint8_t read_cursor_addres();
	/* after reading a symbol the cursor will mix up on 1 */
	uint8_t read_symbol_addres();

	void clear_display();

	/* getters and settors */
	uint8_t get_cursor_pos_x();
	uint8_t get_cursor_pos_y();
	Lcd_sender_abstract *get_sender();

// pv Methods
private:
	void __set_interface_data_length(interface_data_length cnt);
	void __set_DDRAM_addr(uint8_t addr);  /* pos on display */
	void __set_CGRAM_addr(uint8_t addr);
	void __delay_ms(uint32_t ms);
	void __delay_us(uint32_t us);

// vars
private:
	Lcd_sender_abstract *sender = nullptr;
	uint8_t curr_pos;
	display_line_cnt line_cnt;

	// default param
	uint8_t DDRAM_addres			= 0b10000000;  // {1 AC6-AC0}
	uint8_t CGRAM_addres			= 0b01000000;  // {1 AC5-AC0}
	uint8_t function_set 			= 0b00100000;  // {0 0 1 DL  N    F    -   -}
	uint8_t cursor_display_shift 	= 0b00010000;  // {0 0 0 1   S/C  R/L  -   -}
	uint8_t display_on_off 			= 0b00001000;  // {0 0 0 0   1    D    C   B}
	uint8_t entry_mode_set 			= 0b00000100;  // {0 0 0 0   0    1    I/D SH}
	uint8_t return_home				= 0b00000010;  // {0 0 0 0   0    0    1   -}
	uint8_t display_clear			= 0b00000001;  // {0 0 0 0   0    0    0   1}
};

} /* namespace lcd */

#endif /* LCD_I2C_H_ */
#endif // USE_FULL_LL_DRIVER
