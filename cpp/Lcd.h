/*
 * Lcd_i2c.h
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#ifndef LCD_I2C_H_
#define LCD_I2C_H_

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

class Lcd_i2c {

public:
	/*
	 * sender - class realize method for send from Lcd_sender_abstract
	 * size_x - number of symbols on the display on a horizontal
	 * size_y - number of symbols on the display on a vertical
	 * line_cnt - quantity of the displayed lines
	 * */
//	Lcd_i2c(Lcd_sender_abstract *sender, uint8_t size_x, uint8_t size_y);
	Lcd_i2c(Lcd_sender_abstract *sender, display_line_cnt line_cnt);
	Lcd_i2c(Lcd_sender_abstract *sender);
	virtual ~Lcd_i2c();

// pb Methods
public:
	void init();
	void enable_light(bool state);
	void enable_cursor(bool state);
	void enable_blink(bool state);
	void enable_display(bool state);
	void write_symbol(const uint8_t sym);
	void write_string(uint8_t *sym);
	/* for set numbers display line need reinit lcd */
	void set_numbers_disp_line(display_line_cnt cnt);
	void shift_display(shift_direction dir, uint8_t cnt);
	void shift_cursor(shift_direction dir, uint8_t cnt);
	/* arr have size 8, contain 5bit constant, address 0-7 */
	void write_user_symbol(const uint8_t *arr, const uint8_t addres);
	/* start x=0 y=0; x max 63; y max 1*/
	void cursor_set_pos(uint8_t x, uint8_t y);
	void cursor_move_home();
	void clear_display();

// pv Methods
private:
	void __set_4_bit_interface();
	void __set_DDRAM_addr(uint8_t addr);  /* pos on display */
	void __set_CGRAM_addr(uint8_t addr);
	void __delay_ms(uint32_t ms);
	bool __read_busy_flag();

// vars
private:
	Lcd_sender_abstract *sender = nullptr;
	uint8_t size_y;
	uint8_t size_x;
	uint8_t curr_pos;
	display_line_cnt line_cnt;

	// default param
	uint8_t DDRAM_addres			= 0b10000000;  // {1 AC6-AC0}
	uint8_t CGRAM_addres			= 0b01000000;  // {1 AC6-AC0}
	uint8_t function_set 			= 0b00100000;  // {0 0 1 DL  N    F    -   -}
	uint8_t cursor_display_shift 	= 0b00010000;  // {0 0 0 1   S/C  R/L  -   -}
	uint8_t display_on_off 			= 0b00001111;  // {0 0 0 0   1    D    C   B}
	uint8_t entry_mode_set 			= 0b00000110;  // {0 0 0 0   0    1    I/D SH}
	uint8_t return_home				= 0b00000010;  // {0 0 0 0   0    0    1   -}
	uint8_t display_clear			= 0b00000001;  // {0 0 0 0   0    0    0   1}
};

} /* namespace lcd */

#endif /* LCD_I2C_H_ */
#endif // USE_FULL_LL_DRIVER
