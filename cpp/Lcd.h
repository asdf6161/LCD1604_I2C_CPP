/*
 * Lcd_i2c.h
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER2
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

class Lcd_i2c {

public:
	/*
	 * addr - device addres for transmit
	 * f - function for transmit data to lcd
	 * */
	Lcd_i2c(uint8_t addr, func_out f, func_in fi);
	virtual ~Lcd_i2c();

// pb Methods
public:
	void init();
	void enable_light(bool state);
	void enable_cursor(bool state);
	void enable_blink(bool state);
	void enable_display(bool state);
	void clear_display();
	uint8_t get_addres();  // Todo

// pv Methods
private:
	void __send_with_strob(const uint8_t half_byte);
	void __send_half_bt(const uint8_t half_byte);
	void __send_full_bt(const uint8_t bt);
	void __set_4_bit_interface();
	bool __read_busy_flag();

// vars
private:
	uint8_t addres;
	func_out transmit8;
	func_in recive8;
	packet data;

	// default param
	uint8_t function_set 			= 0b00101100;  // {0 0 1 DL  N    F    -   -}
	uint8_t cursor_display_shift 	= 0b00010100;  // {0 0 0 1   S/C  R/L  -   -}
	uint8_t display_on_off 			= 0b00001111;  // {0 0 0 0   1    D    C   B}
	uint8_t entry_mode_set 			= 0b00000110;  // {0 0 0 0   0    1    I/D SH}
	uint8_t return_home				= 0b00000010;  // {0 0 0 0   0    0    1   -}
	uint8_t display_clear			= 0b00000001;  // {0 0 0 0   0    0    0   1}
};

} /* namespace lcd */

#endif /* LCD_I2C_H_ */
#endif // USE_FULL_LL_DRIVER
