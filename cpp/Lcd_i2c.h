/*
 * Lcd_i2c.h
 *
 *  Created on: 18 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "stdint.h"

extern "C" {
#include "stm32f3xx_ll_utils.h"
}

namespace lcd {

// function type for transmit data to lcd
typedef void (*func_out) (uint8_t, uint8_t);
typedef uint8_t (*func_in) (uint8_t);

typedef union
{
  struct
  {
    uint8_t RS : 1;
    uint8_t RW : 1;
    uint8_t EN : 1;
    uint8_t LED : 1;
    uint8_t DB4 : 1;
    uint8_t DB5 : 1;
    uint8_t DB6 : 1;
    uint8_t DB7 : 1;
  }bits;
  unsigned char byte;
} packet;


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

// pv Methods
private:
	void __send_with_strob();
	void __send_half_bt(uint8_t half_byte);
	void __send_full_bt(uint8_t byte);
	bool __read_busy_flag();

// vars
private:
	uint8_t addres;
	func_out transmit8;
	func_in recive8;
	packet data;
};

} /* namespace lcd */

#endif /* LCD_I2C_H_ */
#endif // USE_FULL_LL_DRIVER
