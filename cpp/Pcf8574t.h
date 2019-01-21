/*
 * Pcf8574t.h
 *
 *  Created on: 20 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER

#ifndef PCF8574T_H_
#define PCF8574T_H_

#include "stdint.h"
#include "Lcd_sender_abstract.h"

extern "C" {
#ifdef STM32F303xC
#include "stm32f3xx_ll_utils.h"
#endif
}

namespace pcf {

// function type for transmit data to lcd
typedef void (*pcf_func_out) (const uint8_t, const uint8_t);
typedef uint8_t (*pcf_func_in) (const uint8_t);

typedef union
{
  struct
  {
    uint8_t RS : 1;		// P0
    uint8_t RW : 1;		// P1
    uint8_t EN : 1;		// P2
    uint8_t LED : 1;	// P3
    /*half byte*/
    uint8_t DB4 : 1;	// P4
    uint8_t DB5 : 1;	// P5
    uint8_t DB6 : 1;	// P6
    uint8_t DB7 : 1;	// P7
    /*half byte*/
  }bits;
  unsigned char byte;
} pcf_packet;

class Pcf8574t : public Lcd_sender_abstract {
public:
	/*
	 * addr  - address of module Pcf8574t
	 * f_out - i2c func for send data to Pcf8574t
	 * f_in  - i2c func for recv data from Pcf8574t
	 * */
	Pcf8574t(uint8_t addr, pcf_func_out f_out, pcf_func_in f_in);
	virtual ~Pcf8574t();

// methods
public:
	void send_half_byte(const uint8_t half_bt);
	void send_byte(const uint8_t bt);
	void write_data(uint8_t command);
	void enable_led(const bool state);

// PV methods
private:
	void __send_half_byte_with_strob(const uint8_t half_bt);
	void __send_strobe(pcf_packet *p);
	void __set_pin(pcf_packet *p);
	void __reset_data_pin(pcf_packet *p);
	void __delay_ms(uint32_t ms);  // Todo
	void __delay_us(uint32_t us);  // Todo

// vars
private:
	pcf_func_out f_out;
	pcf_func_in f_in;
	uint8_t addr;
	pcf_packet packet;

};

} /* namespace lcd */

#endif /* PCF8574T_H_ */
#endif // USE_FULL_LL_DRIVER2
