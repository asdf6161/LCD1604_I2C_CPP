/*
 * Lcdgpio.h
 *
 *  Created on: 22 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#ifndef LCDGPIO_H_
#define LCDGPIO_H_

#include "stdint.h"
#include "Lcd_sender_abstract.h"

extern "C" {
#ifdef STM32F303xC
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_gpio.h"
#endif
}

namespace Lcd_gpio {

/*
 * GPIO must be init
 * */

#ifndef LCD_4_BIT
#define PORT_CNT 12
union GPIO_control_union{
	struct {
		GPIO_TypeDef *GPIO_RS;
		GPIO_TypeDef *GPIO_RW;
		GPIO_TypeDef *GPIO_E;
		GPIO_TypeDef *GPIO_D0;
		GPIO_TypeDef *GPIO_D1;
		GPIO_TypeDef *GPIO_D2;
		GPIO_TypeDef *GPIO_D3;
		GPIO_TypeDef *GPIO_D4;
		GPIO_TypeDef *GPIO_D5;
		GPIO_TypeDef *GPIO_D6;
		GPIO_TypeDef *GPIO_D7;
		GPIO_TypeDef *GPIO_LED;
	}gpios;
	GPIO_TypeDef *gpios_arr[12];
};

union PORT_control_union{
	struct {
		uint8_t PORT_RS;
		uint8_t PORT_RW;
		uint8_t PORT_E;
		uint8_t PORT_D0;
		uint8_t PORT_D1;
		uint8_t PORT_D2;
		uint8_t PORT_D3;
		uint8_t PORT_D4;
		uint8_t PORT_D5;
		uint8_t PORT_D6;
		uint8_t PORT_D7;
		uint8_t PORT_LED;
	}ports;
	uint8_t ports_arr[12];
};
#endif
#ifdef LCD_4_BIT
#define PORT_CNT 8
union GPIO_control_union{
	struct {
		GPIO_TypeDef *GPIO_RS;
		GPIO_TypeDef *GPIO_RW;
		GPIO_TypeDef *GPIO_E;
		GPIO_TypeDef *GPIO_D4;
		GPIO_TypeDef *GPIO_D5;
		GPIO_TypeDef *GPIO_D6;
		GPIO_TypeDef *GPIO_D7;
		GPIO_TypeDef *GPIO_LED;
	}gpios;
	GPIO_TypeDef *gpios_arr[8];
};

union PORT_control_union{
	struct {
		uint8_t PORT_RS;
		uint8_t PORT_RW;
		uint8_t PORT_E;
		uint8_t PORT_D4;
		uint8_t PORT_D5;
		uint8_t PORT_D6;
		uint8_t PORT_D7;
		uint8_t PORT_LED;
	}ports;
	uint8_t ports_arr[8];
};
#endif

class Lcd_gpio : public Lcd_sender_abstract{
public:
	Lcd_gpio(GPIO_control_union *gpios, PORT_control_union *ports);
	virtual ~Lcd_gpio();

// PB methods
public:
	/*
	 * Must be send 4 bit data to lcd with E
	 * Has to have delay before send > 50 us
	 * */
#ifdef LCD_4_BIT
	void send_half_byte(const uint8_t half_bt);
#endif

	uint8_t read_busy_and_addres();
	uint8_t read_data_from_ram();

	/*
	 * Must be send 8 bit data to lcd with two E
	 * High bit is first
	 * Has to have delay before send half byte > 50 us
	 * */
	void send_byte(const uint8_t bt);

	/*
	 * Has to send a package of RS=1
	 * */
	void write_data(uint8_t command);

	/*
	 * Method must be enable LED illumination
	 * If led is not control by program - set body method is empty
	 * */
	void enable_led(const bool state);

	void init_pin();

// PV methods
private:
	void __strobe();
	void __set_e();
	void __reset_e();
	void __set_D(uint8_t byte);
	void __reset_D();
	void __delay_ms(uint32_t ms);
	void __delay_us(uint32_t us);
	void __send_with_strobe(uint8_t byte);
	void __init_pin_out();
	void __init_pin_in();
	uint8_t __read_port();
	uint8_t __read_port_with_strobe();

// vars
private:
	GPIO_control_union *gpios;
	PORT_control_union *ports;
};

} /* namespace Lcd_gpio */

#endif /* LCDGPIO_H_ */
#endif // USE_FULL_LL_DRIVER2
