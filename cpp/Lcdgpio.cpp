/*
 * Lcdgpio.cpp
 *
 *  Created on: 22 џэт. 2019 у.
 *      Author: yura
 */

#ifdef USE_FULL_LL_DRIVER
#include <Lcdgpio.h>

namespace Lcd_gpio {

Lcd_gpio::Lcd_gpio(GPIO_control_union *gpios, PORT_control_union *ports) {
	this->gpios = gpios;
	this->ports = ports;
}

Lcd_gpio::~Lcd_gpio() {

}

#ifdef LCD_4_BIT
void Lcd_gpio::send_half_byte(const uint8_t half_bt){
	__send_with_strobe(half_bt);
}
#endif

void Lcd_gpio::send_byte(const uint8_t bt){
#ifdef LCD_4_BIT
	this->send_half_byte(bt >> 4);
	this->send_half_byte(bt);
#endif
	__send_with_strobe(bt);
}

void Lcd_gpio::write_data(uint8_t command){
	LL_GPIO_SetOutputPin(gpios->gpios.GPIO_RS, 1 << ports->ports.PORT_RS);
	LL_GPIO_ResetOutputPin(gpios->gpios.GPIO_RW, 1 << ports->ports.PORT_RW);
	send_byte(command);
	LL_GPIO_ResetOutputPin(gpios->gpios.GPIO_RS, 1 << ports->ports.PORT_RS);
}

void Lcd_gpio::enable_led(const bool state){
	if (state){
		LL_GPIO_SetOutputPin(gpios->gpios.GPIO_LED, 1 << ports->ports.PORT_LED);
	} else {
		LL_GPIO_ResetOutputPin(gpios->gpios.GPIO_LED, 1 << ports->ports.PORT_LED);
	}
}


void Lcd_gpio::__set_e(){
	asm("nop");
	LL_GPIO_SetOutputPin(gpios->gpios.GPIO_E, 1 << ports->ports.PORT_E);
	__delay_us(250);
}

void Lcd_gpio::__reset_e(){
	__delay_us(250);
	LL_GPIO_ResetOutputPin(gpios->gpios.GPIO_E, 1 << ports->ports.PORT_E);
	asm("nop");
}

void Lcd_gpio::__set_D(uint8_t byte){
	__reset_D();
	for (uint8_t i = 3; i < PORT_CNT-1; ++i) {
		if ((byte >> (i-3)) & 0b1){
			LL_GPIO_SetOutputPin(gpios->gpios_arr[i], 1 << ports->ports_arr[i]);
		}
	}
}

void Lcd_gpio::__reset_D(){
	for (uint8_t i = 3; i < PORT_CNT-1; ++i) {
		LL_GPIO_ResetOutputPin(gpios->gpios_arr[i], 1 << ports->ports_arr[i]);
	}
}

void Lcd_gpio::__strobe(){
	asm("nop");
	LL_GPIO_SetOutputPin(gpios->gpios.GPIO_E, 1 << ports->ports.PORT_E);
	__delay_us(500);
	LL_GPIO_ResetOutputPin(gpios->gpios.GPIO_E, 1 << ports->ports.PORT_E);
	asm("nop");
}

void Lcd_gpio::__delay_ms(uint32_t ms){
	LL_mDelay(ms);
}

void Lcd_gpio::__delay_us(uint32_t us){
	if (us < 1000){
		this->__delay_ms(1);
	}
	else{
		this->__delay_ms(us / 1000);
	}
}

void Lcd_gpio::__send_with_strobe(uint8_t byte){
	__set_e();
	__set_D(byte);
	__reset_e();
	asm("nop");
	asm("nop");
	__reset_D();
}

} /* namespace Lcd_gpio */

#endif // USE_FULL_LL_DRIVER2
