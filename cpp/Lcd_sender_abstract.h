/*
 * Lcd_sender_abstract.h
 *
 *  Created on: 20 ���. 2019 �.
 *      Author: yura
 */

#ifndef LCD_SENDER_ABSTRACT_H_
#define LCD_SENDER_ABSTRACT_H_

class Lcd_sender_abstract {
public:
	Lcd_sender_abstract(){};
	virtual ~Lcd_sender_abstract(){};

// virtual methods
public:
	/*
	 * Must be send 4 bit data to lcd with E
	 * Has to have delay before send > 50 us
	 * */
#ifdef LCD_4_BIT
	virtual void send_half_byte(const uint8_t half_bt) = 0;
#endif

	virtual uint8_t read_busy_and_addres() = 0;
	virtual uint8_t read_data_from_ram() = 0;

	/*
	 * Must be send 8 bit data to lcd with two E
	 * High bit is first
	 * Has to have delay before send half byte > 50 us
	 * */
	virtual void send_byte(const uint8_t bt) = 0;

	/*
	 * Has to send a package of RS=1
	 * */
	virtual void write_data(uint8_t command) = 0;

	/*
	 * Method must be enable LED illumination
	 * If led is not control by program - set body method is empty
	 * */
	virtual void enable_led(const bool state) = 0;

};

#endif /* LCD_SENDER_ABSTRACT_H_ */
