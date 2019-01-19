/*
 * Lcd_sender_abstract.h
 *
 *  Created on: 20 џэт. 2019 у.
 *      Author: yura
 */

#ifndef LCD_SENDER_ABSTRACT_H_
#define LCD_SENDER_ABSTRACT_H_

class Lcd_sender_abstract {
public:
	Lcd_sender_abstract();
	virtual ~Lcd_sender_abstract();

// virtual methods
public:
	/*
	 * Must be send 4 bit data to lcd with E
	 * Has to have delay before send > 50 us
	 * */
	virtual void send_half_byte(const uint8_t half_bt) = 0;

	/*
	 * Must be send 8 bit data to lcd with two E
	 * High bit is first
	 * */
	virtual void send_full_byte(const uint8_t bt);

	/*
	 * Method must be enable LED illumination
	 * If led is not control by program - set body method is empty
	 * */
	void enable_led(const bool state);

};

#endif /* LCD_SENDER_ABSTRACT_H_ */
