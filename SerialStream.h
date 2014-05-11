#ifndef __SERIALSTREAM_H
#define __SERIALSTREAM_H

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

struct SerialRingBuffer {
	volatile uint8_t head, tail;
	uint8_t mask;
	volatile uint8_t* data;
};

/*
 * Configurable bit count, parity, stopbit count etc? Bah! 8N1.
 */
class Serial {
public:
	/*
	 * The direction param is a _FDEV_SETUP_* from stdio.h
	 */
	void init(uint32_t baud, int direction);

	/*
	 * Get a char, or -1 if inbuffer is empty.
	 */
	int get();
	int peek();

	/*
	 * Put a char.
	 */
	void put(uint8_t c);

	/*
	 * Number of chars available for get.
	 */
	size_t available();

	/*
	 * Throw away all inbuffer data.
	 */
	void clear();

	/*
	 * Block thread till outbuffer is empty.
	 */
	void flush();
protected:
	Serial(size_t txBufsiz, size_t rxBufsiz) {
		_txBuf.mask = txBufsiz - 1;
		_rxBuf.mask = rxBufsiz - 1;
		// Should get rid of this. Once the shit works.
		_txBuf.data = (uint8_t*)malloc(txBufsiz);
		_rxBuf.data = (uint8_t*)malloc(rxBufsiz);
	}
protected:
	SerialRingBuffer _txBuf;
	SerialRingBuffer _rxBuf;
};

#if defined(USART0_RX_vect)
extern "C"{
	void USART0_RX_vect(void);
	void USART_UDRE0_vect(void);
}
#endif
#if defined(USART_RX_vect)
extern "C"{
	void USART_UDRE_vect(void);
	void USART_RX_vect(void);
}
#endif

class UARTSerial: public Serial {
private:
	volatile uint8_t* const _ubrrh;
	volatile uint8_t* const _ubrrl;
	volatile uint8_t* const _ucsra;
	volatile uint8_t* const _ucsrb;
	volatile uint8_t* const _udr;
#if defined(USART0_RX_vect)
	friend void USART0_RX_vect;
	friend void USART_UDRE0_vect;
#endif
#if defined(USART_RX_vect)
	friend void ::USART_UDRE_vect(void);
	friend void ::USART_RX_vect(void);
#endif
	inline void receive(uint8_t c);
	inline void udre();
public:
	UARTSerial (
			size_t txBufsiz,
			size_t rxBufsiz,
			volatile uint8_t* ubrrh,
			volatile uint8_t* ubrrl,
			volatile uint8_t* ucsra,
			volatile uint8_t* ucsrb,
			volatile uint8_t* udr
	) : Serial(txBufsiz, rxBufsiz),
		_ubrrh(ubrrh),
		_ubrrl(ubrrl),
		_ucsra(ucsra),
		_ucsrb(ucsrb),
		_udr(udr) {
	}
	void init(uint32_t baud, int direction);
	void put(uint8_t c);
};

#endif
