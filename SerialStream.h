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
	 * Save out-index of inbuffer.
	 * Restore() may be called later to re-set the index.
	 * There is NO safety implemented against data being overwritten in the mean time
	 * (although that could be done without too much trouble).
	 * This was intended to allow two different parsers to test the same slow hand-typed
	 * input
	 */
	void mark();

	/*
	 * Un-consume the input buffer data to the state at mark time.
	 */
	void restore();

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
	uint8_t _markTail;
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

	//extern void LEDEvent(uint8_t);
	inline void receive(uint8_t c) {
		uint8_t next = (_rxBuf.head+1) & _rxBuf.mask;
		if (next != _rxBuf.tail) {
			_rxBuf.data[_rxBuf.head] = c;
			//if (_rxBuf.data[_rxBuf.head] == 32)	LEDEvent(8);
			_rxBuf.head = next;
		}
	}

	inline void udre() {
		if (_txBuf.head == _txBuf.tail) {
			*_ucsrb &= ~(1<<UDRIE0);
		} else {
			*_udr = _txBuf.data[_txBuf.tail];
			_txBuf.tail = (_txBuf.tail+1) & _txBuf.mask;
		}
	}

	//inline void receive(uint8_t c);
	//inline void udre();
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
