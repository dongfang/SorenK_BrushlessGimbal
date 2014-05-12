#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Board.h"
#include "I2C.h"
#include "Util.h"

#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);

volatile uint8_t i2c_errors_count;
uint8_t i2c_interrupt_hits[8];

// Okay this is not pretty.
// We could make a struct representing this and let caller pass a pointer to one, if we need to.
uint8_t i2c_buffer[12];
volatile uint8_t i2c_async_status;
static volatile uint8_t i2c_async_sladr;
static volatile uint8_t i2c_async_reg;
static volatile uint8_t i2c_async_datacnt;
static volatile uint8_t i2c_async_bufidx;

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                 // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // set the I2C clock rate to 100kHz
  TWCR = 1<<TWEN;							// enable twi module, possiblhy also interrupt
  i2c_async_status = -1; // an invalid value.
}

void i2c_shutdown(void) {
	TWCR &= ~(1<<TWEN);
}


// Synchronized (blocking till done).
void i2c_busy_wait() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      // neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}
// Synchronized (blocking till done).
void i2c_rep_start(uint8_t address) {
  TWCR = 1<<TWINT | 1<<TWSTA | 1<<TWEN ; // send REPEAT START condition
  i2c_busy_wait();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = 1<<TWINT | 1<<TWEN;
  i2c_busy_wait();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = 1<<TWINT | 1<<TWEN | 1<<TWSTO;
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

// Synchronized (blocking till done).
void i2c_write(uint8_t data) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = 1<<TWINT | 1<<TWEN;
  i2c_busy_wait();
}

// Synchronized (blocking till done).
uint8_t i2c_read(uint8_t ack) {
  TWCR = 1<<TWINT | 1<<TWEN | (ack? (1<<TWEA) : 0);
  i2c_busy_wait();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

// Synchronized (blocking till done).
void i2c_read_regs(uint8_t add, uint8_t reg, uint8_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  uint8_t* b = i2c_buffer;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
  }
}

// Synchronized (blocking till done).
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();

  i2c_read_regs(add, reg, 1);
  bool mismatchIsOkay = (reg==0x68 || reg==0x6b);

  if (val != i2c_buffer[0] && !mismatchIsOkay) {
	  printf_P(PSTR("Verification mismatch: I2C device %x at address %x (wrote %x, read %x)\r\n"), add, reg, val, i2c_buffer[0]);
	  //printf_P(PSTR("(For MPU6050 addresses 68 and 6b it is okay)\r\n"), add, reg, val, i2c_buffer[0]);
  }
}

ISR(TWI_vect) {
	//if (i2c_async_status != I2C_ASYNC_DATA || i2c_async_bufidx == 0)
	i2c_interrupt_hits[i2c_async_status] = TWSR;
	switch(i2c_async_status) {
	case I2C_ASYNC_STARTED_1:
		TWDR = i2c_async_sladr<<1;
		// TWCR = 1<<TWINT | 1<<TWEN | 1<<TWIE; // this kills it! Why?
		TWCR |= 1<<TWINT;
		i2c_async_status++;
		break;
	case I2C_DEVADDR_SENT_1:
		TWDR = i2c_async_reg;
		TWCR = 1<<TWINT | 1<<TWEN | 1<<TWIE;
		i2c_async_status++;
		break;
	case I2C_REGADDR_SENT:
		TWCR = 1<<TWINT | 1<<TWSTA | 1<<TWEN | 1<<TWIE; // send REPEAT START condition
		i2c_async_status++;
		break;
	case I2C_ASYNC_STARTED_2:
		TWDR = (i2c_async_sladr<<1) | 1;
		TWCR = 1<<TWINT | 1<<TWEN | 1<<TWIE;
		i2c_async_status++;
		break;
	case I2C_DEVADR_SENT_2:
		// Nothing? Or first data byte? Dunno.
		TWCR = 1<<TWINT | 1<<TWEN | 1<<TWIE | 1<<TWEA;
		i2c_async_status++;
		i2c_async_bufidx = 0;
		break;
	case I2C_ASYNC_DATA:
		i2c_buffer[i2c_async_bufidx] = TWDR;
		i2c_async_bufidx++;
		if (i2c_async_bufidx == i2c_async_datacnt) {
			i2c_async_status++;
			TWCR = 1<<TWEN | 1<<TWIE | 1<<TWINT; // send nak
		} else {
			TWCR = 1<<TWEN | 1<<TWIE | 1<<TWINT | 1<<TWEA; // send ack
		}
		break;
	default:
		i2c_stop();
		break;
	}
}

void i2c_read_regs_async(uint8_t add, uint8_t reg, uint8_t size) {
	i2c_async_datacnt = size;
	i2c_async_bufidx = 0;
	i2c_async_sladr = add;
	i2c_async_reg = reg;
	i2c_async_status = I2C_ASYNC_STARTED_1;
	TWCR = 1<<TWINT | 1<<TWSTA | 1<<TWEN | 1<<TWIE; // send REPEAT START condition
}

// This will wait till the async handling it completed but not reset the watchdog timer.
// If it takes too long (glitch), bang, watchdog reboot.
void i2c_wait_async_done() {
	// debug_i2c_status[debug_measureing_what] = i2c_async_status;
	if(i2c_async_status == I2C_ASYNC_DONE) return;
	// LED_PIN |= (1<<LED_PIN);
	// wait_16_micros(10);
	// This should in almost all cases just result in a brief wait.
	// If it takes long, such as if the more or less self-sustained
	// I2C interrupt handling has stalled, the WDT should shoot it down.
	// (TODO: That happens but it stalls after restart, for some obscure reason).
	_delay_us(5);

	if(i2c_async_status != I2C_ASYNC_DONE) {
		LED_PIN |= (1<<LED_BIT);
	}
}
