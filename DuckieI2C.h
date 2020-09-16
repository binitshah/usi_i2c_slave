/**********************************************************************************************
 * Arduino Attiny861 I2C Library - Version 1.0.0
 * by Binit shah <bshah@gatech.edu>
 *
 * Github user "usedbytes" wrote an I2C state machine library. It seems like the creators of
 * DuckieTown then modified the library to work between the ATTiny861V microcontroller and
 * a Raspberry Pi 3B+. The library wasn't written for the Arduino ecosystem, so this
 * "DuckieI2C" library modifies the DuckieTown version to support Arduino. This library
 * should work with SpenceKonde's AttinyCore.
 * 
 * There are many Arduino I2C libraries written for the Attinys (SpenceKonde, puuu, rambo, etc.)
 * however, not of them worked between the Duckietown Hat v2.1 and the Raspberry Pi.
 * Attiny861s have a USI module that emulates I2C in a finicky way and requires clock-
 * stretching, which Raspberry Pis don't support well.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#ifndef DuckieI2C_h
#define DuckieI2C_h

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/*
 * Set these appropriately for your platform,
 * currently set for Attiny861V.
 */
#define USI_PORT PORTA
#define USI_DDR DDRA
#define USI_PIN PINA
#define I2C_SDA 0
#define I2C_SCL 2

/*
 * The library supports a write mask for each individual register (bits set are
 * writable) in the i2c_w_mask array. If you don't care about masks for each
 * individual register, you can define a global value to be used for all
 * registers here, saving flash and RAM
 */
#define I2C_GLOBAL_WRITE_MASK 0xFF

#define I2C_SDA_DIR_OUT 1
#define I2C_SDA_DIR_IN 0

#define NAK() USIDR = 0x80
#define ACK() USIDR = 0x00
#define IDLE_SDA() USIDR = 0xff

#define I2C_STATE_ADDR_MATCH   0
#define I2C_STATE_REG_ADDR     1
#define I2C_STATE_MASTER_READ  2
#define I2C_STATE_MASTER_WRITE 3
#define I2C_STATE_IDLE         4

volatile uint8_t *i2c_reg;
volatile uint8_t i2c_addr = 0x30;
volatile uint8_t i2c_num_reg = 0;
volatile uint8_t i2c_update = 0;
volatile uint8_t i2c_state = I2C_STATE_ADDR_MATCH;
volatile uint8_t i2c_offset = 0;

/* USI i2c Slave State Machine
 * ===========================
 *
 * 5 States:
 *     0 I2C_STATE_ADDR_MATCH
 *       Waiting for address (start)
 *
 *     1 I2C_STATE_REG_ADDR
 *       Receive register address*
 *
 *     2 I2C_STATE_MASTER_READ
 *       Transmit data to master
 *
 *     3 I2C_STATE_MASTER_WRITE
 *       Receive data from master
 *
 *     4 I2C_STATE_IDLE
 *       Bus idle/address not matched
 *
 * Valid state transitions:
 *      __To__________
 *      0  1  2  3  4
 * F 0|    a  b     h
 * r 1|          d  ci
 * o 2|       f     e
 * m 3|          g  c
 *   4| j
 *
 * Transition j - Start of transaction
 *  I2C_STATE_IDLE -> I2C_STATE_ADDR_MATCH
 *  Cond:   Start condition interrupt
 *  Action: None.
 *
 * Transition h - Address not matched.
 *  I2C_STATE_ADDR_MATCH -> I2C_STATE_IDLE
 *  Cond:   Pre-ack. Address doesn't match
 *  Action: NAK.
 *
 * Transition a - Address matched, write mode
 *  I2C_STATE_ADDR_MATCH -> I2C_STATE_REG_ADDR
 *  Cond:   Pre-ack. Address matches, bit0 == 0
 *  Action: ACK, Reset reg pointer.
 *
 * Transition b - Address matched, read mode
 *  I2C_STATE_ADDR_MATCH -> I2C_STATE_MASTER_READ
 *  Cond:   Pre-ack. Address matches, bit0 == 1
 *  Action: ACK.
 *
 * Transition c - Write finished
 *  I2C_STATE_XXX -> I2C_STATE_IDLE
 *  Cond:   Stop flag is set.
 *  Action: None.
 *
 * Transition d - Initialise write
 *  I2C_STATE_REG_ADDR -> I2C_STATE_MASTER_WRITE
 *  Cond:   Pre-ack.
 *  Action: ACK, reg_ptr = USIDR.
 *
 * Transition i - Invalid reg addr
 *  I2C_STATE_REG_ADDR -> I2C_STATE_IDLE
 *  Cond:   Pre-ack, USIDR > i2c_num_reg - 1
 *  Action: NAK.
 *
 * Transition e - Read finished
 *  I2C_STATE_MASTER_READ -> I2C_STATE_IDLE
 *  Cond:   Post-ack. Master NAK'd.
 *  Action: None.
 *
 * Transition f - Read continues
 *  I2C_STATE_MASTER_READ -> I2C_STATE_MASTER_READ
 *  Cond:   Post-ack. Master ACK'd.
 *  Action: USIDR = *reg_ptr++
 *
 * Transition g - Write continues
 *  I2C_STATE_MASTER_WRITE -> I2C_STATE_MASTER_WRITE
 *  Cond:   Pre-ack.
 *  Action: ACK, *reg_ptr++ = USIDR
 *
 */

 /*
  * For some reason, avr-libc uses different vector names for the USI
  * on different chips! We have to workaround that here
  */
#if defined(USI_START_vect)
ISR(USI_START_vect)
#elif defined(USI_STRT_vect)
ISR(USI_STRT_vect)
#else
#error "Couldn't figure out what i2c start interrupt to use!"
#endif
{
	i2c_state = I2C_STATE_ADDR_MATCH;
	IDLE_SDA();
	while (USI_PIN & (1 << I2C_SCL));
	USISR = 0xF0;
}

#if defined(USI_OVERFLOW_vect)
ISR(USI_OVERFLOW_vect)
#elif defined(USI_OVF_vect)
ISR(USI_OVF_vect)
#else
#error "Couldn't figure out what i2c overflow interrupt to use!"
#endif
{
	static uint8_t post_ack = 0;
	/* Writing USISR directly has side effects! */
	uint8_t usisr_tmp = 0xD0;
	uint8_t sda_direction;
	uint8_t tmp;

	if (!post_ack) {
		/* Work that needs to be done before the ACK cycle */
		sda_direction = I2C_SDA_DIR_OUT;

		switch (i2c_state) {
		case I2C_STATE_ADDR_MATCH:
			tmp = USIDR >> 1;
			if (tmp && (tmp != i2c_addr)) {
				/* Transition h */
				i2c_state = I2C_STATE_IDLE;
				NAK();
			} else {
				if (USIDR & 1) {
					/* Transition b: Address matched, read mode */
					i2c_state = I2C_STATE_MASTER_READ;
				} else {
					/* Transition a: Address matched, write mode */
					i2c_offset = 0;
					i2c_state = I2C_STATE_REG_ADDR;
					i2c_update = 1;
				}
				ACK();
			}
			break;
		case I2C_STATE_REG_ADDR:
			tmp = USIDR;
			if (tmp > (i2c_num_reg - 1)) {
				/* Transition i:  Invalid reg addr*/
				i2c_state = I2C_STATE_IDLE;
				NAK();
			} else {
				/* Transition d:  Initialise write*/
				i2c_offset = tmp;
				i2c_state = I2C_STATE_MASTER_WRITE;
				ACK();
			}
			break;
		case I2C_STATE_MASTER_READ:
			USIDR = 0;
			/* Listen for master NAK */
			sda_direction = I2C_SDA_DIR_IN;
			break;
		case I2C_STATE_MASTER_WRITE:
#if defined(I2C_GLOBAL_WRITE_MASK)
			tmp = I2C_GLOBAL_WRITE_MASK;
#else
			tmp = i2c_w_mask[i2c_offset];
#endif
			if (tmp) {
				/* Only heed writeable bits */
				i2c_reg[i2c_offset] &= ~tmp;
				i2c_reg[i2c_offset] |= USIDR & tmp;
			}
			i2c_update++;
			i2c_offset++;
			ACK();
			break;
		default:
			NAK();
		}
		/* Counter will overflow again after ACK cycle */
		usisr_tmp |= 14 << USICNT0;
		post_ack = 1;
	} else {
		/* Work that needs to be done after the ACK cycle */
		sda_direction = I2C_SDA_DIR_IN;
		switch (i2c_state) {
		case I2C_STATE_MASTER_READ:
			if (USIDR) {
				/* Transition e: Read finished */
				i2c_offset = 0;
				i2c_state = I2C_STATE_IDLE;
				IDLE_SDA();
			} else {
				/* Transition f: Read continues */
				sda_direction = I2C_SDA_DIR_OUT;
				USIDR = i2c_reg[i2c_offset++];
			}
			break;
		case I2C_STATE_IDLE:
		case I2C_STATE_REG_ADDR:
		case I2C_STATE_MASTER_WRITE:
			IDLE_SDA();
			break;
		/* I2C_STATE_ADDR_MATCH should never reach here */
		}
		post_ack = 0;
	}

	if (i2c_offset > (i2c_num_reg - 1))
		i2c_offset = 0;

	/* Set up SDA direction for next operation */
	if (sda_direction == I2C_SDA_DIR_OUT) {
		USI_DDR |= (1 << I2C_SDA);
	} else {
		USI_DDR &= ~(1 << I2C_SDA);
	}

	/* Clear flags and set counter */
	USISR = usisr_tmp;
}

class DuckieI2C {
    public:
        /*
         * @param addr slave address for uC
         * @param data pointer to data array
         * @param size size of the data array
         */
        DuckieI2C(uint8_t addr, uint8_t *data, uint8_t size);

        /*
         * Run this method once in the Arduino setup.
         */
        void begin();

        /*
         * Tracks whether new data has been pushed to
         * the data array.
         *
         * @return bool if data is updated
         */
        bool isDataUpdated();
};

// -------- Public Methods -------- //

DuckieI2C::DuckieI2C(uint8_t addr, uint8_t *data, uint8_t size) {
    i2c_addr = addr;
    i2c_reg = data;
    i2c_num_reg = size;
}

void DuckieI2C::begin() {
    //enable second USI Port (PA0:2)
	USIPP=0x01;
    i2c_state = 0;

	IDLE_SDA();

	USICR = (1 << USISIE) | (1 << USIOIE) | (3 << USIWM0) | (1 << USICS1);

	USI_DDR |= (1 << I2C_SDA) | (1 << I2C_SCL);
	USI_PORT |= (1 << I2C_SDA) | (1 << I2C_SCL);

	USISR = 0xF0;

	// //enable interrupts
	sei();
}

bool DuckieI2C::isDataUpdated() {
    uint8_t ret = 0;
	if ((i2c_state == I2C_STATE_MASTER_WRITE) && i2c_update) {
		cli();
		uint8_t tmp = USISR;
		if (tmp & (1 << USIPF)) {
			i2c_state = I2C_STATE_IDLE;
			ret = i2c_update;
			i2c_update = 0;
		}
		sei();
	}

	return ret;
}

#endif // DuckieI2C_h