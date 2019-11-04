#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

// constants definition (from SHTC3 datasheet)
#define SHTC3_ADDRESS  0x70
#define SHTC3_READ_ID_REG 0xefc8

#define SHTC3_WAKE_UP  0x3517
#define SHTC3_SLEEP  0xb098
	
#define SHTC3_SOFT_RESET  0x805d

#define SHTC3_MEAS_NORMAL_T_FIRST_CLKS      0x7ca2
#define SHTC3_MEAS_NORMAL_H_FIRST_CLKS      0x5c24
#define SHTC3_MEAS_NORMAL_T_FIRST_NCLKS     0x7866
#define SHTC3_MEAS_NORMAL_H_FIRST_NCLKS     0x58e0
#define SHTC3_MEAS_LOW_POWER_T_FIRST_CLKS   0x6458
#define SHTC3_MEAS_LOW_POWER_H_FIRST_CLKS   0x44de
#define SHTC3_MEAS_LOW_POWER_T_FIRST_NCLKS  0x609c
#define SHTC3_MEAS_LOW_POWER_H_FIRST_NCLKS  0x401a

// I2C bus pins
#define PIN_I2C_SCL PA0
#define PIN_I2C_SDA PA1



/** \fn inline void wait_clk(uint8_t nclk)
 *  \brief Waits a given number of clock cycles (up to 255).
 *  \param nclk : number of clock cycles to wait.
 */
inline void wait_clk(uint8_t nclk) {
  do {
    asm("nop");
	} while (nclk--);
}

/*******************/
/*** I2C handler ***/
/*******************/

/** \fn inline void pull_scl_high()
 *  \brief Pulls SCL line high (the line is let free to go high through
 *  the appropriate pull-up resistor).
 */
inline void pull_scl_high() {
  // I2C doesn't drive lines to VCC but
  // a 1 is signaled by letting the line
  // go to VCC through a pull-up resistor
  DDRA &= ~_BV(PIN_I2C_SCL);
  // set this if there's no pull-up on that line
  // PORTA |= _BV(PIN_I2C_SCL);
}

/** \fn inline void pull_scl_low()
 *  \brief Pulls SCL line low.
 */
inline void pull_scl_low() {
  DDRA |= _BV(PIN_I2C_SCL);
	PORTA &= ~_BV(PIN_I2C_SCL);
  // set this if there's no pull-up on that line
  // PORTA &= ~_BV(PIN_I2C_SCL);
}

/** \fn inline void pull_sda_high()
 *  \brief Pulls SDA line high (the line is let free to go high through
 *  the appropriate pull-up resistor).
 */
inline void pull_sda_high() {
  DDRA &= ~_BV(PIN_I2C_SDA);
  // set this if there's no pull-up on that line
  // PORTA |= _BV(PIN_I2C_SDA);
}

/** \fn inline void pull_sda_low()
 *  \brief Pulls SDA line low.
 */
inline void pull_sda_low() {
  DDRA |= _BV(PIN_I2C_SDA);
	PORTA &= ~_BV(PIN_I2C_SDA);
  // set this if there's no pull-up on that line
  // PORTA &= ~_BV(PIN_I2C_SDA);
}

/** \fn bool i2c_write_byte(unsigned char data)
 *  \brief Writes a byte on the I2C bus.
 *  \param data: byte to write.
 *  \returns true if the receiver answered with ACK, false otherwise.
 */
bool i2c_write_byte(unsigned char data) {
  uint8_t len = 8;
  do {
    if (data & 0x80) {
      pull_sda_high();
    } else {
      pull_sda_low();
    }
    pull_scl_high();
    pull_scl_low();
    data <<= 1;
  } while (--len);
  // byte transfer ended -> read ack bit
	pull_sda_high();
  pull_scl_high();
	bool ack = (PINA & _BV(PIN_I2C_SDA));
  pull_scl_low();
	pull_sda_low();
  // ack = 0, nack = 1
  return (ack == 0);
}

/** \fn bool i2c_write_double_byte(unsigned int data)
 *  \brief Writes a 16bit value on the I2C bus.
 *  \param data: value to write.
 *  \returns true if the receiver answered with ACK, false otherwise.
 */
bool i2c_write_double_byte(unsigned int data) {
	return i2c_write_byte(data >> 8) & i2c_write_byte(data & 0xff);
}

/** \fn bool i2c_start(unsigned char address, bool read)
 *  \brief Starts a communication on the I2C bus.
 *  \param address: address of the device to communicate with.
 *  \param read: defines the type of communication; true = read, false = write.
 *  \returns true if the receiver answered with ACK, false otherwise.
 */
bool i2c_start(unsigned char address, bool read) {
  // format address
  address <<= 1;
  address += read;
  // start condition
  pull_scl_high();
	wait_clk(1);
  pull_sda_low();
  wait_clk(1);
  pull_scl_low();
  wait_clk(1);
  // send address
  return i2c_write_byte(address);
}

/** \fn void i2c_stop()
 *  \brief Stops a communication on the I2C bus.
 */
void i2c_stop() {
  pull_sda_low();
  wait_clk(1);
  pull_scl_high();
  wait_clk(1);
  pull_sda_high();
  wait_clk(1);
}

/** \fn unsigned char i2c_read_byte(bool ack)
 *  \brief Reads a byte from the I2C bus.
 *  \param ack: if true, sends ACK after reading, otherwise sends NACK.
 *  \returns the byte read.
 */
unsigned char i2c_read_byte(bool ack) {
  uint8_t len = 8;
  unsigned char result = 0;
  pull_sda_high();
  do {
    pull_scl_high();
    result <<= 1;
    result |= ((PINA & _BV(PIN_I2C_SDA)) ? 1 : 0);
    pull_scl_low();
  } while (--len);
	if (ack) {
		pull_sda_low();
	} else {
		pull_sda_high();
	}
	pull_scl_high();
	pull_scl_low();
	pull_sda_low();
	
  return result;
}


/***********************/
/*** SHTC3 functions ***/
/***********************/

/** \fn unsigned char crc8(unsigned char vh, unsigned char vl)
 *  \brief Computes the CRC of a 16bit value.
 *  The CRC type is as defined in SHTC3 datasheet p.9.
 *  \param vh: high byte.
 *  \param vl: low byte.
 *  \returns computed CRC.
 */
unsigned char crc8(unsigned char vh, unsigned char vl) {
	unsigned char crc = 0xff;
	unsigned char poly = 0x31;
	unsigned char data[2] = {vl, vh};
	unsigned char i = 2;
	while (i--) {
		crc ^= data[i];
		unsigned char j = 8;
		do {
			if (crc & 0x80)
				crc = (crc << 1) ^ poly;
			else
				crc <<= 1;
		} while (--j);
	}
	return crc;
}

/** \fn bool compute_temperature(float & value, unsigned char Th, unsigned char Tl, unsigned char crc)
 *  \brief Computes temperature from SHTC3 data, performing a CRC check beforehand.
 *  \param value: returned value.
 *  \param Th: raw data (high byte).
 *  \param Tl: raw data (low byte).
 *  \param crc: CRC returned by sensor.
 *  \returns true if conversion was successful, false otherwise.
 */
bool compute_temperature(float & value, unsigned char Th, unsigned char Tl, unsigned char crc) {
	if (!(crc8(Th, Tl) ^ crc)) {
		unsigned int T = (Th << 8) + Tl;
		value = -45.0f + 175.0f*(float)T/65536.0f;
	} else {
		return false;
	}
	return true;
}

/** \fn bool compute_humidity(float & value, unsigned char Hh, unsigned char Hl, unsigned char crc)
 *  \brief Computes humidity from SHTC3 data, performing a CRC check beforehand.
 *  \param value: returned value.
 *  \param Hh: raw data (high byte).
 *  \param Hl: raw data (low byte).
 *  \param crc: CRC returned by sensor.
 *  \returns true if conversion was successful, false otherwise.
 */
bool compute_humidity(float & value, unsigned char Hh, unsigned char Hl, unsigned char crc) {
	if (!(crc8(Hh, Hl) ^ crc)) {
		unsigned int H = (Hh << 8) + Hl;
		value = 100.0f * (float)H/65536.0f;
	} else {
		return false;
	}
	return true;
}

int main() {
  /*** setup ***/
  cli(); // disable interrupts
  // set pin modes and initial states for I2C
  DDRA |= _BV(PIN_I2C_SCL) | _BV(PIN_I2C_SDA);
	PORTA &= ~_BV(PIN_I2C_SCL) & ~_BV(PIN_I2C_SDA);
	
  // watchdog timer for periodic measurements
  ADCSRA &= ~_BV(ADEN);
  MCUSR &= ~_BV(WDRF); // clear watchdog status bit, just in case
  WDTCSR |= _BV(WDCE); // enable watchdog change and watchdog
  WDTCSR |= _BV(WDCE) | _BV(WDE) | _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // set interrupt every 8sec
  
  // timer1 settings (for delays)
  TCCR1B |= _BV(CS10) | _BV(WGM12); // clock prescaler 1/8, CTC
  TIMSK1 &= ~_BV(OCIE1A); // disable the compare match interrupt
  sei(); // enable interrupts
  
	i2c_stop(); // set I2C bus to idle state, just in case
  
	// variables for data measurement
	unsigned char Tl, Th, Tcrc, Hl, Hh, Hcrc;
	
  /*** main loop ***/
  for (;;) {
  	// wake up temperature sensor
  	bool success = i2c_start(SHTC3_ADDRESS, false);
  	if (!success) goto handle_results;
  	i2c_write_double_byte(SHTC3_WAKE_UP);
  	i2c_stop();
  	// start measurement
  	success = i2c_start(SHTC3_ADDRESS, false);
  	if (!success) goto handle_results;
		// set measurement type (2 bytes)
  	i2c_write_double_byte(SHTC3_MEAS_NORMAL_T_FIRST_CLKS);
  	i2c_stop();
		// probe measurement state
  	success = i2c_start(SHTC3_ADDRESS, true);
  	if (!success) goto handle_results;
  	pull_scl_high();
  	// wait for measurement to finish
  	do {} while (!(PINA & _BV(PIN_I2C_SCL)));
		// read temperature (high byte - low byte - CRC)
  	Th = i2c_read_byte(true);
  	Tl = i2c_read_byte(true);
  	Tcrc = i2c_read_byte(true);
		// read humidity (high byte - low byte - CRC)
  	Hh = i2c_read_byte(true);
  	Hl = i2c_read_byte(true);
  	Hcrc = i2c_read_byte(true);
  	i2c_stop();
  	// send sensor to sleep
  	i2c_start(SHTC3_ADDRESS, false);
  	i2c_write_double_byte(SHTC3_SLEEP);
  	i2c_stop();
  
    handle_results:
  	if (success) {
  		float temperature, humidity;
  		if ( compute_temperature(temperature, Th, Tl, Tcrc) & compute_humidity(humidity, Hh, Hl, Hcrc) ) {
				// do something with computed values
  		} else {
				// one or both temperature/humidity measurements failed (invalid CRC)
  		}
  	} else {
  		// do something if measurement failed (communication problem)
  	}
    
    // enable power down sleep mode and send attiny to sleep until watchdog timer triggers
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
}
