# Sensirion SHTC3 / attiny84 example

This is a short example for attiny84 showing how to measure temperature and relative humidity with the Sensirion SHTC3. It communicates on the I2C bus by bit banging. Control and conversion functions are provided.

The main loop wakes up the attiny84 every 8 seconds, which in turn wakes up the SHTC3, takes a measurement, and then sends both back to sleep.

## Pre-requisites

* AVR GCC toolchain
* avrdude
* adequate programmer (Makefile is for USBasp)

## Install

> make & make flash

## Notes
* It may fit (not tested) in an attiny44 by using the MCU's I2C driver (see *Universal Serial Interface* in attiny datasheet)
* The *setup* and *main loop* sections of *main()* function may be split to match Arduino's nomenclature
