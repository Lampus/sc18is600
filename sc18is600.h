/*
 * Register definitions for SPI to I2C-bus interface NXP SC18IS600
 *
 * Copyright (C) 2011 Thesys-Intechna
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __SC18IS600_H__
#define __SC18IS600_H__

/*  Register offsets */
#define SC18IS600_IOCONFIG		0x00
#define SC18IS600_IOSTATE		0x01
#define SC18IS600_I2CCLOCK		0x02
#define SC18IS600_I2CTO			0x03
#define SC18IS600_I2CSTAT		0x04
#define SC18IS600_I2CADR		0x05

/* Bitfields in IOConfig */
#define SC18IS600_IO0_OFFSET	0
#define SC18IS600_IO0_SIZE		2
#define SC18IS600_IO1_OFFSET	2
#define SC18IS600_IO1_SIZE		2
#define SC18IS600_IO2_OFFSET	4
#define SC18IS600_IO2_SIZE		2
#define SC18IS600_IO3_OFFSET	6
#define SC18IS600_IO3_SIZE		2

/* Bitfields in IOState */
#define SC18IS600_GPIO_OFFSET	0
#define SC18IS600_GPIO_SIZE		6

/* Bitfields in I2CTO */
#define SC18IS600_TE_OFFSET		0
#define SC18IS600_TE_SIZE		1
#define SC18IS600_TO_OFFSET		1
#define SC18IS600_TO_SIZE		7

/* Bitfields in I2CAdr */
#define SC18IS600_ADR_OFFSET	1
#define SC18IS600_ADR_SIZE		7

/* Pin configuration */
#define SC18IS600_PIN_BIDIR		0x00
#define SC18IS600_PIN_INPUT		0x01
#define SC18IS600_PIN_PUSHPULL	0x02
#define SC18IS600_PIN_OPENDRAIN	0x03

/* I2C Status */
#define SC18IS600_STAT_OK		0xF0 // Transmission successfull
#define SC18IS600_STAT_ANACK	0xF1 // Device address not acknowledged
#define SC18IS600_STAT_DNACK	0xF2
#define SC18IS600_STAT_BUSY		0xF3 // I2C-bus busy
#define SC18IS600_STAT_TO		0xF4 // I2C-bus time-out
#define SC18IS600_STAT_INVCNT	0xF5 // I2C-bus invalid data count

/* Commands */
#define SC18IS600_CMD_WRBLK		0x00 // Write N bytes to I2C-bus slave device
#define SC18IS600_CMD_RDBLK		0x01 // Read N bytes from I2C-bus slave device
#define SC18IS600_CMD_RDAWR		0x02 // I2C-bus read after write
#define SC18IS600_CMD_RDBUF		0x06 // Read buffer
#define SC18IS600_CMD_WRAWR		0x03 // I2C-bus write after write
#define SC18IS600_CMD_SPICON	0x18 // SPI configuration
#define SC18IS600_CMD_WRREG		0x20 // Write to internal register
#define SC18IS600_CMD_RDREG		0x20 // Read from internal register
#define SC18IS600_CMD_PWRDWN	0x30 // Power-down mode

/* Bit manipulation macros */
#define SC18IS600_BIT(name) \
	(1 << SC18IS600_##name##_OFFSET)
#define SC18IS600_BF(name,value) \
	(((value) & ((1 << SC18IS600_##name##_SIZE) - 1)) << SC18IS600_##name##_OFFSET)
#define SC18IS600_BFEXT(name,value) \
	(((value) >> SC18IS600_##name##_OFFSET) & ((1 << SC18IS600_##name##_SIZE) - 1))
#define SC18IS600_BFINS(name,value,old) \
	( ((old) & ~(((1 << SC18IS600_##name##_SIZE) - 1) << SC18IS600_##name##_OFFSET)) \
	  | SC18IS600_BF(name,value))

#endif /* __SC18IS600_H__ */
