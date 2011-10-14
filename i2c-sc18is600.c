/*
    i2c-sc18is600.c - I2C driver for NXP SC18IS600 SPI to I2C

    Copyright (C) 2011 Lapin Roman <lampus.lapin@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#define DEBUG 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define SPI_SC18IS600_NAME		"i2c-sc18is600"
#define I2C_IRQ_PIN				AT91_PIN_PA28

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

#define fill_wrblk_msg(addr, num) \
	sc18is600_spi_msg[0] = SC18IS600_CMD_WRBLK; \
	sc18is600_spi_msg[1] = num; \
	sc18is600_spi_msg[2] = (u8)addr<<1
	
#define fill_rdblk_msg(addr, num) \
	sc18is600_spi_msg[0] = SC18IS600_CMD_RDBLK; \
	sc18is600_spi_msg[1] = num; \
	sc18is600_spi_msg[2] = (u8)addr<<1|0x01

#define fill_rdawr_msg(addr, numw, numr) \
	sc18is600_spi_msg[0] = SC18IS600_CMD_RDAWR; \
	sc18is600_spi_msg[1] = numw; \
	sc18is600_spi_msg[2] = numr; \
	sc18is600_spi_msg[3] = (u8)addr<<1; \
	sc18is600_spi_msg[numw+4] = (u8)addr<<1|0x01;

#define MAX_CHIPS 10
#define SC18IS600_FUNC (I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE | \
		   I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA | \
		   I2C_FUNC_SMBUS_BLOCK_DATA)

static unsigned short chip_addr[MAX_CHIPS];
module_param_array(chip_addr, ushort, NULL, S_IRUGO);
MODULE_PARM_DESC(chip_addr,
		 "Chip addresses (up to 10, between 0x03 and 0x77)");

static unsigned long functionality = SC18IS600_FUNC;
module_param(functionality, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(functionality, "Override functionality bitfield");

struct sc18is600_chip {
	u8 pointer;
	u16 words[256];		/* Byte operations use the LSB as per SMBus
				   specification */
};

u8 *sc18is600_spi_msg;
struct spi_device *sc18is600_spi_dev;

static struct sc18is600_chip *sc18is600_chips;

void debug_spi_msg(u8 num)
{
	u8 i;
	u8 debug_str[256] = {0, };
	
	if(num>0)
		snprintf(debug_str,	256, "%02x", sc18is600_spi_msg[0]);
	for(i=1; i<num; i++)
		snprintf(debug_str, 256, "%s, %02x", debug_str, sc18is600_spi_msg[i]);
	printk(KERN_DEBUG"sc18is600_spi_msg={%s}", debug_str);
}

/* Return negative errno on error. */
static s32 sc18is600_xfer(struct i2c_adapter * adap, u16 addr, unsigned short flags,
	char read_write, u8 command, int size, union i2c_smbus_data * data)
{
	s32 ret;
	int i, len;
	struct sc18is600_chip *chip = NULL;

	/* Search for the right chip */
	for (i = 0; i < MAX_CHIPS && chip_addr[i]; i++) {
		if (addr == chip_addr[i]) {
			chip = sc18is600_chips + i;
			break;
		}
	}
	if (!chip)
		return -ENODEV;

	switch (size) {

	case I2C_SMBUS_QUICK:
		fill_wrblk_msg(addr, 0);
		if (read_write == I2C_SMBUS_READ)
			sc18is600_spi_msg[2] |= 0x01;
		dev_dbg(&adap->dev, "smbus quick - addr 0x%02x\n", addr);
		debug_spi_msg(3);
		ret=spi_write(sc18is600_spi_dev, sc18is600_spi_msg, 3);
		
		ret = 0;
		break;

	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			chip->pointer = command;
			fill_wrblk_msg(addr, 1);
			sc18is600_spi_msg[3] = command;
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"wrote 0x%02x.\n",
					addr, command);
			debug_spi_msg(4);
		} else {
			data->byte = chip->words[chip->pointer++] & 0xff;
			fill_rdblk_msg(addr, 1);
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"read  0x%02x.\n",
					addr, data->byte);
			debug_spi_msg(3);
		}

		ret = 0;
		break;

	case I2C_SMBUS_BYTE_DATA:
		
		if (read_write == I2C_SMBUS_WRITE) {
			chip->words[command] &= 0xff00;
			chip->words[command] |= data->byte;
			fill_wrblk_msg(addr, 2);
			sc18is600_spi_msg[3] = command;
			sc18is600_spi_msg[4] = data->byte;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"wrote 0x%02x at 0x%02x.\n",
					addr, data->byte, command);
			debug_spi_msg(5);
		} else {
			data->byte = chip->words[command] & 0xff;
			fill_rdawr_msg(addr, 1, 1);
			sc18is600_spi_msg[4] = command;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"read  0x%02x at 0x%02x.\n",
					addr, data->byte, command);
			debug_spi_msg(6);
		}
		chip->pointer = command + 1;

		ret = 0;
		break;

	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			chip->words[command] = data->word;
			fill_wrblk_msg(addr, 3);
			sc18is600_spi_msg[3] = command;
			sc18is600_spi_msg[4] = (u8)(data->word&0x00FF);
			sc18is600_spi_msg[5] = (u8)(data->word>>8);
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"wrote 0x%04x at 0x%02x.\n",
					addr, data->word, command);
			debug_spi_msg(6);
		} else {
			data->word = chip->words[command];
			fill_rdawr_msg(addr, 1, 2);
			sc18is600_spi_msg[4] = command;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"read  0x%04x at 0x%02x.\n",
					addr, data->word, command);
			debug_spi_msg(6);
		}

		ret = 0;
		break;

	case I2C_SMBUS_BLOCK_DATA:
		len = data->block[0];
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, len+1);
			sc18is600_spi_msg[3] = command;
			sc18is600_spi_msg[4] = len;
			for (i = 0; i < len; i++) {
				chip->words[command + i] &= 0xff00;
				chip->words[command + i] |= data->block[1 + i];
				sc18is600_spi_msg[5 + i] = data->block[1 + i];
			}
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"wrote %d bytes at 0x%02x.\n",
					addr, len, command);
			debug_spi_msg(len+5);
		} else {
			for (i = 0; i < len; i++) {
				data->block[1 + i] =
					chip->words[command + i] & 0xff;
			}
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"read  %d bytes at 0x%02x.\n",
					addr, len, command);
		}

		ret = 0;
		break;

	default:
		dev_dbg(&adap->dev, "Unsupported I2C/SMBus command\n");
		ret = -EOPNOTSUPP;
		break;
	} /* switch (size) */

	return ret;
}


void sc18is600_irq_process(unsigned long arg) {
    printk(KERN_INFO "I2C_IRQ!!!\n");
    sc18is600_spi_msg[0] = SC18IS600_CMD_RDBUF;
    //spi_write_then_read(sc18is600_spi_dev, sc18is600_spi_msg, 1, u8 * rxbuf, unsigned n_rx);
}

DECLARE_TASKLET(sc18is600_irq_tasklet,sc18is600_irq_process,0);

static irqreturn_t sc18is600_i2c_irq(int irqno, void *dev_id)
{
	tasklet_hi_schedule(&sc18is600_irq_tasklet);
	return IRQ_HANDLED;
}

static u32 sc18is600_func(struct i2c_adapter *adapter)
{
	return SC18IS600_FUNC & functionality;
}

static const struct i2c_algorithm smbus_algorithm = {
	.functionality	= sc18is600_func,
	.smbus_xfer	= sc18is600_xfer,
};

static struct i2c_adapter sc18is600_adapter = {
	.owner		= THIS_MODULE,
	.class		= I2C_CLASS_HWMON | I2C_CLASS_SPD,
	.algo		= &smbus_algorithm,
	.name		= "sc18is600-i2c",
};

static int __devinit sc18is600_probe(struct spi_device *spi) {
	int ret;
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 1200000;
	ret = spi_setup(spi);
	if(ret<0)
		return ret;
	sc18is600_adapter.dev.parent = &spi->dev;
	sc18is600_spi_dev = spi;
	return 0;
	//return sysfs_create_group(&spi->dev.kobj, &sc18is600_attr_group);
}

static int __devexit sc18is600_remove(struct spi_device *spi) {
	//sysfs_remove_group(&spi->dev.kobj, &sc18is600_attr_group);
	return 0;
}

static struct spi_driver sc18is600_driver = {
	.driver = {
		.name	= SPI_SC18IS600_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= sc18is600_probe,
	.remove	= __devexit_p(sc18is600_remove),
};

static int __init i2c_sc18is600_init(void)
{
	int i, ret;

	if (!chip_addr[0]) {
		printk(KERN_ERR "i2c-stub: Please specify a chip address\n");
		return -ENODEV;
	}

	for (i = 0; i < MAX_CHIPS && chip_addr[i]; i++) {
		if (chip_addr[i] < 0x03 || chip_addr[i] > 0x77) {
			printk(KERN_ERR "i2c-stub: Invalid chip address "
			       "0x%02x\n", chip_addr[i]);
			return -EINVAL;
		}

		printk(KERN_INFO "i2c-stub: Virtual chip at 0x%02x\n",
		       chip_addr[i]);
	}

	/* Allocate memory for all chips at once */
	sc18is600_chips = kzalloc(i * sizeof(struct sc18is600_chip), GFP_KERNEL);
	if (!sc18is600_chips) {
		printk(KERN_ERR "i2c-stub: Out of memory\n");
		return -ENOMEM;
	}
	
	/* TODO: Move all at91_* to the board file!!! */
	at91_set_GPIO_periph(I2C_IRQ_PIN,0);
	if(at91_set_gpio_input(I2C_IRQ_PIN, 0)) {
		printk(KERN_DEBUG"Could not set pin %i for GPIO input.\n", I2C_IRQ_PIN);
	}
	if(at91_set_deglitch(I2C_IRQ_PIN, 1)) {
		printk(KERN_DEBUG"Could not set pin %i for GPIO deglitch.\n", I2C_IRQ_PIN);
	}

	/** Request IRQ for pin */
	// |IRQF_TRIGGER_RISING
	if(request_irq(I2C_IRQ_PIN, sc18is600_i2c_irq, IRQF_TRIGGER_NONE, "i2c-sc18is600", NULL))  {
		printk(KERN_DEBUG"Can't register IRQ %d\n", I2C_IRQ_PIN);
		return -EIO;
	}
	
	sc18is600_spi_msg = kzalloc(256 * sizeof(u8), GFP_KERNEL);
	ret = spi_register_driver(&sc18is600_driver);
	if(ret) {
		kfree(sc18is600_chips);
		return ret;
	}

	ret = i2c_add_adapter(&sc18is600_adapter);
	if (ret)
		kfree(sc18is600_chips);
	return ret;
}

static void __exit i2c_sc18is600_exit(void)
{
	i2c_del_adapter(&sc18is600_adapter);
	spi_unregister_driver(&sc18is600_driver);
	tasklet_kill(&sc18is600_irq_tasklet);
	free_irq(I2C_IRQ_PIN,0);
	kfree(sc18is600_spi_msg);
	kfree(sc18is600_chips);
}

MODULE_AUTHOR("Lapin Roman <lampus.lapin@gmail.com>");
MODULE_DESCRIPTION("I2C driver for SC18IS600 SPI to I2C bridge");
MODULE_LICENSE("GPL");

module_init(i2c_sc18is600_init);
module_exit(i2c_sc18is600_exit);
