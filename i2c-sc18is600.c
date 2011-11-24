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
#include <linux/workqueue.h>
#include <asm/delay.h>	//FIXME

#define SPI_SC18IS600_NAME		"i2c-sc18is600"
#define I2C_IRQ_PIN				AT91_PIN_PA28

#define SC18IS600_REGS_NUM		0x06
#define SC18IS600_BUF_SZ		96

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
#define SC18IS600_CMD_RDREG		0x21 // Read from internal register
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
	sc18is600_spi_msg[1] = (num); \
	sc18is600_spi_msg[2] = (u8)addr<<1
	
#define fill_rdblk_msg(addr, num) \
	sc18is600_spi_msg[0] = SC18IS600_CMD_RDBLK; \
	sc18is600_spi_msg[1] = (num); \
	sc18is600_spi_msg[2] = (u8)addr<<1|0x01

#define fill_rdawr_msg(addr, numw, numr) \
	sc18is600_spi_msg[0] = SC18IS600_CMD_RDAWR; \
	sc18is600_spi_msg[1] = (numw); \
	sc18is600_spi_msg[2] = (numr); \
	sc18is600_spi_msg[3] = (u8)addr<<1; \
	sc18is600_spi_msg[numw+4] = (u8)addr<<1|0x01

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

struct sc18is600_i2c {
	spinlock_t			lock;
	wait_queue_head_t	wait;
	
	unsigned int		irq;
	struct device		*dev;
	struct spi_device	*spi_dev;
	struct i2c_adapter	adap;
	u8 					*spi_tx_buf;
	u8					*spi_rx_buf;
};

u8 *sc18is600_spi_msg, *mrx;
struct spi_device *sc18is600_spi_dev;
wait_queue_head_t wait;
u8 last_i2cstat;

static struct sc18is600_chip *sc18is600_chips;

void debug_spi_msg(u8 num)
{
	u8 i;
	u8 debug_str[256] = {0, };
	
	if(num>0)
		snprintf(debug_str,	256, "%02x", sc18is600_spi_msg[0]);
	for(i=1; i<num; i++)
		snprintf(debug_str, 256, "%s, %02x", debug_str, sc18is600_spi_msg[i]);
	dev_dbg(&sc18is600_spi_dev->dev, "sc18is600_spi_msg={%s}", debug_str);
}

static s32 sc18is600_spi_msg_write(u8 len)
{
	s32 ret;
	
	debug_spi_msg(len);
	if((ret = spi_write(sc18is600_spi_dev, sc18is600_spi_msg, len)) < 0)
		dev_dbg(&sc18is600_spi_dev->dev, "Can't write command to device\n");
	
	return ret;
}

static s32 sc18is600_read_reg(u8 reg_addr)
{
	s32 ret;
	struct spi_message	m;
	struct spi_transfer t, t1;
	
	memset(&t, 0, sizeof(t));
	memset(&t1, 0, sizeof(t1));
	
	sc18is600_spi_msg[0] = SC18IS600_CMD_RDREG;
	sc18is600_spi_msg[1] = reg_addr;
	sc18is600_spi_msg[2] = 0;
	
    t.tx_buf	= sc18is600_spi_msg;
    t.rx_buf 	= mrx;
    t.len		= 2;
    t.bits_per_word = 8;
    t.delay_usecs = 1;
    
    t1.tx_buf	= &sc18is600_spi_msg[2];
    t1.rx_buf 	= &mrx[2];
    t1.len		= 1;
    t1.bits_per_word = 8;

	spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_message_add_tail(&t1, &m);
    
    if((ret=spi_sync(sc18is600_spi_dev, &m))<0) {
		dev_err(&sc18is600_spi_dev->dev, "Can't send full-duplex spi transfer: %d\n", ret);
		return ret;
	}
	ret = (s32)mrx[2];

	return ret;
}

static s32 sc18is600_write_reg(u8 reg_addr, u8 value)
{
	s32 ret;
	
	dev_dbg(&sc18is600_spi_dev->dev, "Write to internal reg 0x%02X value 0x%02X\n", reg_addr, value);
	sc18is600_spi_msg[0] = SC18IS600_CMD_WRREG;
	sc18is600_spi_msg[1] = reg_addr;
	sc18is600_spi_msg[2] = value;
	if((ret = sc18is600_spi_msg_write(3)) < 0)
		return ret;
		
	return 0;
}

/* 
 * Destination buffer must have enough space.
 * Returns zero or a negative error code.
 */
static s32 sc18is600_read_buf(u8 *dst_buf, u8 len)
{
	s32 ret;
	struct spi_message	m;
	struct spi_transfer t, t1;
	
	memset(&t, 0, sizeof(t));
	memset(&t1, 0, sizeof(t1));
	
	sc18is600_spi_msg[0] = SC18IS600_CMD_RDBUF;
	
    t.tx_buf	= sc18is600_spi_msg;
    t.rx_buf 	= dst_buf;
    t.len		= 1;
    t.bits_per_word = 8;
    t.delay_usecs = 1;
    
    t1.tx_buf	= sc18is600_spi_msg;
    t1.rx_buf 	= dst_buf;
    t1.len		= len;
    t1.bits_per_word = 8;

	spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_message_add_tail(&t1, &m);
    
    if((ret=spi_sync(sc18is600_spi_dev, &m))<0) {
		dev_err(&sc18is600_spi_dev->dev, "Can't send full-duplex spi message: %d\n", ret);
		return ret;
	}

	return 0;
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
		ret = sc18is600_spi_msg_write(3);
		
		ret = 0;
		break;

	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			chip->pointer = command;
			fill_wrblk_msg(addr, 1);
			sc18is600_spi_msg[3] = command|0x80;
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"wrote 0x%02x.\n",
					addr, command);
			ret = sc18is600_spi_msg_write(4);
		} else {
			data->byte = chip->words[chip->pointer++] & 0xff;
			fill_rdblk_msg(addr, 1);
			ret = sc18is600_spi_msg_write(3);
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"read  0x%02x.\n",
					addr, data->byte);
		}

		ret = 0;
		break;

	case I2C_SMBUS_BYTE_DATA:
		
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, 2);
			sc18is600_spi_msg[3] = command|0x80;
			sc18is600_spi_msg[4] = data->byte;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"wrote 0x%02x at 0x%02x.\n",
					addr, data->byte, command);
			ret = sc18is600_spi_msg_write(5);
			last_i2cstat = 0x00;
			wait_event_timeout(wait, last_i2cstat == 0xF0, 5 * HZ);
			//udelay(1000); // FIXME
		} else {
			fill_rdawr_msg(addr, 1, 1);
			sc18is600_spi_msg[4] = command|0x80;
			ret = sc18is600_spi_msg_write(6);
			//udelay(1000); // FIXME
			last_i2cstat = 0x00;
			wait_event_timeout(wait, last_i2cstat == 0xF0, 5 * HZ);
			ret = sc18is600_read_buf(&data->byte, 1);
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"read  0x%02x at 0x%02x.\n",
					addr, data->byte, command);
		}
		break;

	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			chip->words[command] = data->word;
			fill_wrblk_msg(addr, 3);
			sc18is600_spi_msg[3] = command|0x80;
			sc18is600_spi_msg[4] = (u8)(data->word&0x00FF);
			sc18is600_spi_msg[5] = (u8)(data->word>>8);
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"wrote 0x%04x at 0x%02x.\n",
					addr, data->word, command);
			ret = sc18is600_spi_msg_write(6);
		} else {
			data->word = chip->words[command];
			fill_rdawr_msg(addr, 1, 2);
			sc18is600_spi_msg[4] = command|0x80;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"read  0x%04x at 0x%02x.\n",
					addr, data->word, command);
			ret = sc18is600_spi_msg_write(6);
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
				sc18is600_spi_msg[5 + i] = data->block[1 + i];
			}
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"wrote %d bytes at 0x%02x.\n",
					addr, len, command);
			ret = sc18is600_spi_msg_write(len+5);
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

void sc18is600_irq_bh_handler(struct work_struct *work)
{
	int ret;
	
	if((ret = sc18is600_read_reg(SC18IS600_I2CSTAT)) < 0)
		dev_err(&sc18is600_spi_dev->dev, "Can't read I2CStat internal register!\n");
	dev_dbg(&sc18is600_spi_dev->dev, "I2CStat: 0x%02X!\n", (u8)ret);
	last_i2cstat = (u8)ret;
	wake_up(&wait);
}

DECLARE_WORK(i2c_irq_bh, &sc18is600_irq_bh_handler);

static irqreturn_t sc18is600_i2c_irq(int irqno, void *dev_id)
{
	if(gpio_get_value(I2C_IRQ_PIN))
		return IRQ_HANDLED; // Faked IRQ
	else
		schedule_work(&i2c_irq_bh);
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

static ssize_t sc18is600_show_buf(struct device *dev,
				struct device_attribute *attr,
                char *buf)
{
	int ret;
	int i;
	u8 debug_str[256] = {0, };
	
	ret = sc18is600_read_buf(mrx, 96);
	
	debug_str[0] = 0;
	for(i = 0; i < SC18IS600_BUF_SZ; i++) {
		snprintf(debug_str,	256, "%s%02x ", debug_str, mrx[i]);
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", debug_str);
}

static ssize_t sc18is600_show_regs(struct device *dev,
				struct device_attribute *attr,
                char *buf)
{
	s32 ret;
	u8 i;
	char out_str[3+2*SC18IS600_REGS_NUM];
	
	snprintf(out_str, 256, "0x");
	
	for(i = 0; i < SC18IS600_REGS_NUM; i++) {
		if((ret = sc18is600_read_reg(i)) < 0)
			return (ssize_t)ret;
		snprintf(out_str, 256, "%s%02X", out_str, (u8)ret);
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", out_str);
}

static ssize_t sc18is600_store_regs(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	s32 ret;
	unsigned long tmp;
		
	if (strict_strtoul(buf, 16, &tmp) < 0)
		return -EINVAL;
	if(tmp&0xFFFF8000UL)
		return -EINVAL;
	
	if((ret = sc18is600_write_reg((u8)tmp>>8, (u8)tmp&0xFF)) < 0)
			return (ssize_t)ret;

	return count;
}

static DEVICE_ATTR(buf, S_IRUSR, sc18is600_show_buf, NULL);
static DEVICE_ATTR(regs, S_IRUSR|S_IWUSR, sc18is600_show_regs, sc18is600_store_regs);

static struct attribute *sc18is600_attributes[] = {
	&dev_attr_buf.attr,
	&dev_attr_regs.attr,
	NULL,
};

static const struct attribute_group sc18is600_attr_group = {
	.attrs = sc18is600_attributes,
};

static int __devinit sc18is600_probe(struct spi_device *spi) {
	int ret;
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = 500000;
	ret = spi_setup(spi);
	if(ret<0)
		return ret;
	sc18is600_adapter.dev.parent = &spi->dev;
	sc18is600_spi_dev = spi;
	init_waitqueue_head(&wait);
	return sysfs_create_group(&spi->dev.kobj, &sc18is600_attr_group);
}

static int __devexit sc18is600_remove(struct spi_device *spi) {
	sysfs_remove_group(&spi->dev.kobj, &sc18is600_attr_group);
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
		dev_dbg(&sc18is600_spi_dev->dev, "Could not set pin %i for GPIO input.\n", I2C_IRQ_PIN);
	}
	if(at91_set_deglitch(I2C_IRQ_PIN, 1)) {
		dev_dbg(&sc18is600_spi_dev->dev, "Could not set pin %i for GPIO deglitch.\n", I2C_IRQ_PIN);
	}

	/** Request IRQ for pin */
	// |IRQF_TRIGGER_RISING
	if(request_irq(I2C_IRQ_PIN, sc18is600_i2c_irq, IRQF_TRIGGER_NONE, "i2c-sc18is600", NULL))  {
		dev_dbg(&sc18is600_spi_dev->dev, "Can't register IRQ %d\n", I2C_IRQ_PIN);
		return -EIO;
	}

	mrx=kzalloc(256 * sizeof(u8), GFP_KERNEL);
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
	free_irq(I2C_IRQ_PIN,0);
    kfree(mrx);
	kfree(sc18is600_spi_msg);
	kfree(sc18is600_chips);
}

MODULE_AUTHOR("Lapin Roman <lampus.lapin@gmail.com>");
MODULE_DESCRIPTION("I2C driver for SC18IS600 SPI to I2C bridge");
MODULE_LICENSE("GPL");

module_init(i2c_sc18is600_init);
module_exit(i2c_sc18is600_exit);
