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

#define SPI_SC18IS600_NAME		"i2c-sc18is600"
#define I2C_IRQ_PIN				AT91_PIN_PA28
#define CMD_RESP_TIME			800

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
#define SC18IS600_STAT_DNACK	0xF2 // Device data not acknowledged
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
	i2c->spi_tx_buf[0] = SC18IS600_CMD_WRBLK; \
	i2c->spi_tx_buf[1] = (num); \
	i2c->spi_tx_buf[2] = (u8)addr<<1
	
#define fill_rdblk_msg(addr, num) \
	i2c->spi_tx_buf[0] = SC18IS600_CMD_RDBLK; \
	i2c->spi_tx_buf[1] = (num); \
	i2c->spi_tx_buf[2] = (u8)addr<<1|0x01

#define fill_rdawr_msg(addr, numw, numr) \
	i2c->spi_tx_buf[0] = SC18IS600_CMD_RDAWR; \
	i2c->spi_tx_buf[1] = (numw); \
	i2c->spi_tx_buf[2] = (numr); \
	i2c->spi_tx_buf[3] = (u8)addr<<1; \
	i2c->spi_tx_buf[numw+4] = (u8)addr<<1|0x01

#define MAX_CHIPS 10
#define SC18IS600_FUNC (I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE | \
		   I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA | \
		   I2C_FUNC_SMBUS_BLOCK_DATA)


static unsigned long functionality = SC18IS600_FUNC;

struct sc18is600_i2c {
	spinlock_t			lock;
	struct completion	 done;
	
	unsigned int		irq;
	struct spi_device	*spi_dev;
	struct device		*dev;
	struct i2c_adapter	adap;
	u8 					*spi_tx_buf;
	u8					*spi_rx_buf;
};

void debug_spi_msg(struct sc18is600_i2c *i2c, u8 num)
{
	u8 i;
	u8 debug_str[256] = {0, };
	
	if(num>0)
		snprintf(debug_str,	256, "%02x", i2c->spi_tx_buf[0]);
	for(i=1; i < num; i++)
		snprintf(debug_str, 256, "%s, %02x", debug_str, i2c->spi_tx_buf[i]);
	dev_dbg(i2c->dev, "spi_tx_buf={%s}", debug_str);
}

static s32 sc18is600_read_reg(struct sc18is600_i2c *i2c, u8 reg_addr)
{
	s32 ret;
	struct spi_message	m;
	struct spi_transfer t, t1;
	
	memset(&t, 0, sizeof(t));
	memset(&t1, 0, sizeof(t1));
	
	i2c->spi_tx_buf[0] = SC18IS600_CMD_RDREG;
	i2c->spi_tx_buf[1] = reg_addr;
	i2c->spi_tx_buf[2] = 0;
	
    t.tx_buf	= i2c->spi_tx_buf;
    t.rx_buf 	= i2c->spi_rx_buf;
    t.len		= 2;
    t.bits_per_word = 8;
    t.delay_usecs = 1;
    
    t1.tx_buf	= &i2c->spi_tx_buf[2];
    t1.rx_buf 	= &i2c->spi_rx_buf[2];
    t1.len		= 1;
    t1.bits_per_word = 8;

	spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_message_add_tail(&t1, &m);
    
    if((ret=spi_sync(i2c->spi_dev, &m))<0) {
		dev_err(i2c->dev, "Can't send full-duplex spi transfer: %d\n", ret);
		return ret;
	}
	ret = (s32)i2c->spi_rx_buf[2];

	return ret;
}

static s32 sc18is600_check_status(struct sc18is600_i2c *i2c, u8 stat)
{
	s32 ret;
	
	switch(stat)
	{
		case SC18IS600_STAT_OK:
			ret = 0;
			break;
		case SC18IS600_STAT_ANACK:
			dev_dbg(i2c->dev, "Device address not acknowledged\n");
			ret = -ECONNREFUSED;
			break;
		case SC18IS600_STAT_DNACK:
			dev_dbg(i2c->dev, "Device data not acknowledged\n");
			ret = -ENXIO;
			break;
		case SC18IS600_STAT_BUSY:
			dev_dbg(i2c->dev, "I2C-bus busy\n");
			ret = -EBUSY;
			break;
		case SC18IS600_STAT_TO:
			dev_dbg(i2c->dev, "I2C-bus time-out\n");
			ret = -ETIMEDOUT;
			break;
		case SC18IS600_STAT_INVCNT:
			dev_dbg(i2c->dev, "I2C-bus invalid data count\n");
			ret = -EINVAL;
			break;
		default:
			dev_err(i2c->dev, "Unknow I2CStat value. Something wrong!\n");
			ret = -EINVAL;
			//BUG();
	}
	return ret;
}

static s32 sc18is600_spi_msg_write(struct sc18is600_i2c *i2c, u8 len)
{
	s32 ret, ret_reg_rd;
	u8 cmd;
	
	debug_spi_msg(i2c, len);
	cmd = i2c->spi_tx_buf[0];
	if((ret = spi_write(i2c->spi_dev, i2c->spi_tx_buf, len)) < 0) {
		dev_dbg(i2c->dev, "Can't write command to device\n");
		return ret;
	}
		
	switch(cmd) {
		// For some commands we should wait for irq
		case SC18IS600_CMD_WRBLK:
		case SC18IS600_CMD_RDBLK:
		case SC18IS600_CMD_RDAWR:
		case SC18IS600_CMD_WRAWR:
		wait_for_completion_timeout(&i2c->done, msecs_to_jiffies(CMD_RESP_TIME));
		if((ret_reg_rd = sc18is600_read_reg(i2c, SC18IS600_I2CSTAT)) < 0)
			dev_err(i2c->dev, "Can't read I2CStat internal register!\n");
		dev_dbg(i2c->dev, "I2CStat: 0x%02X!\n", (u8)ret_reg_rd);
		ret = sc18is600_check_status(i2c, (u8)ret_reg_rd);
		break;
		default:
		break;
	}
	
	return ret;
}

static s32 sc18is600_write_reg(struct sc18is600_i2c *i2c, u8 reg_addr, u8 value)
{
	s32 ret;
	
	dev_dbg(i2c->dev, "Write to internal reg 0x%02X value 0x%02X\n", reg_addr, value);
	i2c->spi_tx_buf[0] = SC18IS600_CMD_WRREG;
	i2c->spi_tx_buf[1] = reg_addr;
	i2c->spi_tx_buf[2] = value;
	if((ret = sc18is600_spi_msg_write(i2c, 3)) < 0)
		return ret;
		
	return 0;
}

/* 
 * Destination buffer must have enough space.
 * Returns zero or a negative error code.
 */
static s32 sc18is600_read_buf(struct sc18is600_i2c *i2c, u8 *dst_buf, u8 len)
{
	s32 ret;
	struct spi_message	m;
	struct spi_transfer t, t1, t2;

	spi_message_init(&m);	
		
	i2c->spi_tx_buf[0] = SC18IS600_CMD_RDBUF;
	
	memset(&t, 0, sizeof(t));
    t.tx_buf	= i2c->spi_tx_buf;
    t.rx_buf 	= dst_buf;
    t.len		= 1;
    t.bits_per_word = 8;
    t.delay_usecs = 2;
    spi_message_add_tail(&t, &m);
    
    memset(&t1, 0, sizeof(t1));
    t1.tx_buf	= i2c->spi_tx_buf;
    t1.rx_buf 	= dst_buf;
    t1.len		= 1;
    t1.bits_per_word = 8;
    t1.delay_usecs = 2;
    spi_message_add_tail(&t1, &m);
    
    if(len > 1) {
		memset(&t2, 0, sizeof(t2));
		t2.tx_buf	= i2c->spi_tx_buf;
		t2.rx_buf 	= &dst_buf[1];
		t2.len		= len - 1;
		t2.bits_per_word = 8;
		spi_message_add_tail(&t2, &m);
	}
    
    if((ret=spi_sync(i2c->spi_dev, &m))<0) {
		dev_err(i2c->dev, "Can't send full-duplex spi message: %d\n", ret);
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
	u16 myword;
	struct sc18is600_i2c *i2c;
	
	i2c = dev_get_drvdata(adap->dev.parent);

	switch (size) {

	case I2C_SMBUS_QUICK:
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, 0);
		}
		else {
			fill_rdblk_msg(addr, 0);
		}
		dev_dbg(&adap->dev, "smbus quick - addr 0x%02x\n", addr);
		ret = sc18is600_spi_msg_write(i2c, 3);
		break;

	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, 1);
			i2c->spi_tx_buf[3] = command;
			ret = sc18is600_spi_msg_write(i2c, 4);
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"wrote 0x%02x.\n",
					addr, command);
		} else {
			fill_rdblk_msg(addr, 1);
			ret = sc18is600_spi_msg_write(i2c, 3);
			if(ret < 0)
				return ret;
			ret = sc18is600_read_buf(i2c, &data->byte, 1);
			dev_dbg(&adap->dev, "smbus byte - addr 0x%02x, "
					"read  0x%02x.\n",
					addr, data->byte);
		}
		break;

	case I2C_SMBUS_BYTE_DATA:
		
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, 2);
			i2c->spi_tx_buf[3] = command;
			i2c->spi_tx_buf[4] = data->byte;
			ret = sc18is600_spi_msg_write(i2c, 5);
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"wrote 0x%02x at 0x%02x.\n",
					addr, data->byte, command);
		} else {
			fill_rdawr_msg(addr, 1, 1);
			i2c->spi_tx_buf[4] = command;
			ret = sc18is600_spi_msg_write(i2c, 6);
			if(ret < 0)
				return ret;
			ret = sc18is600_read_buf(i2c, &data->byte, 1);
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"read  0x%02x at 0x%02x.\n",
					addr, data->byte, command);
		}
		break;

	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, 3);
			i2c->spi_tx_buf[3] = command;
			i2c->spi_tx_buf[4] = (u8)(data->word&0x00FF);
			i2c->spi_tx_buf[5] = (u8)(data->word>>8);
			ret = sc18is600_spi_msg_write(i2c, 6);
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"wrote 0x%04x at 0x%02x.\n",
					addr, data->word, command);
		} else {
			fill_rdawr_msg(addr, 1, 2);
			i2c->spi_tx_buf[4] = command;
			ret = sc18is600_spi_msg_write(i2c, 6);
			if(ret < 0)
				return ret;
			ret = sc18is600_read_buf(i2c, (u8 *)&myword, 2);
			data->word = myword;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"read  0x%04x at 0x%02x.\n",
					addr, data->word, command);
		}
		break;

	case I2C_SMBUS_BLOCK_DATA:
		len = data->block[0];
		if (read_write == I2C_SMBUS_WRITE) {
			fill_wrblk_msg(addr, len + 2);
			i2c->spi_tx_buf[3] = command;
			i2c->spi_tx_buf[4] = len;
			for (i = 0; i < len; i++) {
				i2c->spi_tx_buf[5 + i] = data->block[1 + i];
			}
			ret = sc18is600_spi_msg_write(i2c, len + 5);
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"wrote %d bytes at 0x%02x.\n",
					addr, len, command);
		} else {
			fill_rdawr_msg(addr, 1, 32);
			i2c->spi_tx_buf[4] = command;
			ret = sc18is600_spi_msg_write(i2c, 6);
			if(ret < 0)
				return ret;
			ret = sc18is600_read_buf(i2c, &data->block[0], 1);
			if(ret < 0)
				return ret;
			len = data->block[0];
			ret = sc18is600_read_buf(i2c, &data->block[0], len + 1);
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"read  %d bytes at 0x%02x.\n",
					addr, len, command);
		}
		break;

	default:
		dev_dbg(&adap->dev, "Unsupported I2C/SMBus command\n");
		ret = -EOPNOTSUPP;
		break;
	} /* switch (size) */

	return ret;
}

static irqreturn_t sc18is600_i2c_irq(int irqno, void *dev_id)
{
	struct sc18is600_i2c *i2c = dev_id;

	if(gpio_get_value(I2C_IRQ_PIN))
		return IRQ_HANDLED; // Faked IRQ
	else
		complete(&i2c->done);
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

static ssize_t sc18is600_show_buf(struct device *dev,
				struct device_attribute *attr,
                char *buf)
{
	struct sc18is600_i2c *i2c;
	int ret;
	int i;
	u8 debug_str[256] = {0, };
	
	i2c = dev_get_drvdata(dev);
	
	ret = sc18is600_read_buf(i2c, i2c->spi_rx_buf, 96);
	
	debug_str[0] = 0;
	for(i = 0; i < SC18IS600_BUF_SZ; i++) {
		snprintf(debug_str,	256, "%s%02x ", debug_str, i2c->spi_rx_buf[i]);
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", debug_str);
}

static ssize_t sc18is600_show_regs(struct device *dev,
				struct device_attribute *attr,
                char *buf)
{
	struct sc18is600_i2c *i2c;
	s32 ret;
	u8 i;
	char out_str[3+2*SC18IS600_REGS_NUM];
	
	i2c = dev_get_drvdata(dev);
	printk(KERN_INFO "I2C Dev: %08X\n", i2c->irq);
	snprintf(out_str, 256, "0x");
	
	for(i = 0; i < SC18IS600_REGS_NUM; i++) {
		if((ret = sc18is600_read_reg(i2c, i)) < 0)
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
	struct sc18is600_i2c *i2c;
	
	i2c = dev_get_drvdata(dev);
		
	if (strict_strtoul(buf, 16, &tmp) < 0)
		return -EINVAL;
	if(tmp&0xFFFF8000UL)
		return -EINVAL;
	
	if((ret = sc18is600_write_reg(i2c, (u8)tmp>>8, (u8)tmp&0xFF)) < 0)
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

static int sc18is600_setup_irq(struct sc18is600_i2c *i2c, unsigned gpio_pin)
{
	/* TODO: Move all at91_* to the board file!!! */
	at91_set_GPIO_periph(gpio_pin,0);
	if(at91_set_gpio_input(gpio_pin, 0) < 0) {
		dev_dbg(i2c->dev, "Could not set pin %i for GPIO input.\n", gpio_pin);
		return -EIO;
	}
	
	if(at91_set_deglitch(gpio_pin, 1) < 0) {
		dev_dbg(i2c->dev, "Could not set pin %i for GPIO deglitch.\n", gpio_pin);
		return -EIO;
	}

	/** Request IRQ for pin */
	if(request_irq(gpio_pin, sc18is600_i2c_irq, IRQF_TRIGGER_NONE, "i2c-sc18is600", i2c) < 0)  {
		dev_dbg(i2c->dev, "Can't register IRQ %d\n", gpio_pin);
		return -EIO;
	}
	
	return 0;
}

static int __devinit sc18is600_probe(struct spi_device *spi) {
	struct sc18is600_i2c *i2c;
	int ret;
	
	i2c = kzalloc(sizeof(struct sc18is600_i2c), GFP_KERNEL);
	if (!i2c) {
		printk(KERN_ERR "no memory for state\n");
		return -ENOMEM;
	}
	
	i2c->spi_dev = spi;
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = 500000;
	
	ret = spi_setup(spi);
	if(ret < 0)
		goto err_spi_setup;
		
	ret = sc18is600_setup_irq(i2c, I2C_IRQ_PIN);
	if(ret < 0)
		goto err_irq_setup;
	
	i2c->adap.owner		= THIS_MODULE;
	i2c->adap.class		= I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->adap.algo		= &smbus_algorithm;
	strlcpy(i2c->adap.name, "sc18is600-i2c", sizeof(i2c->adap.name));
	i2c->adap.dev.parent = &spi->dev;
	
	i2c->spi_dev = spi;
	i2c->dev = &spi->dev;
	
	init_completion(&i2c->done);
	
	i2c->spi_tx_buf = kzalloc(256 * sizeof(u8), GFP_KERNEL);
	if (!i2c->spi_tx_buf) {
		printk(KERN_ERR "no memory for spi_tx_buf\n");
		ret = -ENOMEM;
		goto err_irq_setup;
	}
	i2c->spi_rx_buf = kzalloc(256 * sizeof(u8), GFP_KERNEL);
	if (!i2c->spi_rx_buf) {
		printk(KERN_ERR "no memory for spi_rx_buf\n");
		ret = -ENOMEM;
		goto err_spi_rx_buf;
	}

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(i2c->dev, "Can't add i2c adapter\n");
		goto err_i2c_add_adap;
	}

	spi_set_drvdata(spi, i2c);
	
	return sysfs_create_group(&spi->dev.kobj, &sc18is600_attr_group);
	
err_i2c_add_adap:
	kfree(i2c->spi_rx_buf);
err_spi_rx_buf:
	kfree(i2c->spi_tx_buf);
err_irq_setup:
	free_irq(I2C_IRQ_PIN, i2c);
err_spi_setup:
	kfree(i2c);
	return ret;
}

static int __devexit sc18is600_remove(struct spi_device *spi) {
	struct sc18is600_i2c *i2c;
	
	i2c = dev_get_drvdata(&spi->dev);
	
	free_irq(I2C_IRQ_PIN, i2c);
    kfree(i2c->spi_rx_buf);
	kfree(i2c->spi_tx_buf);
	i2c_del_adapter(&i2c->adap);
	kfree(i2c);
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
	int ret;

	ret = spi_register_driver(&sc18is600_driver);
	if(ret < 0) {
		printk(KERN_ERR "Can't register spi driver\n");
		return ret;
	}

	return ret;
}

static void __exit i2c_sc18is600_exit(void)
{
	spi_unregister_driver(&sc18is600_driver);
}

MODULE_AUTHOR("Lapin Roman <lampus.lapin@gmail.com>");
MODULE_DESCRIPTION("I2C driver for SC18IS600 SPI to I2C bridge");
MODULE_LICENSE("GPL");

module_init(i2c_sc18is600_init);
module_exit(i2c_sc18is600_exit);
