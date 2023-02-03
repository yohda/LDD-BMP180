#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

/* mdelay() */
#include <linux/delay.h>

#define BMP180_CHIP_ID_REG_ADDR 			(0xD0)
	#define BMP180_CHIP_ID					(0x55)

#define BMP180_SOFT_REG_ADDR				(0xE0)
	#define BMP180_SOFT_RESET				(0xB6)

#define BMP180_CTRL_REG_ADDR				(0xF4)
	#define BMP180_CTRL_SOC					(0x01 << 5)
	#define BMP180_CTRL_MEAS_TEMP			(0x2E)
	#define BMP180_CTRL_OSS_LOW				(0x00)
	#define BMP180_CTRL_OSS_STD				(0x40)
	#define BMP180_CTRL_OSS_HIGH			(0x80)
	#define BMP180_CTRL_OSS_ULTRA_HIGH		(0xC0)

#define BMP180_OUT_XLSB_REG_ADDR			(0xF8)
#define BMP180_OUT_LSB_REG_ADDR				(0xF7)
#define BMP180_OUT_MSB_REG_ADDR				(0xF6)

#define BMP180_CALI_FIRST_REG				(0xAA)
#define BMP180_CALI_LAST_REG				(0xBF)


#define BMP180_NAME "BMP180"

enum {
	AC1 = 0,
	AC2,
	AC3,
	AC4,
	AC5,
	AC6,
	B1,
	B2,
	MB,
	MC,
	MD,
	CALI_MAX,
};

static short calis[CALI_MAX];

static int bmp180_i2c_write(const struct i2c_client *client, const u8 reg, const u8 data)
{
	if(!client)
	{
		pr_err("i2c istances isn`t exist.\n");
		return -EINVAL;
	}

	if(i2c_smbus_write_byte_data(client, reg, data) < 0)
	{
		pr_err("bmp180 i2c write error. reg:0x%x, data:0x%x\n", reg, data);
		return -EIO;
	}
     
    return 0;
}

static int bmp180_i2c_read(const struct i2c_client *client, const u8 reg, u8 *buf)
{
	int ret = 0;
	if(!client)
	{
		pr_err("i2c istances isn`t exist.\n");
		return -EINVAL;
	}
	
	ret = i2c_smbus_read_byte_data(client, reg);
	if(ret < 0)
	{
		pr_err("bmp180 i2c read error. reg:0x%x\n", reg);
		return -EIO;
	}

	*buf = (u8)(ret & 0xFF);
	 
    return 0;
}

static int bmp180_get_cali(const struct i2c_client *client)
{
	int i, ret = 0;
	u8 msb, lsb;

	for(i = BMP180_CALI_FIRST_REG, msb = 0, lsb = 0; i < BMP180_CALI_LAST_REG; i+=2)
	{
		ret = bmp180_i2c_read(client, i, &msb);
		if(ret)
		{	
			pr_err("[F:%d][L:%d] Failed to get cali. reg:0x%x, err:%d\n", __func__, __LINE__, i, ret);
			return -EIO;
		}
		
		ret = bmp180_i2c_read(client, i+1, &lsb);
		if(ret)
		{	
			pr_err("[F:%d][L:%d] Failed to get cali. reg:0x%x, err:%d\n", __func__, __LINE__, i+1, ret);
			return -EIO;
		}

		calis[(i-BMP180_CALI_FIRST_REG)/2] = 0xFF & msb;
		calis[(i-BMP180_CALI_FIRST_REG)/2] = calis[(i-BMP180_CALI_FIRST_REG)/2] << 8;
		calis[(i-BMP180_CALI_FIRST_REG)/2] |= (0xFF & lsb);
	}

	return 0;
}
	
static u16 bmp180_util_calc_temperature(const u16 ut)
{	
	long t = 0,x1 = 0,x2 = 0,b5 = 0;
	
	x1 = ((ut-calis[AC6]) * (calis[AC5]*10000/(1<<15)))/10000;
	x2 = (calis[MC]*(1<<11))/(x1+calis[MD]);
	b5 = x1 + x2;
	t = (b5+8)/(1<<4);
	
	pr_info("t:%ld\n", t);	
	
	return 0;
}

static int bmp180_get_temperature(const struct i2c_client *client)
{
	int ret = 0, i = 0;
	u8 ctrl = 0;
	u16 ut = 0;

	ret = bmp180_i2c_read(client, BMP180_CTRL_REG_ADDR, &ctrl);
	if(ret < 0)
		return -EIO;
	
	ctrl = (u8)(ret & 0xFF);
	ctrl |= BMP180_CTRL_MEAS_TEMP;
	pr_info("ctrl:0x%x\n", ctrl);
	
	ret = bmp180_i2c_write(client, BMP180_CTRL_REG_ADDR, ctrl);
	if(ret)
	{
		pr_err("[F:%d][L:%d] Failed to get temperature. err:%d\n", __func__, __LINE__, ret);
		return ret;
	}

	while(i < 10)
	{
		ctrl = (u8)(ret & 0xFF);
		ctrl |= BMP180_CTRL_MEAS_TEMP;
		ut = 0;
		
		ret = bmp180_i2c_write(client, BMP180_CTRL_REG_ADDR, ctrl);
		if(ret)
		{
			pr_err("[F:%d][L:%d] Failed to get temperature. err:%d\n", __func__, __LINE__, ret);
			return ret;
		}

		bmp180_i2c_read(client, BMP180_OUT_MSB_REG_ADDR, (u8 *)&ut);
		ut = (ut << 8);
		bmp180_i2c_read(client, BMP180_OUT_LSB_REG_ADDR, (u8 *)&ut);
		
		pr_info("ut:%d\n", ut);
	
		bmp180_util_calc_temperature(ut);
		
		i++;
	}	

	return 0;
}

static int bmp180_reset(const struct i2c_client *client)
{
	int ret = 0;	
	
	ret = bmp180_i2c_write(client, BMP180_SOFT_REG_ADDR, BMP180_SOFT_RESET);
	if(ret != 0)
		return ret;

	return 0;
}

static int bmp180_check_communication(const struct i2c_client *client)
{
	int ret = 0;
	u8 id = 0;

	ret = bmp180_i2c_read(client, BMP180_CHIP_ID_REG_ADDR, &id);
	if(ret < 0)
	{
		pr_err("Failed to communicate with bmp180 via i2c\n");
		return ret;
	}

	if(id != BMP180_CHIP_ID)
	{
		pr_err("Communication is not good. id:0x%x\n", id);
		return -EIO; 
	}

	return ret;
}

static int bmp180_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int err = 0;
	pr_info("BMP180 Probe Started!!!\n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		pr_err("No support i2c smbus");	
		return -EIO;
	}
	
	err = bmp180_check_communication(client);
	if(err)
		return err;

	err = bmp180_get_cali(client);
	if(err)
		return err;

	bmp180_get_temperature(client);
	
	pr_info("BMP180 Probe Done!!!\n");   	
	return 0;
}

static int bmp180_remove(struct i2c_client *client)
{   
    pr_info("YOHDA BMP180 Removed!!!\n");
    return 0;
}

static const struct of_device_id bmp180_dts[] = {
        { .compatible = "robert-bosch,bmp180" },
        {}
};
MODULE_DEVICE_TABLE(of, bmp180_dts);

static struct i2c_driver bmp180_driver = {
        .driver = {
            .name   = BMP180_NAME,
            .owner  = THIS_MODULE,
			.of_match_table = bmp180_dts,
        },
        .probe          = bmp180_probe,
        .remove         = bmp180_remove,
};

module_i2c_driver(bmp180_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yohan Yoon <dbsdy1235@gmail.com>");
MODULE_DESCRIPTION("BMP180 Device Driver");
