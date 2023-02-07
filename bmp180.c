#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

/* mdelay */
#include <linux/delay.h>

/* hwmon */
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

/* regmap */
#include <linux/regmap.h>

#define BMP180_CHIP_ID_REG_ADDR 			(0xD0)
	#define BMP180_CHIP_ID					(0x55)

#define BMP180_SOFT_REG_ADDR				(0xE0)
	#define BMP180_SOFT_RESET				(0xB6)

#define BMP180_CTRL_REG_ADDR				(0xF4)
	#define BMP180_CTRL_SOC					(0x01 << 5)
	#define BMP180_CTRL_MEAS_TEMP			(0x2E)
	#define BMP180_CTRL_MEAS_PRES			(0x34)
	#define BMP180_CTRL_OSS_LOW				(0x00)
	#define BMP180_CTRL_OSS_STD				(0x01<<6)
	#define BMP180_CTRL_OSS_HIGH			(0x01<<7)
	#define BMP180_CTRL_OSS_ULTRA_HIGH		((0x01<<6)|(0x01<<7))

#define BMP180_OUT_XLSB_REG_ADDR			(0xF8)
#define BMP180_OUT_LSB_REG_ADDR				(0xF7)
#define BMP180_OUT_MSB_REG_ADDR				(0xF6)

#define BMP180_CALI_FIRST_REG				(0xAA)
#define BMP180_CALI_LAST_REG				(0xBF)

#define BMP180_CALC_COUNT 		(10)
#define BMP180_TEMP_MEAS_WAIT	(5) /* ms */

#define BMP180_NAME "BMP180"

enum {
	MODE_OSS_LOW,
	MODE_OSS_STAND,
	MODE_OSS_HIGH,
	MODE_OSS_ULTRA,
	MODE_OSS_MAX,
};

u8 pres_delay[MODE_OSS_MAX] = { 5, 8, 14, 26 }; /* ms */ 
struct bmp180_status {
	u32 mode;
};

struct bmp180_chip {
	struct i2c_client *client;
	struct device *hwmon_dev;
	struct regmap *regmap; 
	struct bmp180_status status;
};

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

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off debug level log(default 0)");

#define dprintk(fmt, arg...) do {                   \
    if (debug)                          \
        printk(KERN_DEBUG pr_fmt("%s: " fmt),  __func__, ##arg);         \
} while (0)

static int bmp180_get_cali(const struct bmp180_chip *bmp180)
{
	int i, ret = 0;
	u32 msb, lsb;

	for(i = BMP180_CALI_FIRST_REG, msb = 0, lsb = 0; i < BMP180_CALI_LAST_REG; i+=2)
	{
		ret = regmap_read(bmp180->regmap, i, &msb);
		if(ret)
			return -EIO;
		
		ret = regmap_read(bmp180->regmap, i+1, &lsb);
		if(ret)
			return -EIO;

		calis[(i-BMP180_CALI_FIRST_REG)/2] = 0xFF & msb;
		calis[(i-BMP180_CALI_FIRST_REG)/2] = calis[(i-BMP180_CALI_FIRST_REG)/2] << 8;
		calis[(i-BMP180_CALI_FIRST_REG)/2] |= (0xFF & lsb);
	}

	return 0;
}

static u32 bmp180_util_calc_pressure(const u32 up)
{	
	long t = 0,x1 = 0,x2 = 0,b5 = 0;
	
	x1 = ((up-calis[AC6]) * (calis[AC5]*10000/(1<<15)))/10000;
	x2 = (calis[MC]*(1<<11))/(x1+calis[MD]);
	b5 = x1 + x2;
	t = (b5+8)/(1<<4);
	
	return (u16)t;
}

static int bmp180_pressure_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct bmp180_chip *bmp180 = dev_get_drvdata(dev);
	struct bmp180_status status = bmp180->status;
	int ret = 0;
	u8 cmd = 0, i = 0;
 	u32 pres = 0, tmp = 0, up = 0;

	cmd = BMP180_CTRL_MEAS_PRES|(status.mode << 6);

	while(i++ < BMP180_CALC_COUNT)
	{
		ret = regmap_write(bmp180->regmap, BMP180_CTRL_REG_ADDR, cmd);
		if(ret)
			return ret;

		mdelay(pres_delay[status.mode]);

		up = 0;
		regmap_read(bmp180->regmap, BMP180_OUT_MSB_REG_ADDR, &tmp);
		up = 0xFF00 & (tmp << 8);
		regmap_read(bmp180->regmap, BMP180_OUT_LSB_REG_ADDR, &tmp);
		up |= 0x00FF & tmp;

		pres += bmp180_util_calc_pressure(up);
	}

	return sprintf(buf, "%d", pres/BMP180_CALC_COUNT);
}

static u16 bmp180_util_calc_temperature(const u16 ut)
{	
	long t = 0,x1 = 0,x2 = 0,b5 = 0;
	
	x1 = ((ut-calis[AC6]) * (calis[AC5]*10000/(1<<15)))/10000;
	x2 = (calis[MC]*(1<<11))/(x1+calis[MD]);
	b5 = x1 + x2;
	t = (b5+8)/(1<<4);
	
	return (u16)t;
}

static int bmp180_temperature_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct bmp180_chip *bmp180 = dev_get_drvdata(dev);
	int ret = 0, i = 0;
	u32 ctrl = 0, temp = 0, tmp = 0;
	u16 ut = 0;
	
	while(i++ < BMP180_CALC_COUNT)
	{
		ret = regmap_write(bmp180->regmap, BMP180_CTRL_REG_ADDR, BMP180_CTRL_MEAS_TEMP);
		if(ret)
			return ret;

		mdelay(BMP180_TEMP_MEAS_WAIT);

		ut = 0;
		regmap_read(bmp180->regmap, BMP180_OUT_MSB_REG_ADDR, &tmp);
		ut = 0xFF00 & (tmp << 8);
		regmap_read(bmp180->regmap, BMP180_OUT_LSB_REG_ADDR, &tmp);
		ut |= 0x00FF & tmp;

		temp += bmp180_util_calc_temperature(ut);
	}	

	return sprintf(buf, "%d", temp/BMP180_CALC_COUNT);
}

static int bmp180_reset(const struct bmp180_chip *bmp180)
{
	int ret = 0;	
	
	ret = regmap_write(bmp180->regmap, BMP180_SOFT_REG_ADDR, BMP180_SOFT_RESET);	
	if(ret != 0)
		return ret;

	return 0;
}

static int bmp180_check_communication(const struct bmp180_chip *bmp180)
{
	int ret = 0;
	u32 id = 0;
		
	ret = regmap_read(bmp180->regmap, BMP180_CHIP_ID_REG_ADDR, &id);
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

static SENSOR_DEVICE_ATTR_RO(temp, bmp180_temperature, 0);
static SENSOR_DEVICE_ATTR_RO(pres, bmp180_pressure, 1);
static struct attribute *bmp180_attrs[] = {
	&sensor_dev_attr_temp.dev_attr.attr,
	&sensor_dev_attr_pres.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(bmp180);

static const struct regmap_config bmp180_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int bmp180_parse_dt(struct device_node *np, struct bmp180_chip *bmp180)
{
	int ret = 0;

	ret = of_property_read_u32(np, "oss_mode", &bmp180->status.mode);
	if(ret) {
		pr_err("Not founded the oss_mode property in device node.\n");
		return ret;
	}

	dprintk("oss_mode: %d\n", bmp180->status.mode);

	return 0;
}

static int bmp180_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	struct bmp180_chip *bmp180;
	struct device *hwmon_dev;
    int err = 0;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		pr_err("No support i2c smbus");	
		return -EIO;
	}

	bmp180 = devm_kzalloc(&client->dev, sizeof(*bmp180), GFP_KERNEL);
	if(!bmp180)
		return -ENOMEM; 

	bmp180->client = client;	
	bmp180->regmap = devm_regmap_init_i2c(client, &bmp180_regmap_config);
	if (IS_ERR(bmp180->regmap)) 
		return PTR_ERR(bmp180->regmap);

	err = bmp180_parse_dt(client->dev.of_node, bmp180);
	if(err)
		return err;

	err = bmp180_check_communication(bmp180);
	if(err)
		return err;

	err = bmp180_get_cali(bmp180);
	if(err)
		return err;

	hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev, client->name, bmp180, bmp180_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
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
};

module_i2c_driver(bmp180_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yohan Yoon <dbsdy1235@gmail.com>");
MODULE_DESCRIPTION("BMP180 Device Driver");
