#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/string.h>

static char aw9120_ram_name[64]="defaul.bin";

#define SUCCESS 0
#define FAIL    -1

#define REG_RSTR    0x00
#define REG_GCR     0x01

#define REG_LER1    0x50
#define REG_LER2    0x51
#define REG_LCR     0x52
#define REG_PMR     0x53
#define REG_RMR     0x54
#define REG_CTRS1   0x55
#define REG_CTRS2   0x56
#define REG_IMAX1   0x57
#define REG_IMAX2   0x58
#define REG_IMAX3   0x59
#define REG_IMAX4   0x5a
#define REG_IMAX5   0x5b
#define REG_TIER    0x5c
#define REG_INTVEC  0x5d
#define REG_LISR2   0x5e
#define REG_SADDR   0x5f

#define REG_PCR     0x60
#define REG_CMDR    0x61
#define REG_RA      0x62
#define REG_RB      0x63
#define REG_RC      0x64
#define REG_RD      0x65
#define REG_R1      0x66
#define REG_R2      0x67
#define REG_R3      0x68
#define REG_R4      0x69
#define REG_R5      0x6a
#define REG_R6      0x6b
#define REG_R7      0x6c
#define REG_R8      0x6d
#define REG_GRPMSK1 0x6e
#define REG_GRPMSK2 0x6f

#define REG_TCR     0x70
#define REG_TAR     0x71
#define REG_TDR     0x72
#define REG_TDATA   0x73
#define REG_TANA    0x74
#define REG_TKST    0x75
#define REG_FEXT    0x76
#define REG_WP      0x7d
#define REG_WADDR   0x7e
#define REG_WDATA   0x7f


#define LEDS_12     12
#define LEDS_20     20
#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5

static int start_ui_code[] = {
	0xbf00,
	0x9f03,
	//power
	0xe9fa,
	0xeafa,
	0xebfa,
	0x3c20,//delay
	0xc9fa,
	0xcafa,
	0xcbfa,
	0x3c38,//delay

	//multyfunc
	0xe0fa,
	0xedfa,
	0xecfa,
	0x3c20,//delay
	0xc0fa,
	0xcdfa,
	0xccfa,
	0x3c38,//delay

	//pair
	0xe1fa,
	0xe2fa,
	0xe3fa,
	0x3c20,//delay
	0xc1fa,
	0xc2fa,
	0xc3fa,
	0x3c38,//delay

	//mute
	0xeefa,
	0xeffa,
	0xe4fa,
	0x3c20,//delay
	0xcefa,
	0xcffa,
	0xc4fa,
	0x3c38,//delay

	//pair
	0xe1fa,
	0xe2fa,
	0xe3fa,
	0x3c20,//delay
	0xc1fa,
	0xc2fa,
	0xc3fa,
	0x3c38,//delay

	//multyfunc
	0xe0fa,
	0xedfa,
	0xecfa,
	0x3c20,//delay
	0xc0fa,
	0xcdfa,
	0xccfa,
	0x3c38,//delay

	0x0002,
};

static int ui_code_len = sizeof(start_ui_code)/sizeof(start_ui_code[0]);

typedef enum
{
	SRAM_CTRL,
	IIC_CTRL
} aw9120_source;

typedef enum
{
	LedMaxC_0,//0mA
	LedMaxC_3_5,//3.5mA
	LedMaxC_7_0,//7.0mA
	LedMaxC_10_5,//10.5mA
	LedMaxC_14_0,//14.0mA
	LedMaxC_17_5,//17.5mA
	LedMaxC_21_0,//21.0mA
	LedMaxC_24_5//24.5mA
} led_maxcurrent;

struct aw9120_priv;
struct aw9120_led_data {
	struct led_classdev cdev;
	u8 channel; /* 1-based, max priv->cdef->channels */
	struct aw9120_priv *priv;
	led_maxcurrent maxcurrent;
};

struct aw9120_priv {
	//	const struct aw9120_chipdef *cdef;
	struct i2c_client *client;
	unsigned int num_leds;
	struct mutex lock;
	struct aw9120_led_data leds[0];
};

unsigned int *pattern_num;
unsigned int pattern_num_size;


static int i2c_write(struct aw9120_priv *aw9120, unsigned char addr, unsigned int reg_data)
{
	int ret;
	unsigned char wbuf[512] = { 0 };

	struct i2c_msg msgs[] = {
		{
			.addr = aw9120->client->addr,
			.flags = 0,
			.len = 3,
			.buf = wbuf,
		},
	};

	wbuf[0] = addr;
	wbuf[1] = (unsigned char)(reg_data >> 8);
	wbuf[2] = (unsigned char)(reg_data & 0x00ff);

	ret = i2c_transfer(aw9120->client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("%s: i2c write error: %d\n", __func__, ret);

	return ret;
}

static int i2c_read(struct aw9120_priv *aw9120, unsigned char addr, unsigned int *reg_data)
{
	int ret;
	unsigned char rbuf[512] = { 0 };
	unsigned int get_data;

	struct i2c_msg msgs[] = {
		{
			.addr = aw9120->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = aw9120->client->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = rbuf,
		},
	};

	ret = i2c_transfer(aw9120->client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	get_data = (unsigned int)(rbuf[0] << 8);
	get_data |= (unsigned int)(rbuf[1]);

	*reg_data = get_data;

	return ret;
}

static int aw9120_i2c_write(struct aw9120_priv *aw9120, unsigned char reg_addr, unsigned int reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write(aw9120, reg_addr, reg_data);
		if (ret < 0) {
			printk("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
					ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
				AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw9120_i2c_read(struct aw9120_priv *aw9120, unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read(aw9120, reg_addr, reg_data);
		if (ret < 0) {
			printk("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
					ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
				AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw9120_enable(struct aw9120_priv *aw9120)
{
	//software enabled
	aw9120_i2c_write(aw9120, REG_GCR, 0x0001);//enable LED module
	return SUCCESS;
}

static int aw9120_software_reset(struct aw9120_priv *aw9120)
{
	aw9120_i2c_write(aw9120, REG_RSTR, 0x55AA);
	return SUCCESS;
}

//static int aw9120_hart_reset();

static int aw9120_enable_ledx(struct aw9120_led_data *led_data)
{
	unsigned int data = 0;
	unsigned char pin = led_data->channel;

	/* code */
	if(pin < LEDS_12){
		//enable driver output
		aw9120_i2c_read(led_data->priv, REG_LER1, &data);
		data |= (1 << pin);
		aw9120_i2c_write(led_data->priv, REG_LER1, data);
	}else if(pin < LEDS_20){
		//enable driver output
		aw9120_i2c_read(led_data->priv, REG_LER2, &data);
		data |= (1 << (pin - LEDS_12));
		aw9120_i2c_write(led_data->priv, REG_LER2, data);
	}

	return SUCCESS;
}

static int aw9120_enable_all_led(struct aw9120_priv *aw9120)
{
	aw9120_i2c_write(aw9120, REG_LER1, 0x0FFF); /* LER1-LED1~LED12 Enable */
	aw9120_i2c_write(aw9120, REG_LER2, 0x00FF); /* LER2-LED13~LED20 Enable */

	return SUCCESS;
}

static int aw9120_set_ledx_maxcurrent(struct aw9120_led_data *led_data)
{
	unsigned int data = 0;

	if(led_data->channel < 4){
		aw9120_i2c_read(led_data->priv, REG_IMAX1, &data);
		data &= (~(0x0007 << (led_data->channel * 4)));
		data |= (led_data->maxcurrent << (led_data->channel * 4));
		aw9120_i2c_write(led_data->priv, REG_IMAX1, data);
	}else if(led_data->channel < 8){
		aw9120_i2c_read(led_data->priv, REG_IMAX2, &data);
		data &= (~(0x0007 << ((led_data->channel - 4) * 4)));
		data |= (led_data->maxcurrent << ((led_data->channel - 4) * 4));
		aw9120_i2c_write(led_data->priv, REG_IMAX2, data);
	}else if(led_data->channel < LEDS_12){
		aw9120_i2c_read(led_data->priv, REG_IMAX3, &data);
		data &= (~(0x0007 << ((led_data->channel - 8) * 4)));
		data |= (led_data->maxcurrent << ((led_data->channel - 8) * 4));
		aw9120_i2c_write(led_data->priv, REG_IMAX3, data);
	}else if(led_data->channel < 16){
		aw9120_i2c_read(led_data->priv, REG_IMAX4, &data);
		data &= (~(0x0007 << ((led_data->channel - LEDS_12) * 4)));
		data |= (led_data->maxcurrent << ((led_data->channel - LEDS_12) * 4));
		aw9120_i2c_write(led_data->priv, REG_IMAX4, data);
	}else if(led_data->channel < LEDS_20){
		aw9120_i2c_read(led_data->priv, REG_IMAX5, &data);
		data &= (~(0x0007 << ((led_data->channel - 16) * 4)));
		data |= (led_data->maxcurrent << ((led_data->channel - 16) * 4));
		aw9120_i2c_write(led_data->priv, REG_IMAX5, data);
	}

	return SUCCESS;
}

static int aw9120_set_all_led_maxcurrent(struct aw9120_priv *aw9120, led_maxcurrent maxcurrent)
{
	unsigned int imax = 0;
	imax = (maxcurrent << 12) | (maxcurrent << 8) |
		(maxcurrent << 4) | (maxcurrent << 0);

	aw9120_i2c_write(aw9120, REG_IMAX1, imax);  /* IMAX1-LED1~LED4 Current */
	aw9120_i2c_write(aw9120, REG_IMAX2, imax);  /* IMAX2-LED5~LED8 Current */
	aw9120_i2c_write(aw9120, REG_IMAX3, imax);  /* IMAX3-LED9~LED12 Current */
	aw9120_i2c_write(aw9120, REG_IMAX4, imax);  /* IMAX4-LED13~LED16 Current */
	aw9120_i2c_write(aw9120, REG_IMAX5, imax);  /* IMAX5-LED17~LED20 Current */

	return SUCCESS;
}

static int aw9120_set_ledx_source(struct aw9120_led_data *led_data, aw9120_source source)
{
	unsigned int data = 0;

	if(led_data->channel < LEDS_12){
		aw9120_i2c_read(led_data->priv, REG_CTRS1, &data);
		if (source == IIC_CTRL){
			data |= (1 << led_data->channel);
		}else{
			data &= ~(1 << led_data->channel);
		}
		aw9120_i2c_write(led_data->priv, REG_CTRS1, data);
	}else if(led_data->channel < LEDS_20){
		aw9120_i2c_read(led_data->priv, REG_CTRS2, &data);
		if (source == IIC_CTRL){
			data |= (1 << (led_data->channel - LEDS_12));
		}else{
			data &= ~(1 << (led_data->channel - LEDS_12));
		}
		aw9120_i2c_write(led_data->priv, REG_CTRS2, data);
	}

	return SUCCESS;
}

static int aw9120_set_all_led_source(struct aw9120_priv *aw9120, aw9120_source source)
{

	if (source == SRAM_CTRL){
		/* code */
		aw9120_i2c_write(aw9120, REG_CTRS1, 0x0000);    /* CTRS1-LED1~LED12: SRAM Control */
		aw9120_i2c_write(aw9120, REG_CTRS2, 0x0000);    /* CTRS2-LED13~LED20: SRAM Control */
	}else{
		/* code */
		aw9120_i2c_write(aw9120, REG_CTRS1, 0x0fff);    /* CTRS1-LED1~LED12: IIC Control */
		aw9120_i2c_write(aw9120, REG_CTRS2, 0x00ff);    /* CTRS2-LED13~LED20: IIC Control */
	}

	return SUCCESS;
}

static int aw9120_turn_off_ledx(struct aw9120_led_data *led_data)
{
	unsigned int data = 0;
	aw9120_source source = IIC_CTRL;
	//send SETPWMI cmd
	data = 0xA000;
	data |= led_data->channel << 8;
	data |= (0x0000);
	aw9120_i2c_write(led_data->priv, REG_CMDR, data);

	if(led_data->channel < LEDS_12){
		//disable driver output
		aw9120_i2c_read(led_data->priv, REG_LER1, &data);
		data &= (~(1 << led_data->channel));
		aw9120_i2c_write(led_data->priv, REG_LER1, data);
	}else if(led_data->channel < LEDS_20){
		aw9120_i2c_read(led_data->priv, REG_LER2, &data);
		data &= (~(1 << (led_data->channel - LEDS_12)));
		aw9120_i2c_write(led_data->priv, REG_LER2, data);
	}

	//    aw9120_set_ledx_source(led_data, source);

	return SUCCESS;
}

static int aw9120_turn_off_all_led(struct aw9120_priv *aw9120)
{
	aw9120_source source = IIC_CTRL;

	aw9120_i2c_write(aw9120, REG_LER1, 0x0000); /* LER1-LED1~LED12 disable */
	aw9120_i2c_write(aw9120, REG_LER2, 0x0000); /* LER2-LED13~LED20 disable */

	aw9120_set_all_led_source(aw9120, source);

	return SUCCESS;    
}

static int aw9120_set_brightness(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	int data = 0;
	int ret;
	const struct aw9120_led_data *led_data = container_of(led_cdev, struct aw9120_led_data, cdev);    
	dev_dbg(led_cdev->dev, "%s: %d\n", __func__, brightness);

	mutex_lock(&(led_data->priv->lock));
	//send SETPWMI cmd
	data = 0xA000; //SETPWMI Ch Im
	data |= (led_data->channel) << 8;
	data |= (brightness);
	ret = aw9120_i2c_write(led_data->priv ,REG_CMDR, data);
	if (ret < 0) {
		printk("%s: i2c_write error=%d\n", __func__, ret);
		mutex_unlock(&led_data->priv->lock);
		return ret;
	}		

	mutex_unlock(&(led_data->priv->lock));
	return SUCCESS;    
}

static int aw9120_run_sram(struct aw9120_priv *aw9120, int addr)
{
	unsigned int i = 0;

	for (i = 0; i < pattern_num_size; i++){
		if (addr == (int)pattern_num[i]){	
			aw9120_source source = SRAM_CTRL;
			aw9120_turn_off_all_led(aw9120);
			aw9120_enable_all_led(aw9120);
			aw9120_set_all_led_source(aw9120, source);

			printk("run pattern = %d\n",pattern_num[i]);
			/* LED SRAM Run */
			aw9120_i2c_write(aw9120, REG_SADDR, addr);	/* SADDR-SRAM Run Start Addr:0 */
			aw9120_i2c_write(aw9120, REG_PMR, 0x0001);	/* PMR-Reload and Excute SRAM */
			aw9120_i2c_write(aw9120, REG_RMR, 0x0002);	/* RMR-Run */
			return SUCCESS;
		}
	}
	return SUCCESS;
}

static void aw9120_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw9120_priv *aw9120 = context;
	int i;
	unsigned char ram_cnt = 0;
	unsigned int ram_temp[256] = { 0 };
	unsigned int pattern_temp[24]= { 0 };
	unsigned int pattern_cnt = 0;

	pr_debug("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw9120_ram_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw9120_ram_name,
			cont ? cont->size : 0);

	for (i = 0; i < cont->size; i++) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i,
				*(cont->data + i));
	}

	/* LED SRAM Hold Mode */
	aw9120_i2c_write(aw9120, REG_PMR, 0x0000);	/* PMR-Load SRAM with I2C */
	aw9120_i2c_write(aw9120, REG_RMR, 0x0000);	/* RMR-Hold Mode */

	/* Load LED SRAM */
	aw9120_i2c_write(aw9120, REG_WADDR, 0x0000);	/* WADDR-SRAM Load Addr */
	for (i = 0; i < cont->size; i += 2) {
		ram_temp[ram_cnt] = *(cont->data + i) << 8;
		ram_temp[ram_cnt] |= *(cont->data + i + 1);
		ram_cnt++;
	}

	for (i = 0; i < ram_cnt; i++){
//		pr_info("------%s[%d]: data:0x%04x------\n", __func__, i, ram_temp[i]);
	}

	for (i = 0; i < ram_cnt; i++){
		aw9120_i2c_write(aw9120, REG_WDATA, ram_temp[i]);
		pr_info("%s[%d]: data:0x%04x\n", __func__, i, ram_temp[i]);
		if (pattern_cnt < 24){
			if ((ram_temp[i] & 0xFF00) == 0){
				pr_info("%s: sram-addr:0x%04x   ", __func__, ram_temp[i]);
				pattern_temp[pattern_cnt] = ram_temp[i];
				pattern_temp[++ pattern_cnt] = ram_temp[i+1]; // recive pattern 
				pr_info("ADK-pattern:0x%04x\n", ram_temp[i+1]);
				pattern_cnt ++;
			}
		}
	}

	pattern_num = (unsigned int*)kmalloc(pattern_cnt, GFP_KERNEL);
	pattern_num_size = pattern_cnt;
	for ( i = 0; i < pattern_cnt; i++){
		pattern_num[i] = pattern_temp[i];
//		printk("--------pattern_num= 0x%04x\n",pattern_num[i]);
	}

	aw9120_source source = SRAM_CTRL;                       
	aw9120_set_all_led_source(aw9120, source);

	aw9120_i2c_write(aw9120, REG_SADDR, 0x0000);	/* SADDR-SRAM Run Start Addr:0 */
	aw9120_i2c_write(aw9120, REG_PMR, 0x0001);	/* PMR-Reload and Excute SRAM */
	aw9120_i2c_write(aw9120, REG_RMR, 0x0002);	/* RMR-Run */

	release_firmware(cont);

	pr_info("%s: fw update complete\n", __func__);
}

static int aw9120_ram_update(struct device *dev)
{

	//struct led_classdev *led_cdev = dev_get_drvdata(dev);
	//struct aw9120_led_data *led_data = container_of(led_cdev, struct aw9120_led_data, cdev);
	struct aw9120_priv *aw9120 = dev_get_drvdata(dev);
	aw9120_turn_off_all_led(aw9120);
	aw9120_enable_all_led(aw9120);
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw9120_ram_name, dev, GFP_KERNEL,
			aw9120, aw9120_ram_loaded);
}

static ssize_t aw9120_ram_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	//sprintf(aw9120_ram_name, "%s", buf);
	memset(aw9120_ram_name, '\0', 64);	
	strncpy(aw9120_ram_name, buf, count-1);
	printk("buf lenth= %d %d \r\n", strlen(buf), count);
	if (strncmp(aw9120_ram_name, "iic_conctrl.bin", strlen("iic_conctrl.bin")) != 0){
		aw9120_ram_update(dev);
//		printk("----------------------%s\n", aw9120_ram_name);
	}else{
//		printk("======================%s\n",aw9120_ram_name);
		struct aw9120_priv *aw9120 = dev_get_drvdata(dev);
		aw9120_source source = IIC_CTRL;
		aw9120_turn_off_all_led(aw9120);
		aw9120_enable_all_led(aw9120);
		aw9120_set_all_led_source(aw9120, source);
	}		

	return count;
}

static ssize_t aw9120_ram_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw9120_ram_show, aw9120_ram_store);

static ssize_t aw9120_pattern_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw9120_priv *aw9120 = dev_get_drvdata(dev);
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		printk("[INFO] aw9120_pattern_store = %x\r\n", databuf[0]);
		aw9120_run_sram(aw9120, databuf[0]);
	}
	return count;	
}

static ssize_t aw9120_pattern_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	size_t i = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "pattern ");
	for (i = 0; i < pattern_num_size; i++)
	{
		if ((pattern_num[i] & 0xff00) == 0){
			len += snprintf(buf + len, PAGE_SIZE - len, "%d ", pattern_num[i]);
		}else{
			len += snprintf(buf + len, PAGE_SIZE - len, "%d ", pattern_num[i] & 0xff);
		}
	}	
	len += snprintf(buf + len, PAGE_SIZE - len, "end\r\n");
	return len;
}

static DEVICE_ATTR(pattern, S_IWUSR | S_IRUGO, aw9120_pattern_show, aw9120_pattern_store);

static struct attribute *aw9120_attributes[] = {
	&dev_attr_ram.attr,
	&dev_attr_pattern.attr,
	NULL
};

static struct attribute_group aw9120_attribute_group = {
	.attrs = aw9120_attributes
};

static int aw9120_source_update(struct device *dev, aw9120_source source)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9120_led_data *led_data = container_of(led_cdev, struct aw9120_led_data, cdev);

	aw9120_turn_off_ledx(led_data);
	aw9120_set_ledx_source(led_data, source);
	aw9120_enable_ledx(led_data);

	return SUCCESS;
}
static ssize_t aw9120_led_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int databuf[1] = { 0 };
	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if ((databuf[0] == 1) | (databuf[0] == 0))
			aw9120_source_update(dev, (aw9120_source) databuf[0]);	
	}

	return count;
}

static ssize_t aw9120_led_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}
static DEVICE_ATTR(source, S_IWUSR | S_IRUGO, aw9120_led_source_show, aw9120_led_source_store);

static void aw9120_bootloader_led(struct aw9120_priv *aw9120, unsigned int *start_ui_code, unsigned int ui_code_len)
{

	int i = 0;	
	/* LED SRAM Hold Mode */
	aw9120_i2c_write(aw9120, REG_PMR, 0x0000);	/* PMR-Load SRAM with I2C */
	aw9120_i2c_write(aw9120, REG_RMR, 0x0000);	/* RMR-Hold Mode */

	/* Load LED SRAM */
	aw9120_i2c_write(aw9120, REG_WADDR, 0x0000);	/* WADDR-SRAM Load Addr */

	for (i = 0; i < ui_code_len; i++){
		aw9120_i2c_write(aw9120, REG_WDATA, start_ui_code[i]);
	}

	aw9120_source source = SRAM_CTRL;
	aw9120_enable_all_led(aw9120);
	aw9120_set_all_led_source(aw9120, source);

	aw9120_i2c_write(aw9120, REG_SADDR, 0x0000);	/* SADDR-SRAM Run Start Addr:0 */
	aw9120_i2c_write(aw9120, REG_PMR, 0x0001);	/* PMR-Reload and Excute SRAM */
	aw9120_i2c_write(aw9120, REG_RMR, 0x0002);	/* RMR-Run */

}

static struct attribute *led_attributes[] = {
	&dev_attr_source.attr,

	NULL
};

static struct attribute_group led_attribute_group = {
	.attrs = led_attributes
};

static int aw9120_init(struct aw9120_priv *aw9120)
{
	//	const struct aw9120_chipdef *cdef = aw9120->cdef;
	u8 channels = aw9120->num_leds;
	unsigned char i = 0;
	int ret;
	led_maxcurrent maxcurrent = LedMaxC_0;
	aw9120_source source = IIC_CTRL;

	ret = aw9120_software_reset(aw9120);
	if (ret < 0)
		return ret;

	aw9120_enable(aw9120);
	aw9120_enable_all_led(aw9120);
	//default is IIC control
	aw9120_set_all_led_source(aw9120, source); 
	//default current is 0
	aw9120_set_all_led_maxcurrent(aw9120, maxcurrent);

	// follow dts to set channel maxcurrent
	for (i = 0; i < channels; i++){
		printk("[INFO] i = %d channel= %d\r\n", i , aw9120->leds[i].channel);
		aw9120_set_ledx_maxcurrent(&(aw9120->leds[i]));
	}

	/*
	   request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
	   aw9120_ram_name, &(aw9120->client->dev), GFP_KERNEL,
	   aw9120, aw9120_ram_loaded);
	 */
	aw9120_bootloader_led(aw9120, start_ui_code, ui_code_len);

	return 0;    
}

static inline size_t sizeof_aw9120_priv(int num_leds)
{
	return sizeof(struct aw9120_priv) +
		(sizeof(struct aw9120_led_data) * num_leds);
}

static struct aw9120_led_data *aw9120_find_led_data(struct aw9120_priv *aw9120, u8 channel)
{
	size_t i;

	for (i = 0; i < aw9120->num_leds; i++) {
		if (aw9120->leds[i].channel == channel){
			printk("[INFO] aw9120_led_data= %d\r\n", channel);
			return &aw9120->leds[i];
		}

	}

	return NULL;
}

static int aw9120_parse_child_dt(const struct device *dev, const struct device_node *child, struct aw9120_led_data *led_data)
{
	struct led_classdev *cdev = &led_data->cdev;
	int ret = 0;
	u32 channel;
	u32 mc;

	if (of_property_read_string(child, "label", &cdev->name))
		cdev->name = child->name;

	ret = of_property_read_u32(child, "channel", &channel);
	if (ret || channel < 0 || channel > LEDS_20) {
		dev_err(dev,
				"Child node %pOF does not have a valid channel property\n",
				child);
		return -EINVAL;
	}
	led_data->channel = channel;

	ret = of_property_read_u32(child, "mc", &mc);
	if (mc > 7 || mc < 0) {
		dev_err(dev,
				"Child node %pOF does not have a valid mc property\n",
				child);
		return -EINVAL;
	}
	led_data->maxcurrent = mc;

	of_property_read_string(child, "linux,default-trigger",
			&cdev->default_trigger);

	cdev->brightness_set_blocking = aw9120_set_brightness;

	return 0;
}

static int aw9120_parse_dt(struct device *dev, struct aw9120_priv *aw9120)
{
	struct device_node *child;
	int ret = 0;

	for_each_child_of_node(dev->of_node, child) {
		struct aw9120_led_data *led_data =
			&aw9120->leds[aw9120->num_leds];
		const struct aw9120_led_data *other_led_data;

		led_data->priv = aw9120;

		ret = aw9120_parse_child_dt(dev, child, led_data);
		if (ret)
			goto err;

		/* Detect if channel is already in use by another child */

		other_led_data = aw9120_find_led_data(aw9120,
				led_data->channel);
		if (other_led_data) {
			dev_err(dev,
					"%s and %s both attempting to use channel %d\n",
					led_data->cdev.name,
					other_led_data->cdev.name,
					led_data->channel);
			goto err;
		}

		ret = devm_led_classdev_register(dev, &led_data->cdev);
		if (ret) {
			dev_err(dev, "failed to register PWM led for %s: %d\n",
					led_data->cdev.name, ret);
			goto err;
		}

		ret = sysfs_create_group(&led_data->cdev.dev->kobj,
				&led_attribute_group);

		aw9120->num_leds++;
	}
	// create node at /sys/class/i2c-dev/i2c-2/device/2-002d/
	ret = sysfs_create_group(&dev->kobj,
			&aw9120_attribute_group);

	return 0;

err:
	of_node_put(child);
	return ret;
}

static const struct of_device_id of_aw9120_match[] = {
	//{ .compatible = "awinic,aw9120-leds", .data = &aw9120_cdef, },
	{ .compatible = "awinic,aw9120-leds"},
	{},
};

MODULE_DEVICE_TABLE(of, of_aw9120_match);

static int aw9120_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//	const struct aw9120_chipdef *cdef;
	const struct of_device_id *of_dev_id;
	struct device *dev = &client->dev;
	struct aw9120_priv *aw9120;
	int count;
	int ret = 0;
	struct pinctrl *aw9120_en_pin_ctrl;
	struct pinctrl_state *aw9120_en_rst_on, *aw9120_en_rst_off;

	of_dev_id = of_match_device(of_aw9120_match, dev);
	if (!of_dev_id)
		return -EINVAL;

	//cdef = of_dev_id->data;

	count = of_get_child_count(dev->of_node);
	if (!count)
		return -EINVAL;

	aw9120 = devm_kzalloc(dev, sizeof_aw9120_priv(count),
			GFP_KERNEL);
	if (!aw9120)
		return -ENOMEM;

	//aw9120->cdef = cdef;
	aw9120->client = client;
	i2c_set_clientdata(client, aw9120);

	ret = aw9120_parse_dt(dev, aw9120);
	if (ret)
		return ret;

#if 1 // pinctrl oe1 enable sdp

	aw9120_en_pin_ctrl = devm_pinctrl_get(dev); /*Resource managed pinctrl_get()*/

	if (IS_ERR(aw9120_en_pin_ctrl)) {
		printk("aw9120_en_pin_ctrl failed\n");
		ret = PTR_ERR(aw9120_en_pin_ctrl);
		return ret;
	}


	aw9120_en_rst_on = pinctrl_lookup_state(aw9120_en_pin_ctrl, "aw9120-enabled");

	if (IS_ERR(aw9120_en_rst_on)) {
		printk("aw9120_en_rst_on failed\n");
		ret = PTR_ERR(aw9120_en_rst_on);
		return ret;
	}

	aw9120_en_rst_off = pinctrl_lookup_state(aw9120_en_pin_ctrl, "aw9120-disabled");
	if (IS_ERR(aw9120_en_rst_off)) {
		printk("aw9120_en_rst_off failed\n");
		ret = PTR_ERR(aw9120_en_rst_off);
		return ret;
	}


	pinctrl_select_state(aw9120_en_pin_ctrl, aw9120_en_rst_off);/*gpio low*/
	udelay(500);
	pinctrl_select_state(aw9120_en_pin_ctrl, aw9120_en_rst_on);/*gpio high*/
	udelay(500);

	devm_pinctrl_put(aw9120_en_pin_ctrl);
	mutex_init(&aw9120->lock);
#endif
	ret = aw9120_init(aw9120);
	if (ret)
		goto free_mutex;

	return SUCCESS;
free_mutex:
	mutex_destroy(&aw9120->lock);
	return ret;
}

static int aw9120_remove(struct i2c_client *client)
{

	struct aw9120_priv *aw9120 = i2c_get_clientdata(client);
	struct pinctrl *aw9120_en_pin_ctrl;
	struct pinctrl_state *aw9120_en_rst_off;
	int ret = 0;	

	mutex_destroy(&aw9120->lock);
	aw9120_en_pin_ctrl = devm_pinctrl_get(&client->dev); /*Resource managed pinctrl_get()*/
	if (IS_ERR(aw9120_en_pin_ctrl)) {
		printk("remove failed\n");
		ret = PTR_ERR(aw9120_en_pin_ctrl);
		return ret;
	}

	aw9120_en_rst_off = pinctrl_lookup_state(aw9120_en_pin_ctrl, "aw9120-disabled");
	if (IS_ERR(aw9120_en_rst_off)) {
		printk("remove . failed\n");
		ret = PTR_ERR(aw9120_en_rst_off);
		return ret;
	}

	pinctrl_select_state(aw9120_en_pin_ctrl, aw9120_en_rst_off);/*gpio low*/
	devm_pinctrl_put(aw9120_en_pin_ctrl);	

	sysfs_remove_group(&client->dev.kobj, &aw9120_attribute_group);

	return aw9120_software_reset(aw9120);
}

static struct i2c_driver aw9120_driver = {
	.driver = {
		.name	= "aw9120-leds",
		.of_match_table = of_aw9120_match,
	},
	.probe		= aw9120_probe,
	.remove		= aw9120_remove,
};

module_i2c_driver(aw9120_driver);

MODULE_AUTHOR("Zack Li <zack.li@tymphany.com>");
MODULE_DESCRIPTION("AW9120 LED driver");
MODULE_LICENSE("GPL v2");

