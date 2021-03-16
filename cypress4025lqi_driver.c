#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/extcon.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/workqueue.h>


#define CYPRESS4025LQI_IIC_REG          0x10
#define CYPRESS4025LQI_IIC_READLEN      6
#define KEY_CODE_HIGH_BIT               4
#define KEY_CODE_LOW_BIT                5
#define KEY_VALUE_BIT                   1

#define DEBUG_FLAG 1

struct cypress4025lqi_dev
{
    struct i2c_client client;
    int irq_pin;
    int cypress_addr;
    void *private_data;
    struct input_dev *input;
    int buttons_nunber;
    u32 *linux_code;
    unsigned int irq;
};


/*
 * @description     : reset cypress4025lqi
 * @param - client  : which i2c adapter
 * @param - dev     : cypress4025lqi_dev
 * @return          : 0 is succeed，other is lose
 */
static int cypress4025lqi_reset(struct i2c_client *client)
{
    struct pinctrl *reset_pin_ctrl;
	struct pinctrl_state *enabled_reset, *disabled_reset;
    int ret = 0;    

	reset_pin_ctrl = devm_pinctrl_get(&client->dev); /*Resource managed pinctrl_get()*/    
    if (IS_ERR(reset_pin_ctrl)) {
        printk("cypress reset_pin_ctrl failed\n");
        ret = PTR_ERR(reset_pin_ctrl);
        return ret;
    }

    enabled_reset = pinctrl_lookup_state(reset_pin_ctrl, "cypress-enabled");
    if (IS_ERR(enabled_reset)) {
        printk("cypress enabled_reset failed\n");
        ret = PTR_ERR(enabled_reset);
        return ret;
    }

    disabled_reset = pinctrl_lookup_state(reset_pin_ctrl, "cypress-disabled");
    if (IS_ERR(disabled_reset)) {
        printk("cypress disabled_reset failed\n");
        ret = PTR_ERR(disabled_reset);
        return ret;
    }

	pinctrl_select_state(reset_pin_ctrl, enabled_reset);
    udelay(500);
	pinctrl_select_state(reset_pin_ctrl, enabled_reset);
    udelay(500);

    devm_pinctrl_put(reset_pin_ctrl);

    return 0;
}

static int cypress4025lqi_enable(struct i2c_client *client)
{
    struct pinctrl *reset_pin_ctrl;
    struct pinctrl_state *enabled_reset;
    int ret = 0;    

    reset_pin_ctrl = devm_pinctrl_get(&client->dev); /*Resource managed pinctrl_get()*/
    if (IS_ERR(reset_pin_ctrl)) {
        printk("cypress reset_pin_ctrl failed\n");
        ret = PTR_ERR(reset_pin_ctrl);
        return ret;
    }

    enabled_reset = pinctrl_lookup_state(reset_pin_ctrl, "cypress-enabled");
    if (IS_ERR(enabled_reset)) {
        printk("cypress enabled_reset failed\n");
        ret = PTR_ERR(enabled_reset);
        return ret;
    }


    pinctrl_select_state(reset_pin_ctrl, enabled_reset);
    udelay(500);

    devm_pinctrl_put(reset_pin_ctrl);

    return 0;
}

/*
 * @description : read the iic reg from cypress4025lqi
 * @param - dev:  cypress4025lqi device
 * @param - reg:  which reg addr
 * @param - val:  read back value
 * @param - len:  read lenth
 * @return      : result
 */
static int cypress4025lqi_read_regs(struct cypress4025lqi_dev *dev, u8 reg, void *val, int len)
{
    int ret;
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client *) &dev->client;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = 1;


    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = val;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if(ret == 2)
    {
        ret = 0;
    }
    else
    {
        ret = -EREMOTEIO;
    }
    return ret;
}

/*
 * @description : write iic reg into cypress4025lqi
 * @param - dev:  cypress4025lqi device
 * @param - reg:  write reg
 * @param - val:  write data buffer
 * @param - len:  write data lenth
 * @return    :   return result
 */
static s32 cypress4025lqi_write_regs(struct cypress4025lqi_dev *dev, u8 reg, u8 *buf, u8 len)
{
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client *)&dev->client;

    b[0] = reg;
    memcpy(&b[1],buf,len);

    msg.addr = client->addr;
    msg.flags = 0;

    msg.buf = b;
    msg.len = len + 1;

    return i2c_transfer(client->adapter, &msg, 1);
}

/*
 * @description     : cypress4025lqi irq thread function
 * @param - irq     : irq number
 * @param - dev_id  : device struc
 * @return          : return result
 */
static irqreturn_t cypress4025lqi_handler(int irq, void *dev_id)
{
    struct cypress4025lqi_dev *multidata = dev_id;
    u8 CYPRESS_IIC_BUF[CYPRESS4025LQI_IIC_READLEN];
    int ret;

    ret = cypress4025lqi_read_regs(multidata, CYPRESS4025LQI_IIC_REG, CYPRESS_IIC_BUF, CYPRESS4025LQI_IIC_READLEN);

    ret = (CYPRESS_IIC_BUF[KEY_CODE_HIGH_BIT]<<8)+CYPRESS_IIC_BUF[KEY_CODE_LOW_BIT];
#if DEBUG_FLAG
    printk(" KEY_VALUE_BIT= %d  KEY_CODE_BIT= %d  \n", CYPRESS_IIC_BUF[KEY_VALUE_BIT], ret);
#endif

    input_report_key(multidata->input, ret, CYPRESS_IIC_BUF[KEY_VALUE_BIT]);//key

    input_sync(multidata->input);

    return IRQ_HANDLED;

}

/*
 * @description                : cypress4025lqi irq init
 * @param - client             : the i2c adapter
 * @param - cypress4025lqi_dev : cypress4025lqi_dev device
 * @return                     : 0 is succeed,other is lose
 */
static int cypress4025lqi_irq(struct i2c_client *client, struct cypress4025lqi_dev *dev)
{

    int ret = 0;
    if (gpio_is_valid(dev->irq_pin)){

        ret = devm_gpio_request_one(&client->dev, dev->irq_pin,GPIOF_IN, "interrupt-gpios");
        if (ret) {
            dev_err(&client->dev,
                        "Failed to request GPIO %d, error %d\n",
                        dev->irq_pin, ret);
            return ret;
        }
    }

    ret = devm_request_threaded_irq(&client->dev, dev->irq, NULL,
                                        cypress4025lqi_handler,
                                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                        "interrupt-gpios", dev);
    if (ret) {
        dev_err(&client->dev, "Unable to request cypress IRQ.\n");
        return ret;
    }

    return 0;
}

/*
 * @description     : i2c driver probe function
 *                    when the driver probe succeed,this function will be run
 * @param - client  : i2c device
 * @param - id      : i2c device id
 * @return          : 0 is succeed,other is fail
 */
static int cypress4025lqi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct fwnode_handle *child;
    struct cypress4025lqi_dev *cypress4025lqi = container_of(client, struct cypress4025lqi_dev, client);
    u32 linux_code = 0;
    int node_buttons;
    int button = 0;

    cypress4025lqi->irq_pin = of_get_named_gpio(client->dev.of_node, "interrupt-gpios", 0);

    node_buttons = device_get_child_node_count(&client->dev);
    if (node_buttons == 0)
        return ERR_PTR(-ENODEV);

    cypress4025lqi->linux_code = devm_kzalloc(&client->dev,node_buttons * sizeof(*cypress4025lqi->linux_code), GFP_KERNEL);
    cypress4025lqi->buttons_nunber = node_buttons;


    ret = cypress4025lqi_reset(client);
    if(ret < 0)
    {
        printk("cypress reset failed\n");
        goto fail;
    } 

    ret = cypress4025lqi_enable(client);
    if(ret < 0)
    {
        printk("cypress enable failed\n");
        goto fail;
    }

    cypress4025lqi->irq = gpio_to_irq(cypress4025lqi->irq_pin);
    ret = cypress4025lqi_irq(client, cypress4025lqi);
    if(ret < 0)
    {
        goto fail;
    }

    cypress4025lqi->input = devm_input_allocate_device(&client->dev);
    if (!cypress4025lqi->input)
    {
        ret = -ENOMEM;
        goto fail;
    }

    cypress4025lqi->input->name = client->name;
    cypress4025lqi->input->id.bustype = BUS_I2C;
    cypress4025lqi->input->dev.parent = &client->dev;


    __set_bit(EV_KEY, cypress4025lqi->input->evbit);//key

    device_for_each_child_node(&client->dev, child)
    {

        if (fwnode_property_read_u32(child, "linux,code", &linux_code))
        {
            dev_err(&(client->dev), "Button without keycode\n");
            fwnode_handle_put(child);
            return ERR_PTR(-EINVAL);
        }
        cypress4025lqi->linux_code[button] = linux_code;

        __set_bit(linux_code,  cypress4025lqi->input->keybit);//key

        button ++;
    }

    device_init_wakeup(&cypress4025lqi->client.dev, device_property_read_bool(&cypress4025lqi->client.dev, "wakeup-source"));

    ret = input_register_device(cypress4025lqi->input);
    if (ret)
        goto fail;

#if DEBUG_FLAG
    printk("probe ok, button= %d\n",button);
#endif

    return 0;

fail:
    return ret;
}

/*
 * @description     : i2c driver remove function，when remod the dirver,this function will run
 * @param - client  : i2c device
 * @return          : 0 is succeed,other is fail
 */
static int cypress4025lqi_remove(struct i2c_client *client)
{
    struct cypress4025lqi_dev *cypress4025lqi = container_of(client, struct cypress4025lqi_dev, client);
    input_unregister_device(cypress4025lqi->input);
    return 0;
}

static int __maybe_unused cypress4025lqi_pm_suspend(struct device *dev)
{

    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct cypress4025lqi_dev *cypress4025lqi = container_of(client, struct cypress4025lqi_dev, client);
    if (device_may_wakeup(dev)) {
        enable_irq_wake(cypress4025lqi->irq);
    }
    return 0;
}

static int __maybe_unused cypress4025lqi_pm_resume(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct cypress4025lqi_dev *cypress4025lqi = container_of(client, struct cypress4025lqi_dev, client);
    if (device_may_wakeup(dev)) {
        disable_irq_wake(cypress4025lqi->irq);
    }

    return 0;
}

static const struct dev_pm_ops cypress4025lqi_pm_ops = {
//        SET_RUNTIME_PM_OPS(cypress4025lqi_runtime_suspend, cypress4025lqi_runtime_resume,NULL)
    SET_SYSTEM_SLEEP_PM_OPS(cypress4025lqi_pm_suspend, cypress4025lqi_pm_resume)
};

static const struct i2c_device_id cypress4025lqi_id[] =
{
    { "cypress4025lqi", 0, },
    { /* sentinel */ }
};


static const struct of_device_id cypress4025lqi_of_match[] =
{
    { .compatible = "cypress,cypress-wedge", },
    { .compatible = "cypress,cypress-zeppelin", },
    { .compatible = "cypress,cypresslqi4024", },
    { .compatible = "cypress,cypress-md", },
    { /* sentinel */ }
};

static struct i2c_driver cypress4025lqi_driver =
{
    .driver = {
        .owner = THIS_MODULE,
        .name = "cypress-cypress4025lqi",
        .pm = &cypress4025lqi_pm_ops,
        .of_match_table = of_match_ptr(cypress4025lqi_of_match),
    },
    .id_table = cypress4025lqi_id,
    .probe    = cypress4025lqi_probe,
    .remove   = cypress4025lqi_remove,
};

module_i2c_driver(cypress4025lqi_driver);

MODULE_AUTHOR("Zack Li <Zack.Li@tymphany.com>");
MODULE_DESCRIPTION("Cypress4025lqi I2C Input Driver");
MODULE_LICENSE("GPL");
