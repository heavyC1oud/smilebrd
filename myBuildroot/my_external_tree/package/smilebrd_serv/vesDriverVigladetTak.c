#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>           // miscellaneous character module
#include <linux/kernel.h>
#include <linux/of.h>                   // Device Tree (of - open firmware)
#include <linux/i2c.h>                  // i2c devices
#include <linux/uaccess.h>
#include <linux/platform_device.h>      // platform devices
#include <linux/gpio/consumer.h>        // GPIO descriptor
#include <linux/interrupt.h>            // IRQ
#include <linux/wait.h>

#define CMD_CONTROL_LED 0x10
#define CMD_CONTROL_LED_ARG_OFF 0x00
#define CMD_CONTROL_LED_ARG_ON 0x01

// private smilebrd structure
struct smilebrd_dev {
    struct i2c_client* i2c_dev;
    struct platform_device* gpio_dev;
    struct gpio_desc* button;
    struct gpio_desc* led;
    unsigned int irq;
    unsigned int irq_f;
    unsigned int tsc_data;

    struct miscdevice miscdevice;
    char name[8];
};

static struct smilebrd_dev* smilebrd;

static DECLARE_WAIT_QUEUE_HEAD(wq);

/**
 * @brief Device callback for device node ioctl
 */
static long smilebrd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    // now exist only SWITCH LED command
    switch(cmd) {
    case CMD_CONTROL_LED:
        if(arg == CMD_CONTROL_LED_ARG_ON) {
            gpiod_set_value(smilebrd->led, 1);
        }
        else {
            gpiod_set_value(smilebrd->led, 0);
        }
        break;
    default:
        break;
    }

    return 0;
}

/**
 * @brief Device callback for device node read
 */
static ssize_t smilebrd_read(struct file *filp, char __user *userbuf, size_t count, loff_t *ppos)
{
    // stopping the process until an interrupt is received
    wait_event_interruptible(wq, smilebrd->irq_f != 0);

    // interrupt received and handled
    smilebrd->irq_f = 0;

    // copy received device data from kernel space to user space
    if(copy_to_user(userbuf, &smilebrd->tsc_data, sizeof(smilebrd->tsc_data)) != 0) {
        return -EIO;
    }

    return sizeof(unsigned int);
}

/**
 * @brief Input GPIO interrupt handler
 */
static irqreturn_t smilebrd_gpio_irq_handler(int irq, void* dev_id)
{
    // send i2c data
    smilebrd->tsc_data = i2c_smbus_read_byte(smilebrd->i2c_dev);

    // set irq flag
    smilebrd->irq_f = 1;
    wake_up_interruptible(&wq);

    return IRQ_HANDLED;
}

/**
 * @brief File operations structure
 */
static const struct file_operations smilebrd_fops = {
    .owner = THIS_MODULE,
    .read = smilebrd_read,
    .unlocked_ioctl = smilebrd_ioctl,
};

/**
 * @brief Install GPIO
 */
static int smilebrd_gpio_probe(struct platform_device* pdev)
{
    int retval;
    smilebrd->gpio_dev = pdev;

    // input GPIO, check signal from smile board
    smilebrd->button = gpiod_get(&smilebrd->gpio_dev->dev, "button", 0);
    gpiod_direction_input(smilebrd->button);

    // output GPIO, controls "enable server" LED
    smilebrd->led = gpiod_get(&smilebrd->gpio_dev->dev, "led", 0);
    gpiod_direction_output(smilebrd->led, 0);

    // add debounce interval to input GPIO
    retval = gpiod_set_debounce(smilebrd->button, 1000 * 5);        // time unit 1 us, 1000 us * 5 = 5 ms, MAX = 7 ms
    if(retval != 0) {
        pr_err("could not set debounce interval\n");
    }

    // add interrupt to input GPIO
    smilebrd->irq = gpiod_to_irq(smilebrd->button);
    retval = request_threaded_irq(smilebrd->irq, NULL, smilebrd_gpio_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "smilepd_drv", NULL);
    if(retval != 0) {
        pr_err("could not register smilebrd irq handler\n");
        return retval;
    }

    pr_info("smilebrd gpio probed!\n");
    return 0;
}

/**
 * @brief Remove GPIO
 */
static int smilebrd_gpio_remove(struct platform_device* pdev)
{
    free_irq(smilebrd->irq, NULL);
    gpiod_put(smilebrd->button);
    gpiod_put(smilebrd->led);

    pr_info("smilebrd gpio remove\n");

    return 0;
}

/**
 * @brief Match driver data with device tree data
 */
static const struct of_device_id smilebrd_gpio_dt_ids[] = {
    { .compatible = "heavyc1oud,smilebrd_gpio", },
    {}
};
MODULE_DEVICE_TABLE(of, smilebrd_gpio_dt_ids);

static struct platform_driver smilebrd_gpio_drv = {
    .probe = smilebrd_gpio_probe,
    .remove = smilebrd_gpio_remove,
    .driver = {
        .name = "smilebrd_gpio",
        .of_match_table = of_match_ptr(smilebrd_gpio_dt_ids),
        .owner = THIS_MODULE,
    },
};

/**
 * @brief Install I2C
 */
static int smilebrd_i2c_probe(struct i2c_client* client, const struct i2c_device_id *id)
{
    // store pointer to device structure in the bus
    i2c_set_clientdata(client, smilebrd);

    // store pointer to I2C client into private structure
    smilebrd->i2c_dev = client;

    pr_info("smilebrd i2c probed!\n");

    return 0;
}

/**
 * @brief Remove I2C
 */
static int smilebrd_i2c_remove(struct i2c_client* client)
{
    // get device structure from device bus
    smilebrd = i2c_get_clientdata(client);

    pr_info("smilebrd i2c remove\n");

    return 0;
}

/**
 * @brief Match driver data with device tree data
 */
static const struct of_device_id smilebrd_i2c_dt_ids[] = {
    { .compatible = "heavyc1oud,smilebrd_i2c", },
    {}
};

static const struct i2c_device_id smilebrd_i2c_i2cbus_id[] ={
    {"smilebrd_i2c", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, smilebrd_i2c_i2cbus_id);

static struct i2c_driver smilebrd_i2c_drv = {
    .probe = smilebrd_i2c_probe,
    .remove = smilebrd_i2c_remove,
    .id_table = smilebrd_i2c_i2cbus_id,
    .driver = {
        .name = "smilebrd_i2c",
        .of_match_table = of_match_ptr(smilebrd_i2c_dt_ids),
        .owner = THIS_MODULE,
    },
};

/**
 * @brief Whole driver initialization
 */
static int __init smilebrd_init(void)
{
    // allocate mem for private structure
    smilebrd = kzalloc(sizeof(struct smilebrd_dev), GFP_KERNEL);

    // initialize misc device
    smilebrd->miscdevice.name = "smilebrd";
    smilebrd->miscdevice.minor = MISC_DYNAMIC_MINOR;
    smilebrd->miscdevice.fops = &smilebrd_fops;

    // register miscdevice
    if(misc_register(&smilebrd->miscdevice)) {
        pr_err("could not register smilebrd misc device\n");
        return EINVAL;
    }

    // register smilebrd gpio driver
    if(platform_driver_register(&smilebrd_gpio_drv)) {
        pr_err("could not register smilebrd gpio driver\n");
        return EINVAL;
    }

    // register smilebrd i2c driver
    if(i2c_register_driver(THIS_MODULE, &smilebrd_i2c_drv)) {
        pr_err("could not register smilebrd i2c driver\n");
        return EINVAL;
    }

    return 0;
}

/**
 * @brief Whole driver deinitialization
 */
static void __exit smilebrd_exit(void)
{
    // deregister miscdevice
    misc_deregister(&smilebrd->miscdevice);

    // deregister smilebrd gpio driver
    platform_driver_unregister(&smilebrd_gpio_drv);

    // deregister smilebrd i2c driver
    i2c_del_driver(&smilebrd_i2c_drv);

    // free mem previously allocated in smilebrd_init
    kfree(smilebrd);
}

module_init(smilebrd_init);
module_exit(smilebrd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HeavyC1oud vheavyC1oud@gmail.com");
MODULE_DESCRIPTION("smilebrd driver");
