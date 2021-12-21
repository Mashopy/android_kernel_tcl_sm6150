/* Copyright (C) 2019 Tcl Corporation Limited */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_device.h>
/* MODIFIED-BEGIN by hongwei.tian, 2019-11-06,BUG-8315980*/
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
/* MODIFIED-END by hongwei.tian,BUG-8315980*/

#define DEVICE_NAME		"kookong"
#define DRIVER_NAME		"kookong-ir-tx"

#define NS_IN_1HZ				(1000000000UL)
#define IR_PWM_ID			0

static const struct of_device_id pwm_ir_of_match[] = {
	{ .compatible = "kookong-ir-tx", },
	{ },
};

MODULE_DEVICE_TABLE(of, pwm_ir_of_match); // MODIFIED by hongwei.tian, 2019-11-06,BUG-8315980

struct pwm_ir {
	struct pwm_device *pwm;
	unsigned int carrier;
	unsigned int duty_cycle;
};

struct pwm_ir  *kookong_pwm_ir;


static int pwm_set_freq(unsigned long freq) {

	int duty, period;

	period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, kookong_pwm_ir->carrier);
	duty = DIV_ROUND_CLOSEST(kookong_pwm_ir->duty_cycle * period, 100);

	pwm_config(kookong_pwm_ir->pwm, duty, period);

	return 0;
}

static int kookong_pwm_open(struct inode *inode, struct file *file) {
	return 0;
}

static int kookong_pwm_close(struct inode *inode, struct file *file) {
	return 0;
}


static int pwm_ir_tx(unsigned int *txbuf, unsigned int count)
{
	/* MODIFIED-BEGIN by hongwei.tian, 2019-11-06,BUG-8315980*/
	struct pwm_device *pwm = kookong_pwm_ir->pwm;
	int i, duty, period ,ret;
	ktime_t edge;
	long delta;


	period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, kookong_pwm_ir->carrier);
	duty = DIV_ROUND_CLOSEST(kookong_pwm_ir->duty_cycle * period, 100);

	ret = pwm_config(pwm, duty, period);
	/* MODIFIED-END by hongwei.tian,BUG-8315980*/

	edge = ktime_get();

	for (i = 0; i < count; i++) {
		if (i % 2) // space
			pwm_disable(pwm);
		else
			pwm_enable(pwm);

		edge = ktime_add_us(edge, txbuf[i]);
		delta = ktime_us_delta(edge, ktime_get());
		if (delta > 0)
		/* MODIFIED-BEGIN by hongwei.tian, 2019-10-31,BUG-8315980*/
		{
			//usleep_range(delta, delta + 10);
			udelay(delta);
		}
		/* MODIFIED-END by hongwei.tian,BUG-8315980*/

	}

	pwm_disable(pwm);

	return count;
}

static ssize_t kookong_pwm_write(struct file *filp,const char __user *buf_user,size_t size,loff_t * offp) {
	int ret,i;
	static int buf[1024]={0};
	int count=0;



	ret=copy_from_user(buf,buf_user,size);
	if(ret!=0) {
		return -EINVAL;
	}
	count=size/(sizeof(int))-1;
	pr_info(DEVICE_NAME" count = %d\n",count);

	/* MODIFIED-BEGIN by hongwei.tian, 2019-11-06,BUG-8315980*/
	kookong_pwm_ir->carrier = (unsigned long)buf[0];

	pr_info(DEVICE_NAME" carrier = %d\n",kookong_pwm_ir->carrier);
	/* MODIFIED-END by hongwei.tian,BUG-8315980*/

	for(i=0;i<count;i++)
		pr_debug(DEVICE_NAME" data[%d] = %d\n",i,buf[i+1]);

	ret = pwm_ir_tx(&buf[1],count);

	return ret;
}

static long kookong_pwm_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
		case 2:
//		s3c_gpio_cfgpin(PMW_GPIO, S3C_GPIO_OUTPUT);
//		gpio_set_value(PMW_GPIO, 1);
			break;
		case 3:
//		s3c_gpio_cfgpin(PMW_GPIO, S3C_GPIO_OUTPUT);
//		gpio_set_value(PMW_GPIO, 0);
			break;
		case 4:
//		s3c_gpio_cfgpin(PMW_GPIO, S3C_GPIO_SFN(2));
		pwm_config(kookong_pwm_ir->pwm, (NS_IN_1HZ / 38000) / 2, NS_IN_1HZ / 38000);
		pwm_enable(kookong_pwm_ir->pwm);
			break;
		case 5:
//		s3c_gpio_cfgpin(PMW_GPIO, S3C_GPIO_SFN(2));
		pwm_config(kookong_pwm_ir->pwm, 0, NS_IN_1HZ / 100);
		pwm_disable(kookong_pwm_ir->pwm);
			break;

		case 1:
			if (arg == 0)
				return -EINVAL;
			pwm_set_freq(arg);
			break;

		case 0:
		default:
			pwm_disable(kookong_pwm_ir->pwm);
			break;
	}

	return 8;
}


static struct file_operations kookong_pwm_ops = {
	.owner			= THIS_MODULE,
	.open			= kookong_pwm_open,
	.release		= kookong_pwm_close,
	.write			= kookong_pwm_write,
	.unlocked_ioctl		= kookong_pwm_ioctl,
};

static struct miscdevice kookong_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &kookong_pwm_ops,
};


static int pwm_ir_probe(struct platform_device *pdev)
{
	int rc;

	kookong_pwm_ir = devm_kmalloc(&pdev->dev, sizeof(*kookong_pwm_ir), GFP_KERNEL);
	if (!kookong_pwm_ir)
		return -ENOMEM;

	kookong_pwm_ir->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(kookong_pwm_ir->pwm))
		return PTR_ERR(kookong_pwm_ir->pwm);

	kookong_pwm_ir->carrier = 38000;
	kookong_pwm_ir->duty_cycle = 50;

	rc = misc_register(&kookong_misc_dev);

	pr_info(DEVICE_NAME"\tinitialized\n"); // MODIFIED by hongwei.tian, 2019-11-06,BUG-8315980
	return rc;
}


static struct platform_driver pwm_ir_driver = {
	.probe = pwm_ir_probe,
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(pwm_ir_of_match),
	},
};
module_platform_driver(pwm_ir_driver);

MODULE_DESCRIPTION("PWM IR Transmitter");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kookong Inc.");
