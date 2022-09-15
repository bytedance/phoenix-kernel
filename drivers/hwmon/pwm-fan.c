/*
 * pwm-fan.c - Hwmon driver for fans connected to PWM lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
/*added by webber.wang for bug 54475 :fan debug--start*/
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
/*added by webber.wang for bug 54475 :fan debug--end*/

#define MAX_PWM 40 //modified by webber.wang for bug 54475 :fan debug

struct pwm_fan_ctx {
	struct mutex lock;
	struct pwm_device *pwm;
	unsigned int pwm_value;
	unsigned int pwm_fan_state;
	unsigned int pwm_fan_max_state;
	unsigned int *pwm_fan_cooling_levels;
	struct thermal_cooling_device *cdev;
    /*added by webber.wang for bug 54475 :fan debug--start*/
    int             fan_pwr_enable_gpio;
    int 		    fan_pwm_gpio;
	unsigned int 	fan_pwm_irq;
	int             fan_pwm_count;
	ktime_t         start_count_time;
	ktime_t         end_count_time;
	bool 			irq_active;
    bool            pwm_enable;
    unsigned long   resume_pwm;
    /*added by webber.wang for bug 54475 :fan debug--end*/
};

static int  __set_pwm(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	unsigned long period;
	int ret = 0;
	struct pwm_state state = { };

	mutex_lock(&ctx->lock);
	if (ctx->pwm_value == pwm)
		goto exit_set_pwm_err;

	pwm_init_state(ctx->pwm, &state);
	period = ctx->pwm->args.period;
	state.duty_cycle = DIV_ROUND_UP((40 - pwm) * (period - 1), MAX_PWM);
	state.enabled = true;

	ret = pwm_apply_state(ctx->pwm, &state);
    ctx->pwm_value = pwm;
	if (!ret)
		ctx->pwm_value = pwm;
exit_set_pwm_err:
	mutex_unlock(&ctx->lock);
	return ret;
}

static void pwm_fan_update_state(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	int i;

	for (i = 0; i < ctx->pwm_fan_max_state; ++i)
		if (pwm < ctx->pwm_fan_cooling_levels[i + 1])
			break;

	ctx->pwm_fan_state = i;
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_PWM)
		return -EINVAL;

	/*Added by gavin.zhao for bug_59468 modify pwm freq to 50KHZ ---begin */
	if(pwm == MAX_PWM){
		pwm = pwm - 1;
	}
	/*Added by gavin.zhao for bug_59468 modify pwm freq to 50KHZ ---end */

	ret = __set_pwm(ctx, pwm);
	if (ret)
		return ret;

	pwm_fan_update_state(ctx, pwm);
	return count;
}
static ssize_t show_pwm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->pwm_value);
}
/*added by webber.wang for bug 54475 :fan debug--start*/
static ssize_t show_pwm_rpm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int fan_pwm_rpm = 0;
	struct pwm_fan_ctx *fan_data = dev_get_drvdata(dev);
        ktime_t   elapsed;
	unsigned int elapsed_msecs = 0;

	if(fan_data->fan_pwm_count == 100){
		elapsed = ktime_sub(fan_data->end_count_time, fan_data->start_count_time);
		elapsed_msecs = ktime_to_ms(elapsed);
		fan_pwm_rpm = (30 * 1000 * 99)/elapsed_msecs;
		pr_err("show_pwm fan_pwm_rpm = %d\n", fan_pwm_rpm);
	}else {
		fan_pwm_rpm = 0;
	}

	return sprintf(buf, "%d\n", fan_pwm_rpm);
}
static ssize_t set_pwm_rpm(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{	int ret = count;
	struct pwm_fan_ctx *fan_data = dev_get_drvdata(dev);
	unsigned long set_pwm_begin;

	if (kstrtoul(buf, 10, &set_pwm_begin) || set_pwm_begin > 1)
		return -EINVAL;
	if(set_pwm_begin == 1){
		fan_data->fan_pwm_count = 0;
		if(fan_data->irq_active == 0){
			enable_irq(fan_data->fan_pwm_irq);
			fan_data->irq_active = true;
		}
	}
	return ret;
}

static ssize_t set_pwm_enable(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
    struct pwm_fan_ctx *fan_data = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) || val > 1)
		return -EINVAL;
	gpio_set_value_cansleep(fan_data->fan_pwr_enable_gpio, val);
    fan_data->pwm_enable = (bool)val;
    return count;
}

static ssize_t show_pwm_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct pwm_fan_ctx *fan_data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", fan_data->pwm_enable);
}
/*added by webber.wang for bug 54475 :fan debug--end*/
static DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm);
static DEVICE_ATTR(pwm1_enable, S_IRUGO | S_IWUSR,show_pwm_enable, set_pwm_enable);
static DEVICE_ATTR(pwm_rpm, S_IRUGO | S_IWUSR, show_pwm_rpm, set_pwm_rpm);


static struct attribute *pwm_fan_attrs[] = {
	&dev_attr_pwm1.attr,
    &dev_attr_pwm1_enable.attr,
    &dev_attr_pwm_rpm.attr,
	NULL,
};

ATTRIBUTE_GROUPS(pwm_fan);

/* thermal cooling device callbacks */
static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_max_state;

	return 0;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_state;

	return 0;
}

static int
pwm_fan_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	int ret;

	if (!ctx || (state > ctx->pwm_fan_max_state))
		return -EINVAL;

	if (state == ctx->pwm_fan_state)
		return 0;

	ret = __set_pwm(ctx, ctx->pwm_fan_cooling_levels[state]);
	if (ret) {
		dev_err(&cdev->device, "Cannot set pwm!\n");
		return ret;
	}

	ctx->pwm_fan_state = state;

	return ret;
}

static const struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
};
/*deleted by webber.wang for bug 54475 :fan debug--start*/
/*
static int pwm_fan_of_get_cooling_data(struct device *dev,
				       struct pwm_fan_ctx *ctx)
{
	struct device_node *np = dev->of_node;
	int num, i, ret;

	if (!of_find_property(np, "cooling-levels", NULL))
		return 0;

	ret = of_property_count_u32_elems(np, "cooling-levels");
	if (ret <= 0) {
		dev_err(dev, "Wrong data!\n");
		return ret ? : -EINVAL;
	}

	num = ret;
	ctx->pwm_fan_cooling_levels = devm_kcalloc(dev, num, sizeof(u32),
						   GFP_KERNEL);
	if (!ctx->pwm_fan_cooling_levels)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "cooling-levels",
					 ctx->pwm_fan_cooling_levels, num);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		if (ctx->pwm_fan_cooling_levels[i] > MAX_PWM) {
			dev_err(dev, "PWM fan state[%d]:%d > %d\n", i,
				ctx->pwm_fan_cooling_levels[i], MAX_PWM);
			return -EINVAL;
		}
	}

	ctx->pwm_fan_max_state = num - 1;

	return 0;
}
*/
/*deleted by webber.wang for bug 54475 :fan debug--end*/
/*added by webber.wang for bug 54475 :fan debug--start*/

static irqreturn_t fan_pwm_interrupt(int irq, void *data)
{
	struct pwm_fan_ctx *fan_data = data;

	fan_data->fan_pwm_count ++;
	if(fan_data->fan_pwm_count == 1){
		fan_data->start_count_time = ktime_get_boottime();
	}
	if(fan_data->fan_pwm_count == 100){
		fan_data->end_count_time = ktime_get_boottime();
		disable_irq_nosync(fan_data->fan_pwm_irq);
		fan_data->irq_active = false;
	}
    return IRQ_HANDLED;
}
/*added by webber.wang for bug 54475 :fan debug--end*/
/*modified by webber.wang for bug 54475 :fan debug--start*/
static int pwm_fan_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct pwm_fan_ctx *ctx;
	struct device *hwmon;
	int ret;
	struct pwm_state state = { };
    struct device_node *node = pdev->dev.of_node;
	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mutex_init(&ctx->lock);
    ctx->pwm = devm_of_pwm_get(&pdev->dev,node,NULL);
	if (IS_ERR(ctx->pwm)) {
		dev_err(&pdev->dev, "Could not get PWM\n");
		return PTR_ERR(ctx->pwm);
	}

    ctx->fan_pwr_enable_gpio = of_get_gpio(node,0);
    if(ctx->fan_pwr_enable_gpio){
		ret = gpio_request(ctx->fan_pwr_enable_gpio,"pwm_pwr_enable");
        if(ret < 0)
        {
            pr_err("pwm_fan request gpio %d failed!\n",ctx->fan_pwr_enable_gpio);
            goto err_pwm_disable;
        }
		ret = gpio_direction_output(ctx->fan_pwr_enable_gpio,0);
        if(ret < 0)
        {
            pr_err("pwm_fan set value gpio %d failed!\n",ctx->fan_pwr_enable_gpio);
            goto err_pwm_disable;
        }
	}

    if((ctx->fan_pwm_gpio = of_get_named_gpio(node, "fan_pwm_irq", 0)) < 0){
        pr_err("of_get_named_gpio fan_pwm_irqo error!\n");
        return ctx->fan_pwm_gpio;
    }

	platform_set_drvdata(pdev, ctx);
	ctx->pwm_value = MAX_PWM;
    /* Set duty cycle to maximum allowed and enable PWM output */

	pwm_init_state(ctx->pwm, &state);
	state.duty_cycle = 0;
    state.period =  40;
	state.enabled = false;
	state.output_type = PWM_OUTPUT_FIXED;
	/* Use default pattern in PWM device */
	state.output_pattern = NULL;
	ret = pwm_apply_state(ctx->pwm, &state);
	if (ret) {
		dev_err(&pdev->dev, "Failed to configure PWM\n");
		return ret;
	}

	hwmon = devm_hwmon_device_register_with_groups(&pdev->dev, "pwmfan",
						       ctx, pwm_fan_groups);
	if (IS_ERR(hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device\n");
		ret = PTR_ERR(hwmon);
		goto err_pwm_disable;
	}

	//ret = pwm_fan_of_get_cooling_data(&pdev->dev, ctx);
	if (ret)
		goto err_pwm_disable;

	ctx->pwm_fan_state = ctx->pwm_fan_max_state;
	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = thermal_of_cooling_device_register(pdev->dev.of_node,
							  "pwm-fan", ctx,
							  &pwm_fan_cooling_ops);
		if (IS_ERR(cdev)) {
			dev_err(&pdev->dev,
				"Failed to register pwm-fan as cooling device");
			ret = PTR_ERR(cdev);
			goto err_pwm_disable;
		}
		ctx->cdev = cdev;
		thermal_cdev_update(cdev);
	}
    if(ctx->fan_pwm_gpio){
		gpio_request(ctx->fan_pwm_gpio, "fan pwm gpio");
		gpio_direction_input(ctx->fan_pwm_gpio);

		ctx->fan_pwm_irq = gpio_to_irq(ctx->fan_pwm_gpio);
		if((ret = request_irq(ctx->fan_pwm_irq, fan_pwm_interrupt, IRQF_TRIGGER_RISING, "fan_pwm irq", ctx)) < 0)
		{
			pr_err("request fan pwm irq error\n");
		}
		disable_irq(ctx->fan_pwm_irq);
		ctx->irq_active = false;
	}
	return 0;

err_pwm_disable:
	state.enabled = false;
	pwm_apply_state(ctx->pwm, &state);

	return ret;
}
/*modified by webber.wang for bug 54475 :fan debug--end*/
static int pwm_fan_remove(struct platform_device *pdev)
{
	struct pwm_fan_ctx *ctx = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(ctx->cdev);
	if (ctx->pwm_value)
		pwm_disable(ctx->pwm);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_fan_suspend(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

    ctx->resume_pwm = ctx->pwm_value;
    __set_pwm(ctx,40);

    gpio_direction_output(ctx->fan_pwr_enable_gpio,0);
	return 0;
}

static int pwm_fan_resume(struct device *dev)
{
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_fan_pm, pwm_fan_suspend, pwm_fan_resume);

static const struct of_device_id of_pwm_fan_match[] = {
	{ .compatible = "pwm-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_fan_match);

static struct platform_driver pwm_fan_driver = {
	.probe		= pwm_fan_probe,
	.remove		= pwm_fan_remove,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= &pwm_fan_pm,
		.of_match_table	= of_pwm_fan_match,
	},
};

module_platform_driver(pwm_fan_driver);

MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
