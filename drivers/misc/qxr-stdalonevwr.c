// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <dt-bindings/regulator/qcom,rpmh-regulator.h>

#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/console.h>
#include <linux/pm_qos.h>
#include <linux/pm_wakeirq.h>
#include <linux/delay.h>

//#define QXR_SLEEP_POWER0FF_DISABLE	1
struct qxr_stdalonevwr {
	struct platform_device *pdev;
	struct regulator *reg_vdd;
	struct regulator *reg_vddio;
	int ndi_5v_en;
	bool initDone;
	struct pinctrl *pctrl;
	struct pinctrl_state *pins_enable;
	struct pinctrl_state *pins_disable;
};

static struct qxr_stdalonevwr *pdata;
static bool special_mode;
static int __init qxr_get_boot_mode(void)
{
	if (strstr(boot_command_line, "pvrboot.mode=recovery")){
		special_mode = true;
	    pr_info("%s boot mode: recovery mode\n",__func__);
	}else if(strstr(boot_command_line, "pvrboot.mode=charger")){
		special_mode = true;
	    pr_info("%s boot mode: charger mode\n",__func__);
	}else{
	    pr_info("%s boot mode: normal mode\n",__func__);
	}
	pr_info("%s boot mode: %d\n",__func__,special_mode);
	return 0;
}
static int qxr_stdalonevwr_allocate_res(void)
{
	int rc = -EINVAL;
	//bool gpioEnabled = false;
	pr_info("%s start\n", __func__);
	if (pdata->initDone) {
		pr_info("%s init is done already\n", __func__);
		return 0;
	}
	/* Invensense 3.3 PowerRail */
	pdata->reg_vdd = devm_regulator_get(&pdata->pdev->dev, "imu-vdd");
	if (IS_ERR(pdata->reg_vdd)) {
		pr_info("%s can not find vdd\n", __func__);
	}else{
		pr_info("%s vdd has been found in dts\n", __func__);
		rc = regulator_set_load(pdata->reg_vdd, 600000);
		if (rc < 0) {
			pr_info("%s IMU rail pm8150a_l11 failed\n", __func__);
			devm_regulator_put(pdata->reg_vdd);
		}
		rc = regulator_enable(pdata->reg_vdd);
		if (rc < 0) {
			pr_info("%s IMU rail pm8150a_l11 failed\n", __func__);
			devm_regulator_put(pdata->reg_vdd);
		}
	}
	/* Invensense 1.8 PowerRail */
	pdata->reg_vddio = devm_regulator_get(&pdata->pdev->dev, "imu-vddio");
	if (IS_ERR(pdata->reg_vddio)) {
		pr_info("%s can not find vddio\n", __func__);
	}else{
		pr_info("%s vddio has been found in dts\n", __func__);
		rc = regulator_set_load(pdata->reg_vddio, 600000);
		if (rc < 0) {
			pr_info("%s IMU rail pm8150a_l8 failed\n", __func__);
			devm_regulator_put(pdata->reg_vddio);
		}
		rc = regulator_enable(pdata->reg_vddio);
		if (rc < 0) {
			pr_info("%s IMU rail pm8150a_l8 failed\n", __func__);
			devm_regulator_put(pdata->reg_vddio);
		}
	}
	//pinctrl_select_state(pdata->pctrl, pdata->pins_enable);
	//pdata->initDone = true;
	pr_info("%s end\n", __func__);
	return 0;
}

static void qxr_stdalonevwr_free_res(void)
{
#ifdef QXR_SLEEP_POWER0FF_DISABLE
	if (pdata->initDone) {
		if (pdata->reg_vdd) {
			regulator_disable(pdata->reg_vdd);
			devm_regulator_put(pdata->reg_vdd);
		}
		if (pdata->reg_vddio) {
			regulator_disable(pdata->reg_vddio);
			devm_regulator_put(pdata->reg_vddio);
		}
		pinctrl_select_state(pdata->pctrl, pdata->pins_disable);
		pdata->initDone = false;
	}
#endif
	pr_debug("%s initDone:%d\n", __func__, pdata->initDone);
}

static int qxr_stdalonevwr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	pr_info("%s start\n",__func__);
	if(special_mode){
		pr_info("%s done, in special mode\n", __func__);
		return 0;
	}
	pdata = devm_kzalloc(&pdev->dev, sizeof(struct qxr_stdalonevwr),
						GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->pdev = pdev;
	pdata->ndi_5v_en = 1237;
	pdata->initDone = false;
	qxr_stdalonevwr_allocate_res();
	msleep(1);
	pdata->pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pctrl)) {
		pr_info("qxr failed to get pinctrl\n");
		return PTR_ERR(pdata->pctrl);
	};
	pdata->pins_enable = pinctrl_lookup_state(pdata->pctrl, "enable");
	if (IS_ERR(pdata->pins_enable)) {
		 ret = PTR_ERR(pdata->pins_enable);
		 pr_info("qxr Could not get active pinstates, err:%d\n", ret);
		 return PTR_ERR(pdata->pins_enable);
	};
	pdata->pins_disable = pinctrl_lookup_state(pdata->pctrl, "sleep");
	if (IS_ERR(pdata->pins_disable)) {
		ret = PTR_ERR(pdata->pins_disable);
		pr_info("qxr Could not get sleep pinstates, err:%d\n", ret);
		return PTR_ERR(pdata->pins_disable);
	};
	pinctrl_select_state(pdata->pctrl, pdata->pins_enable);
	pdata->initDone = true;
	pr_info("%s done\n", __func__);
	return 0;
}

static int qxr_stdalonevrw_pm_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	if (pdata)
		qxr_stdalonevwr_free_res();
	return 0;
}

static int qxr_stdalonevrw_pm_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	if (pdata)
		qxr_stdalonevwr_allocate_res();
	return 0;
}

static int qxr_stdalonevrw_pm_freeze(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return qxr_stdalonevrw_pm_suspend(dev);
};

static int qxr_stdalonevrw_pm_restore(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return qxr_stdalonevrw_pm_resume(dev);
};

static int qxr_stdalonevrw_pm_poweroff(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return qxr_stdalonevrw_pm_suspend(dev);
};

static int qxr_stdalonevrw_pm_thaw(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return qxr_stdalonevrw_pm_resume(dev);
};

static const struct dev_pm_ops qxr_stdalonevwr_dev_pm_ops = {
	.suspend = qxr_stdalonevrw_pm_suspend,
	.resume = qxr_stdalonevrw_pm_resume,
	.freeze = qxr_stdalonevrw_pm_freeze,
	.restore = qxr_stdalonevrw_pm_restore,
	.thaw = qxr_stdalonevrw_pm_thaw,
	.poweroff = qxr_stdalonevrw_pm_poweroff,
};

static const struct of_device_id qxr_stdalonevwr_match_table[] = {
	{ .compatible = "qcom,xr-stdalonevwr-misc", },
	{}
};

MODULE_DEVICE_TABLE(of, qxr_stdalonevwr_match_table);

static struct platform_driver qxr_stdalonevwr_driver = {
	.driver = {
		.name           = "qxr-stdalonevwr",
		.of_match_table = qxr_stdalonevwr_match_table,
		.pm = &qxr_stdalonevwr_dev_pm_ops,
	},
	.probe          = qxr_stdalonevwr_probe,
};

module_init(qxr_get_boot_mode);
module_platform_driver(qxr_stdalonevwr_driver);
MODULE_DESCRIPTION("QTI XR STANDALONE MISC driver");
MODULE_LICENSE("GPL v2");
