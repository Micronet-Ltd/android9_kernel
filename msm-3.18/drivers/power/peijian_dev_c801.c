/*
 * Driver for External Devices  fingerprint & identity function
 *
 * Copyright (C) 2016 Texas Instruments
 *
 * Author : Yuwangqing .A.G. <wangqing.yu@lovdream.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "peijian_dev_c801.h"

static ssize_t pogo_irq_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct peijian_data *pdata = dev_get_drvdata(pdev);
	int enabled = 0;

	sscanf(buff, "%d", &enabled);

	if(enabled){
		printk("+++ peijian pogo_irq to 1 +++\n");
		gpio_direction_output(pdata->peijian_pogo_irq, 1);
	}
	else{
		printk("+++ peijian pogo_irq to 0 +++\n");
		gpio_direction_output(pdata->peijian_pogo_irq, 0);
	}

	return size;
}
static ssize_t pogo_irq_show(struct device *pdev,
			   struct device_attribute *attr, char *buf)
{
	struct peijian_data *pdata = dev_get_drvdata(pdev);
	int ret;

	ret = gpio_get_value(pdata->peijian_pogo_irq);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static DEVICE_ATTR(pogo_irq, 0660, pogo_irq_show, pogo_irq_store);

static ssize_t pogo_gpio_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct peijian_data *pdata = dev_get_drvdata(pdev);
	int enabled = 0;

	sscanf(buff, "%d", &enabled);

	if(enabled){
		printk("+++ peijian pogo_gpio1 to 1 +++\n");
		gpio_direction_output(pdata->peijian_pogo_pin4, 1);
		gpio_direction_output(pdata->peijian_pogo_pin8, 1);
		gpio_direction_output(pdata->peijian_pogo_pin12, 1);
		gpio_direction_output(pdata->peijian_pogo_pin13, 1);
		gpio_direction_output(pdata->peijian_pogo_pin21, 1);
		gpio_direction_output(pdata->peijian_pogo_pin22, 1);
		gpio_direction_output(pdata->peijian_pogo_pin24, 1);
		gpio_direction_output(pdata->peijian_pogo_pin25, 1);
		gpio_direction_output(pdata->peijian_pogo_pin26, 1);
	}
	else{
		printk("+++ peijian pogo_gpio1 to 0 +++\n");
		gpio_direction_output(pdata->peijian_pogo_pin4, 0);
		gpio_direction_output(pdata->peijian_pogo_pin8, 0);
		gpio_direction_output(pdata->peijian_pogo_pin12, 0);
		gpio_direction_output(pdata->peijian_pogo_pin13, 0);
		gpio_direction_output(pdata->peijian_pogo_pin21, 0);
		gpio_direction_output(pdata->peijian_pogo_pin22, 0);
		gpio_direction_output(pdata->peijian_pogo_pin24, 0);
		gpio_direction_output(pdata->peijian_pogo_pin25, 0);
		gpio_direction_output(pdata->peijian_pogo_pin26, 0);
	}

	return size;
}
static ssize_t pogo_gpio_show(struct device *pdev,
			   struct device_attribute *attr, char *buf)
{
	struct peijian_data *pdata = dev_get_drvdata(pdev);
	int ret;

	ret = gpio_get_value(pdata->peijian_pogo_pin4);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static DEVICE_ATTR(gpio_en, 0660, pogo_gpio_show, pogo_gpio_store);


static int peijian_dev_pinctrl_configure(struct pinctrl *peijian_pinctrl)
{
	struct pinctrl_state *set_state;
	int retval;

	set_state =
		pinctrl_lookup_state(peijian_pinctrl,
					"peijian_dev_active");
	if (IS_ERR(set_state)) {
		pr_err("cannot get pinctrl peijian dev state\n");
		return PTR_ERR(set_state);
	}

	retval = pinctrl_select_state(peijian_pinctrl, set_state);
	if (retval) {
		pr_err("cannot set pinctrl peijian dev state\n");
		return retval;
	}

	return 0;
}

static void init_peijian_dev_gpio(struct peijian_data *pdata)
{
	int rc;

	pr_info("===PEIJIAN DEV config GPIO===\n");

	rc = gpio_request(pdata->peijian_pogo_irq, "peijian_pogo_irq");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_irq);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin4, "peijian_pogo_pin4");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin4);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin8, "peijian_pogo_pin8");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin8);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin12, "peijian_pogo_pin12");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin12);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin13, "peijian_pogo_pin13");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin13);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin21, "peijian_pogo_pin21");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin21);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin22, "peijian_pogo_pin22");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin22);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin24, "peijian_pogo_pin24");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin24);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin25, "peijian_pogo_pin25");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin25);
		return;
	}
	rc = gpio_request(pdata->peijian_pogo_pin26, "peijian_pogo_pin26");
	if (rc < 0) {
		pr_err("PEIJIAN DEV config: request  gpio[%d] Failed!\n", pdata->peijian_pogo_pin26);
		return;
	}
	
	gpio_direction_output(pdata->peijian_pogo_irq, 1);
	gpio_direction_output(pdata->peijian_pogo_pin4, 0);
	gpio_direction_output(pdata->peijian_pogo_pin8, 0);
	gpio_direction_output(pdata->peijian_pogo_pin12, 0);
	gpio_direction_output(pdata->peijian_pogo_pin13, 0);
	gpio_direction_output(pdata->peijian_pogo_pin21, 0);
	gpio_direction_output(pdata->peijian_pogo_pin22, 0);
	gpio_direction_output(pdata->peijian_pogo_pin24, 0);
	gpio_direction_output(pdata->peijian_pogo_pin25, 0);
	gpio_direction_output(pdata->peijian_pogo_pin26, 0);
	

	pr_info("===PEIJIAN DEV setup GPIO done===\n");
	return;
}

static int peijian_dev_parse_dt(struct device *dev, struct peijian_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->peijian_pogo_irq= of_get_named_gpio(np, "peijian_pogo_irq", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_irq))
		pr_err("%s:%d, peijian_pogo_irq gpio not specified\n", __func__, __LINE__);

	pdata->peijian_pogo_pin4= of_get_named_gpio(np, "peijian_pogo_pin4", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin4))
		pr_err("%s:%d, peijian_pogo_pin4 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin8= of_get_named_gpio(np, "peijian_pogo_pin8", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin8))
		pr_err("%s:%d, peijian_pogo_pin8 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin12= of_get_named_gpio(np, "peijian_pogo_pin12", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin12))
		pr_err("%s:%d, peijian_pogo_pin12 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin13= of_get_named_gpio(np, "peijian_pogo_pin13", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin13))
		pr_err("%s:%d, peijian_pogo_pin13 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin21= of_get_named_gpio(np, "peijian_pogo_pin21", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin21))
		pr_err("%s:%d, peijian_pogo_pin21 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin22= of_get_named_gpio(np, "peijian_pogo_pin22", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin22))
		pr_err("%s:%d, peijian_pogo_pin22 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin24= of_get_named_gpio(np, "peijian_pogo_pin24", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin24))
		pr_err("%s:%d, peijian_pogo_pin24 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin25= of_get_named_gpio(np, "peijian_pogo_pin25", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin25))
		pr_err("%s:%d, peijian_pogo_pin25 gpio not specified\n", __func__, __LINE__);
	
	pdata->peijian_pogo_pin26= of_get_named_gpio(np, "peijian_pogo_pin26", 0);
	if (!gpio_is_valid(pdata->peijian_pogo_pin26))
		pr_err("%s:%d, peijian_pogo_pin26 gpio not specified\n", __func__, __LINE__);

	return 0;
}

static void peijian_resume(struct peijian_data *pdata)
{
	return;
}

static void peijian_suspend(struct peijian_data *pdata)
{
	return;
}

static int peijian_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	struct peijian_data *pdata = container_of(noti, struct peijian_data, peijian_notifier);
	int *blank;

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK && pdata) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			peijian_resume(pdata);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			peijian_suspend(pdata);
		}
	}
	return 0;
}

void pogo_irq_disable(struct peijian_data *data)
{
	unsigned long irqflags;
	int irq_num;
	irq_num = gpio_to_irq(data->peijian_pogo_irq);

	spin_lock_irqsave(&data->pogo_irq_lock, irqflags);
	if (!data->pogo_irq_is_disable)
	{
		data->pogo_irq_is_disable = 1;
		disable_irq_nosync(irq_num);
	}
	spin_unlock_irqrestore(&data->pogo_irq_lock, irqflags);
}

void pogo_irq_enable(struct peijian_data *data)
{
	unsigned long irqflags = 0;
	int irq_num;
	irq_num = gpio_to_irq(data->peijian_pogo_irq);

	spin_lock_irqsave(&data->pogo_irq_lock, irqflags);
	if (data->pogo_irq_is_disable)
	{
		enable_irq(irq_num);
		data->pogo_irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&data->pogo_irq_lock, irqflags);
}

static void pogo_irq_work_func(struct work_struct *w)
{
	struct peijian_data *pdata = container_of(w, struct peijian_data,
						pogo_work.work);
	int ret;
	printk("%s\n", __func__);
	ret = gpio_get_value(pdata->peijian_pogo_irq);

	if(ret){
		gpio_direction_output(12,0);
		gpio_direction_output(47,0);
		printk("open peijian usb\n");
	}
	else{
		gpio_direction_output(12,1);
		gpio_direction_output(47,1);
		printk("close peijian usb\n");
	}
	pogo_irq_enable(pdata);
	return;
}

static irqreturn_t pogo_irq_handler(int irq, void *dev)
{
	struct peijian_data *pdata = dev;
	pogo_irq_disable(pdata);

	queue_delayed_work(pdata->pogo_wq, &pdata->pogo_work,
			msecs_to_jiffies(30));

	return IRQ_HANDLED;
}

static void init_pogo_interrupt(struct peijian_data *pdata)
{
	int ret, irq_num;
	ret = gpio_direction_input(pdata->peijian_pogo_irq);
	if (ret) {
		pr_err("%s:%d, set input failed\n", __func__, __LINE__);
		gpio_free(pdata->peijian_pogo_irq);
	}

	irq_num = gpio_to_irq(pdata->peijian_pogo_irq);
        if (request_irq(irq_num, pogo_irq_handler, (IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT),
                "pogo_irq", pdata)) {
                pr_err("setup pogo irq failed!!\n");
        }
}

static int peijian_dev_probe(struct platform_device *pdev)
{
	struct peijian_data *pdata;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
	{
		pr_err("peijian Alloc GFP_KERNEL memory failed.\n");
		return -ENOMEM;
	}

	pdata->usb_psy = power_supply_get_by_name("usb");
	if (!pdata->usb_psy) {
		printk("peijian DEV:USB supply not found, contining\n");
		return -EPROBE_DEFER;
	}

	if (pdev->dev.of_node) {
		ret = peijian_dev_parse_dt(&pdev->dev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else{
		pr_err("%s: Device Tree must be supported\n", __func__);
		return -1;
	}
	pdata->pogo_wq = create_singlethread_workqueue("peijian_pogo_wq");
	if (!pdata->pogo_wq)
	{
		pr_err("peijian dev Creat workqueue failed.\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&pdata->pogo_work, pogo_irq_work_func);
	spin_lock_init(&pdata->pogo_irq_lock);
	pdata->peijian_dev_class = class_create(THIS_MODULE, "ext_dev");
	if (IS_ERR(pdata->peijian_dev_class)){
		pr_err("peijian DEV class create failed\n");
		return PTR_ERR(pdata->peijian_dev_class);
	}

	pdata->dev = kzalloc(sizeof(struct device), GFP_KERNEL);
	if (pdata->dev == NULL) {
		pr_err("%s : no memory for device\n", __func__);
		return -ENOMEM;
	}

	pdata->dev = device_create(pdata->peijian_dev_class, NULL,
					MKDEV(0, 0), NULL, "function");
	if (IS_ERR(pdata->dev))
		return PTR_ERR(pdata->dev);

	ret = device_create_file(pdata->dev, &dev_attr_pogo_irq);
		if (ret) {
			pr_err("peijian ext dev can not create file pogo_irq");
			device_destroy(pdata->peijian_dev_class, pdata->dev->devt);
			return ret;
		}

	ret = device_create_file(pdata->dev, &dev_attr_gpio_en);
		if (ret) {
			pr_err("peijian ext dev can not create file dev_attr_gpio_en");
			device_destroy(pdata->peijian_dev_class, pdata->dev->devt);
			return ret;
		}

	dev_set_drvdata(pdata->dev, pdata);

	/* Get pinctrl if target uses pinctrl */
	pdata->peijian_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pdata->peijian_pinctrl)) {
		if (PTR_ERR(pdata->peijian_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_err("Target does not use pinctrl\n");
		pdata->peijian_pinctrl = NULL;
	}

	if (pdata->peijian_pinctrl) {
		ret = peijian_dev_pinctrl_configure(pdata->peijian_pinctrl);
		if (ret) {
			pr_err("cannot set peijian dev pinctrl\n");
		}
	}

	init_peijian_dev_gpio(pdata);
	init_pogo_interrupt(pdata);

	platform_set_drvdata(pdev, pdata);

	pdata->peijian_notifier.notifier_call = peijian_fb_notifier_callback;
	fb_register_client(&pdata->peijian_notifier);

	return 0;
}

static int peijian_dev_remove(struct platform_device *pdev)
{
	struct peijian_data *pdata;
	pdata = (struct peijian_data *)platform_get_drvdata(pdev);

	pr_info("===PEIJIAN DEV exit===\n");
	if (pdata->peijian_wq)
	{
		destroy_workqueue(pdata->peijian_wq);
	}
	device_remove_file(pdata->dev, &dev_attr_pogo_irq);
	device_remove_file(pdata->dev, &dev_attr_gpio_en);
	device_destroy(pdata->peijian_dev_class, pdata->dev->devt);
	class_destroy(pdata->peijian_dev_class);
	//regulator_put(reg_peijian_vio);
	fb_unregister_client(&pdata->peijian_notifier);
	//regulator_put(reg_gps_vio);
	return 0;
}

static void peijian_shutdown(struct platform_device *pdev)
{
	struct peijian_data *pdata;
	pdata = (struct peijian_data *)platform_get_drvdata(pdev);

	pr_info("===PEIJIAN shutdown call===\n");
	return;
}

static struct of_device_id peijian_dev_match_table[] = {
	{ .compatible = "lovdream,peijian-dev",},
	{ },
};

static struct platform_driver peijian_dev_driver = {
	.probe = peijian_dev_probe,
	.remove = peijian_dev_remove,
	.shutdown	= peijian_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name = "peijian_dev",
		.of_match_table = peijian_dev_match_table,
	},
};

module_platform_driver(peijian_dev_driver);
MODULE_LICENSE("GPL v2");

