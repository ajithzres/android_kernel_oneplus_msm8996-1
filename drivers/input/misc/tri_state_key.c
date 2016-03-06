/*
 * drivers/input/misc/tri_state_key.c
 *
 * Copyright (C) 2018, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "tri-state-key"
#define NR_STATES (3)

enum key_vals {
	KEYCODE_TOTAL_SILENCE = 600,
	KEYCODE_MODE_ALARMS_ONLY,
	KEYCODE_MODE_PRIORITY_ONLY,
	KEYCODE_MODE_NONE,
	KEYCODE_MODE_MAX
};

static const char *const proc_names[] = {
	"keyCode_top", "keyCode_middle", "keyCode_bottom"
};

#define KEYCODE_BASE 600
#define TOTAL_KEYCODES 6

static int current_mode = 0;
static int keyCode_slider_top = KEYCODE_BASE + 1;
static int keyCode_slider_middle = KEYCODE_BASE + 2;
static int keyCode_slider_bottom = KEYCODE_BASE + 3;

struct switch_dev_data {
	//tri_mode_t last_type;
	//tri_mode_t mode_type;
	//int switch_enable;
	int irq_key3;
	int irq_key2;
	int irq_key1;
	int key1_gpio;//key1 gpio34
	int key2_gpio;//key2 gpio77
	int key3_gpio;

	struct regulator *vdd_io;
	//bool power_enabled;

	struct work_struct work;
	struct switch_dev sdev;
	struct device *dev;
	struct input_dev *input;

	struct timer_list s_timer;
	struct pinctrl * key_pinctrl;
	struct pinctrl_state * set_state;

};

static struct switch_dev_data *switch_data;
static DEFINE_MUTEX(sem);

static void send_input(int keyCode)
{
	input_report_key(switch_data->input, keyCode, 1);
	input_sync(switch_data->input);
	input_report_key(switch_data->input, keyCode, 0);
	input_sync(switch_data->input);
}

static void switch_dev_work(struct work_struct *work)
{

	int keyCode;
	int mode;
	mutex_lock(&sem);

	if(!gpio_get_value(switch_data->key3_gpio))
	{
		mode = 3;
		keyCode = keyCode_slider_bottom;
	}
	else if(!gpio_get_value(switch_data->key2_gpio))
	{
		mode = 2;
		keyCode = keyCode_slider_middle;
	}
	else if(!gpio_get_value(switch_data->key1_gpio))
	{
		mode = 1;
		keyCode = keyCode_slider_top;
	}
        if (current_mode != mode) {
		current_mode = mode;
		switch_set_state(&switch_data->sdev, current_mode);
		send_input(keyCode);
		printk("%s ,tristate set to state(%d) \n", __func__, switch_data->sdev.state);
	}
	mutex_unlock(&sem);
}

irqreturn_t switch_dev_interrupt(int irq, void *_dev)
{
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static int keycode_show(struct seq_file *seq, void *offset)
{
//	mod_timer(&s_timer, jiffies + HZ);
//	if(set_gpio_by_pinctrl() < 0)
//	printk(KERN_ERR "tristate_key set_gpio_by_pinctrl FAILD!!!. \n");
	schedule_work(&switch_data->work);
//	del_timer(&switch_data->s_timer);

//	printk(KERN_ERR "tristate_key set gpio77 timer. \n");
}

static ssize_t tristate_keycode_write(struct file *file,
	const char __user *page, size_t count, loff_t *offset)
{
	struct tristate_data *t = tdata_g;
	char buf[sizeof("600")];
	int data, idx;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, page, min_t(int, count, 3))) {
		dev_err(t->dev, "Failed to read procfs input\n");
		return count;
	}

	if (kstrtoint(buf, 10, &data) < 0)
		return count;

	if (data < KEYCODE_TOTAL_SILENCE || data >= KEYCODE_MODE_MAX)
		return count;

	idx = keycode_get_idx(file->f_path.dentry->d_iname);

	mutex_lock(&t->irq_lock);
	t->key_codes[idx] = data;
	if (!(t->curr_state & BIT(idx)))
		send_input(t, data);
	mutex_unlock(&t->irq_lock);

	return count;
}

static int tristate_keycode_open(struct inode *inode, struct file *file)
{
	int idx = keycode_get_idx(file->f_path.dentry->d_iname);

	/* Pass the index to keycode_show */
	return single_open(file, keycode_show, (void *)(long)idx);
}

static const struct file_operations tristate_keycode_proc = {
	.owner		= THIS_MODULE,
	.open		= tristate_keycode_open,
	.read		= seq_read,
	.write		= tristate_keycode_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tristate_parse_dt(struct tristate_data *t)
{
	struct device_node *np;
	char prop_name[sizeof("tristate,gpio_key#")];
	int i;

	np = t->dev->of_node;
	if (!np)
		return -EINVAL;

	for (i = 0; i < NR_STATES; i++) {
		sprintf(prop_name, "tristate,gpio_key%d", i + 1);

		t->key_gpios[i] = of_get_named_gpio(np, prop_name, 0);
		if (!gpio_is_valid(t->key_gpios[i])) {
			dev_err(t->dev, "Invalid %s property\n", prop_name);
			return -EINVAL;
		}
	}

	return 0;
}

static int tristate_register_irqs(struct tristate_data *t)
{
	char label[sizeof("tristate_key#-int")];
	int i, j, key_irqs[NR_STATES], ret;

	/* Get the IRQ numbers corresponding to the GPIOs */
	for (i = 0; i < NR_STATES; i++) {
		key_irqs[i] = gpio_to_irq(t->key_gpios[i]);
		if (key_irqs[i] < 0) {
			dev_err(t->dev, "Invalid IRQ (%d) for GPIO%d\n",
				key_irqs[i], i + 1);
			return -EINVAL;
		}
	}

	for (i = 0; i < NR_STATES; i++) {
		sprintf(label, "tristate_key%d-int", i + 1);

		ret = gpio_request(t->key_gpios[i], label);
		if (ret < 0) {
			dev_err(t->dev, "Failed to request GPIO%d, ret: %d\n",
				i + 1, ret);
			goto free_gpios;
		}

		ret = gpio_direction_input(t->key_gpios[i]);
		if (ret < 0) {
			dev_err(t->dev, "Failed to set GPIO%d dir, ret: %d\n",
				i + 1, ret);
			goto free_gpios;
		}

		sprintf(label, "tristate_key%d", i + 1);

		ret = request_threaded_irq(key_irqs[i], NULL,
				tristate_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, label, t);
		if (ret < 0) {
			dev_err(t->dev, "Failed to register %s IRQ, ret: %d\n",
				label, ret);
			goto free_irqs;
		}
	}

	for (i = 0; i < NR_STATES; i++)
		enable_irq_wake(key_irqs[i]);

	return 0;

free_irqs:
	for (j = i; j--;)
		free_irq(key_irqs[j], t);
	i = NR_STATES;
free_gpios:
	for (j = i; j--;)
		gpio_free(t->key_gpios[j]);
	return -EINVAL;
}

static void tristate_create_procfs(void)
{
	struct proc_dir_entry *proc_dir;
	int i;

	proc_dir = proc_mkdir("tri-state-key", NULL);
	if (!proc_dir)
		return;

	for (i = 0; i < NR_STATES; i++)
		proc_create_data(proc_names[i], 0644, proc_dir,
			&tristate_keycode_proc, NULL);
}

static int tristate_probe(struct platform_device *pdev)
{
	struct tristate_data *t;
	int i, ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->dev = &pdev->dev;
	tdata_g = t;

	ret = tristate_parse_dt(t);
	if (ret)
		goto free_t;

	t->input = input_allocate_device();
	if (!t->input) {
		dev_err(t->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto free_t;
	}

	t->input->name = DRIVER_NAME;
	t->input->dev.parent = t->dev;
	input_set_drvdata(t->input, t);

	set_bit(EV_KEY, t->input->evbit);
	set_bit(KEYCODE_TOTAL_SILENCE, t->input->keybit);
	set_bit(KEYCODE_MODE_ALARMS_ONLY, t->input->keybit);
	set_bit(KEYCODE_MODE_PRIORITY_ONLY, t->input->keybit);
	set_bit(KEYCODE_MODE_NONE, t->input->keybit);

static int keyCode_top_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n", keyCode_slider_top);
    return 0;
}

static ssize_t keyCode_top_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int data;
	char buf[10];

	if (copy_from_user(buf, page, t))
	{
		dev_err(switch_data->dev, "read proc input error.\n");
		return t;
	}

	if (sscanf(buf, "%d", &data) != 1)
		return t;
	if (data < KEYCODE_BASE || data >= (KEYCODE_BASE + TOTAL_KEYCODES))
		return t;

	keyCode_slider_top = data;
	if (current_mode == 1)
		send_input(keyCode_slider_top);

	return t;
}

static int keyCode_top_open(struct inode *inode, struct file *file)
{
	return single_open(file, keyCode_top_show, inode->i_private);
}

const struct file_operations proc_keyCode_top =
{
	.owner		= THIS_MODULE,
	.open		= keyCode_top_open,
	.read		= seq_read,
	.write		= keyCode_top_write,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int keyCode_middle_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n", keyCode_slider_middle);
    return 0;
}

static ssize_t keyCode_middle_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int data;
	char buf[10];

	if (copy_from_user(buf, page, t))
	{
		dev_err(switch_data->dev, "read proc input error.\n");
		return t;
	}

	if (sscanf(buf, "%d", &data) != 1)
		return t;
	if (data < KEYCODE_BASE || data >= (KEYCODE_BASE + TOTAL_KEYCODES))
		return t;

	keyCode_slider_middle = data;
	if (current_mode == 2)
		send_input(keyCode_slider_middle);

	return t;
}

static int keyCode_middle_open(struct inode *inode, struct file *file)
{
	return single_open(file, keyCode_middle_show, inode->i_private);
}

const struct file_operations proc_keyCode_middle =
{
	.owner		= THIS_MODULE,
	.open		= keyCode_middle_open,
	.read		= seq_read,
	.write		= keyCode_middle_write,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int keyCode_bottom_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n", keyCode_slider_bottom);
    return 0;
}

static ssize_t keyCode_bottom_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int data;
	char buf[10];

	if (copy_from_user(buf, page, t))
	{
		dev_err(switch_data->dev, "read proc input error.\n");
		return t;
	}

	if (sscanf(buf, "%d", &data) != 1)
		return t;
	if (data < KEYCODE_BASE || data >= (KEYCODE_BASE + TOTAL_KEYCODES))
		return t;

	keyCode_slider_bottom = data;
	if (current_mode == 3)
		send_input(keyCode_slider_bottom);

	return t;
}

static int keyCode_bottom_open(struct inode *inode, struct file *file)
{
	return single_open(file, keyCode_bottom_show, inode->i_private);
}

const struct file_operations proc_keyCode_bottom =
{
	.owner		= THIS_MODULE,
	.open		= keyCode_bottom_open,
	.read		= seq_read,
	.write		= keyCode_bottom_write,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int tristate_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct proc_dir_entry *procdir;
	int error=0;
	int i;

	//void __iomem *cfg_reg;

        switch_data = kzalloc(sizeof(struct switch_dev_data), GFP_KERNEL);
        switch_data->dev = dev;

	switch_data->input = input_allocate_device();

	switch_data->input->name = DRV_NAME;
	switch_data->input->dev.parent = &pdev->dev;
	set_bit(EV_KEY, switch_data->input->evbit);
	for (i = KEYCODE_BASE; i < KEYCODE_BASE + TOTAL_KEYCODES; i++)
	    set_bit(i, switch_data->input->keybit);
	input_set_drvdata(switch_data->input, switch_data);
	error = input_register_device(switch_data->input);
	if (error) {
		dev_err(dev, "Failed to register input device\n");
		goto err_input_device_register;
	}
        #if 0
	    switch_data->key_pinctrl = devm_pinctrl_get(switch_data->dev);
         if (IS_ERR_OR_NULL(switch_data->key_pinctrl)) {
		        dev_err(switch_data->dev, "Failed to get pinctrl \n");
		        goto err_switch_dev_register;
	     }
         switch_data->set_state =pinctrl_lookup_state(switch_data->key_pinctrl,"pmx_tri_state_key_active");
         if (IS_ERR_OR_NULL(switch_data->set_state)) {
		        dev_err(switch_data->dev, "Failed to lookup_state \n");
		        goto err_switch_dev_register;
	     }

	     set_gpio_by_pinctrl();
		#endif
        //switch_data->last_type = MODE_UNKNOWN;

        //tristate_supply_init();
		error = switch_dev_get_devtree_pdata(dev);
		if (error) {
			dev_err(dev, "parse device tree fail!!!\n");
			goto err_switch_dev_register;
		}

		//config irq gpio and request irq
	switch_data->irq_key1 = gpio_to_irq(switch_data->key1_gpio);
       if (switch_data->irq_key1 <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq_key1, switch_data->key1_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key1_gpio,"tristate_key1-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key1_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}

			error = request_irq(switch_data->irq_key1, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key1", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key1);

        		switch_data->irq_key1 = -EINVAL;
        		goto err_request_irq;
            }
       }
       //config irq gpio and request irq
	 switch_data->irq_key2 = gpio_to_irq(switch_data->key2_gpio);
       if (switch_data->irq_key2 <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq_key2, switch_data->key2_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key2_gpio,"tristate_key2-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key2_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}

			error = request_irq(switch_data->irq_key2, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key2", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key2);

        		switch_data->irq_key2 = -EINVAL;
        		goto err_request_irq;
            }

       }

	   switch_data->irq_key3 = gpio_to_irq(switch_data->key3_gpio);
	   if (switch_data->irq_key3 <= 0)
	   {
	            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, \
	            switch_data->irq_key3, switch_data->key3_gpio);
	            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key3_gpio,"tristate_key3-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key3_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}


			error = request_irq(switch_data->irq_key3, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key3", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key3);

        		switch_data->irq_key3 = -EINVAL;
        		goto err_request_irq;
            }

       }


        INIT_WORK(&switch_data->work, switch_dev_work);

        init_timer(&switch_data->s_timer);
        switch_data->s_timer.function = &timer_handle;
        switch_data->s_timer.expires = jiffies + 5*HZ;

        add_timer(&switch_data->s_timer);

        enable_irq_wake(switch_data->irq_key1);
        enable_irq_wake(switch_data->irq_key2);
	    enable_irq_wake(switch_data->irq_key3);


        switch_data->sdev.name = DRV_NAME;
       	error = switch_dev_register(&switch_data->sdev);
	    if (error < 0)
		    goto err_request_gpio;
		 //set_gpio_by_pinctrl();
        //report the first switch
        //switch_dev_work(&switch_data->work);

	procdir = proc_mkdir("tri-state-key", NULL);
	proc_create_data("keyCode_top", 0666, procdir, &proc_keyCode_top, NULL);
	proc_create_data("keyCode_middle", 0666, procdir, &proc_keyCode_middle, NULL);
	proc_create_data("keyCode_bottom", 0666, procdir, &proc_keyCode_bottom, NULL);

        return 0;


err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->key2_gpio);
	gpio_free(switch_data->key1_gpio);
	gpio_free(switch_data->key3_gpio);
err_switch_dev_register:
	kfree(switch_data);
err_input_device_register:
	input_unregister_device(switch_data->input);
	input_free_device(switch_data->input);


	return error;
}

	/* Process initial state */
	tristate_process_state(t);

	mutex_init(&t->irq_lock);
	ret = tristate_register_irqs(t);
	if (ret)
		goto input_unregister;

	tristate_create_procfs();
	return 0;

input_unregister:
	input_unregister_device(t->input);
free_input:
	input_free_device(t->input);
free_t:
	kfree(t);
	return ret;
}

static const struct of_device_id tristate_of_match[] = {
	{ .compatible = "oneplus,tri-state-key", },
	{ },
};

static struct platform_driver tristate_driver = {
	.probe	= tristate_probe,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = tristate_of_match,
	},
};

static int __init tristate_init(void)
{
	return platform_driver_register(&tristate_driver);
}
device_initcall(tristate_init);
