// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

//Smt:[FEAT_LAB_TAG, FEAT_AVC_CLEAN] {@
#include <linux/uaccess.h>
#include <config/ProductConfig.h>
#ifndef FEAT_LAB_TAG
#define FEAT_LAB_TAG 0
#endif

#ifndef FEAT_AVC_CLEAN
#define FEAT_AVC_CLEAN 0
#endif
//@}

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, saved_command_line);
	seq_putc(m, '\n');
	return 0;
}

static int __init proc_cmdline_init(void)
{
	proc_create_single("cmdline", 0, NULL, cmdline_proc_show);
	return 0;
}
fs_initcall(proc_cmdline_init);

//Smt:[FEAT_LAB_TAG, FEAT_AVC_CLEAN] {@
#if FEAT_LAB_TAG && FEAT_AVC_CLEAN
bool isEnableAvcCleanSwitch = false;
int sampling_num_max = 1;

static int avcclean_proc_show(struct seq_file *m, void *v)
{
	if (isEnableAvcCleanSwitch) {
		seq_printf(m, "%s %d\n", "enable", sampling_num_max);
	} else {
		seq_printf(m, "%s\n", "disable");
	}
	return 0;
}

static int avcclean_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, avcclean_proc_show, NULL);
}


static ssize_t avcclean_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *offs) {
	char buffer[64];
	char sampling_num_buf[64];
	int var;
	memset(buffer, 0, sizeof(buffer));
	if (count >= sizeof(buffer) || copy_from_user(buffer, buf, count) != 0) {
		count = -EINVAL;
		goto out;
	}
	if (memcmp(buffer,"enable",6) == 0) {
		isEnableAvcCleanSwitch = true;
		strncpy(sampling_num_buf, &buffer[6], count-6);
		kstrtoint(sampling_num_buf, 10, &var);
		if (var > 0)
			sampling_num_max = var;
	} else if (memcmp(buffer,"disable",7) == 0) {
		isEnableAvcCleanSwitch = false;
	}
out:
	return count;
}

static const struct file_operations avcclean_proc_fops = {
	.open		= avcclean_proc_open,
	.write		= avcclean_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_avcclean_init(void)
{
	proc_create("avcclean", S_IRUSR|S_IWUSR, NULL, &avcclean_proc_fops);
	return 0;
}
fs_initcall(proc_avcclean_init);
//@}
#endif
