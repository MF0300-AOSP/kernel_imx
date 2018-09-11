/*
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   Copyright (C) 2006-2007 - Motorola
   Copyright (c) 2008-2010, The Linux Foundation. All rights reserved.

   Date         Author           Comment
   -----------  --------------   --------------------------------
   2006-Apr-28	Motorola	 The kernel module for running the Bluetooth(R)
				 Sleep-Mode Protocol from the Host side
   2006-Sep-08  Motorola         Added workqueue for handling sleep work.
   2007-Jan-24  Motorola         Added mbm_handle_ioi() call to ISR.

*/

#include <linux/module.h>	/* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
    #include <linux/ioport.h>
#include <linux/param.h>
#include <linux/bitops.h>
#include <linux/termios.h>
    #include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/platform_data/serial-imx-ctrl.h>
#include <linux/serial_core.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h> /* event notifications */
#include "hci_uart.h"

//#define BT_SLEEP_DBG
//#ifndef BT_SLEEP_DBG
//#define BT_DBG(fmt, arg...)
//#else
//#define BT_DBG(fmt, arg...)  printk("%s: " fmt "\n" , __func__, ## arg)
//#endif


/*
 * Defines
 */

#define VERSION		"1.1"
#define PROC_DIR	"bluetooth/sleep"

    #define POLARITY_LOW 0
    #define POLARITY_HIGH 1

    /* enable/disable wake-on-bluetooth */
    #define BT_ENABLE_IRQ_WAKE 1

    #define BT_BLUEDROID_SUPPORT 1

struct bluesleep_info {
	unsigned host_wake;
	unsigned ext_wake;
	unsigned host_wake_irq;
	struct uart_port *uport;
            struct wake_lock wake_lock;
            int irq_polarity;
            int has_ext_wake;
};

/* work function */
static void bluesleep_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, bluesleep_sleep_work);

/* Macros for handling sleep work */
#define bluesleep_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)

/* 1 second timeout */
    #define TX_TIMER_INTERVAL  10

/* state variable names and bit positions */
#define BT_PROTO	0x01
#define BT_TXDATA	0x02
#define BT_ASLEEP	0x04
    #define BT_EXT_WAKE     0x08
    #define BT_SUSPEND      0x10

    #if BT_BLUEDROID_SUPPORT
    static bool has_lpm_enabled = false;
    #else
    /* global pointer to a single hci device. */
    static struct hci_dev *bluesleep_hdev;
    #endif

    static struct platform_device *bluesleep_uart_dev;
static struct bluesleep_info *bsi;

/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);

/*
 * Local function prototypes
 */
    #if !BT_BLUEDROID_SUPPORT
    static int bluesleep_hci_event(struct notifier_block *this,
                            unsigned long event, void *data);
    #endif
    static int bluesleep_start(void);
    static void bluesleep_stop(void);

/*
 * Global variables
 */
/** Global state flags */
static unsigned long flags;

/** Tasklet to respond to change in hostwake line */
static struct tasklet_struct hostwake_task;

/** Transmission timer */
    static void bluesleep_tx_timer_expire(unsigned long data);
    static DEFINE_TIMER(tx_timer, bluesleep_tx_timer_expire, 0, 0);

/** Lock for state transitions */
static spinlock_t rw_lock;

    #if !BT_BLUEDROID_SUPPORT
/** Notifier block for HCI events */
struct notifier_block hci_event_nblock = {
	.notifier_call = bluesleep_hci_event,
};
    #endif

struct proc_dir_entry *proc_sleep_dir;
struct proc_dir_entry *proc_bluetooth_dir;

/*
 * Local functions
 */
static void hsuart_power(int on)
{
       if (test_bit(BT_SUSPEND, &flags))
              return;
	if (on) {
		imx_uart_request_clock_on(bsi->uport);
		imx_uart_set_mctrl(bsi->uport, TIOCM_RTS);
		BT_DBG("uart_power on");
	} else {
		imx_uart_set_mctrl(bsi->uport, 0);
		imx_uart_request_clock_off(bsi->uport);
		BT_DBG("uart_power off");
	}
}


/**
 * @return 1 if the Host can go to sleep, 0 otherwise.
 */
    int bluesleep_can_sleep(void)
{
	BT_DBG("(gpio_get_value(bsi->host_wake) != bsi->irq_polarity is %d,(!test_bit(BT_EXT_WAKE, &flags)) is %d,(bsi->uport != NULL) is %d",
		gpio_get_value(bsi->host_wake) != bsi->irq_polarity,!test_bit(BT_EXT_WAKE, &flags),bsi->uport != NULL);
	/* check if MSM_WAKE_BT_GPIO and BT_WAKE_MSM_GPIO are both deasserted */
            return ((gpio_get_value(bsi->host_wake) != bsi->irq_polarity) &&
                    (!test_bit(BT_EXT_WAKE, &flags)) &&
                    (bsi->uport != NULL));
}

int bluesleep_sleep_wakeup(void)
{
	int clk_flag = 0;

	if (test_bit(BT_ASLEEP, &flags)) {
		BT_DBG("waking up...");
                    wake_lock(&bsi->wake_lock);
		/* Start the timer */
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
                    if (bsi->has_ext_wake == 1)
                            gpio_set_value(bsi->ext_wake, 1);
                    set_bit(BT_EXT_WAKE, &flags);
		clear_bit(BT_ASLEEP, &flags);
		/*Activating UART */
		clk_flag = 1;
	}

	return clk_flag;
}

/**
 * @brief@  main sleep work handling function which update the flags
 * and activate and deactivate UART ,check FIFO.
 */
static void bluesleep_sleep_work(struct work_struct *work)
{
	int wake_flag = 0;
	if (bluesleep_can_sleep()) {
					BT_DBG("bluesleep can sleep");									// added by  yjhuang 2013-08-08
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			BT_DBG("already asleep");
			return;
		}

		if (imx_uart_tx_empty(bsi->uport)) {
			BT_DBG("going to sleep...");
			set_bit(BT_ASLEEP, &flags);
			/*Deactivating UART */
			hsuart_power(0);
                            /* UART clk is not turned off immediately. Release
                             * wakelock after 500 ms.
                             */
                            wake_lock_timeout(&bsi->wake_lock, HZ / 2);
		} else {
                            mod_timer(&tx_timer, jiffies + TX_TIMER_INTERVAL * HZ);
			return;
		}
            } else if (!test_bit(BT_EXT_WAKE, &flags)
                            && !test_bit(BT_ASLEEP, &flags)) {
					BT_DBG("== set bt exit wake ==");								// added by yjhuang 2013-08-08
                    mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
                    if (bsi->has_ext_wake == 1)
                            gpio_set_value(bsi->ext_wake, 1);
                    set_bit(BT_EXT_WAKE, &flags);
	} else {
					BT_DBG("== call bluesleep wakeup ==");							// added by yjhuang 2013-08-08
		wake_flag = bluesleep_sleep_wakeup();
		if(wake_flag)
			hsuart_power(1);
	}
}

/**
 * A tasklet function that runs in tasklet context and reads the value
 * of the HOST_WAKE GPIO pin and further defer the work.
 * @param data Not used.
 */
static void bluesleep_hostwake_task(unsigned long data)
{
	BT_DBG("hostwake line change");

	spin_lock(&rw_lock);
            if ((gpio_get_value(bsi->host_wake) == bsi->irq_polarity))
                    bluesleep_rx_busy();
            else
                    bluesleep_rx_idle();
	spin_unlock(&rw_lock);

}

/**
 * Handles proper timer action when outgoing data is delivered to the
 * HCI line discipline. Sets BT_TXDATA.
 */
static int bluesleep_outgoing_data(void)
{
	unsigned long irq_flags;

	int i = 0;

	spin_lock_irqsave(&rw_lock, irq_flags);
            /* log data passing by */
            set_bit(BT_TXDATA, &flags);
            /* if the tx side is sleeping... */
            if (!test_bit(BT_EXT_WAKE, &flags)) {
                    BT_DBG("tx was sleeping");
                    i = bluesleep_sleep_wakeup();
            }
	spin_unlock_irqrestore(&rw_lock, irq_flags);

	return i;
}

    #if BT_BLUEDROID_SUPPORT
/*    static struct uart_port *bluesleep_get_uart_port(void)
    {
            struct uart_port *uport = NULL;
            if (bluesleep_uart_dev)
                    uport =  (struct uart_port *)platform_get_drvdata(bluesleep_uart_dev);

            return uport;
    }
*/

    static int bluesleep_read_seq_lpm(struct seq_file *seq, void *offset)
    {
            return seq_printf(seq, "unsupported to read\n");
    }

static int bluesleep_open_seq_lpm(struct inode *inode,
					     struct file *file)
{
	return single_open(file, bluesleep_read_seq_lpm,
			   NULL);
}
    static int bluesleep_write_seq_lpm(struct file *file, const char *buffer,
					      size_t count, loff_t * ppos)
    {
            char b;

		BT_DBG("zhanglongbo: bluesleep_write_seq_lpm in");

            if (count < 1)
                    return -EINVAL;

            if (copy_from_user(&b, buffer, 1))
                    return -EFAULT;

		BT_DBG("zhanglongbo: b is %c",b);

            if (b == '0') {
                    /* HCI_DEV_UNREG */
                    bluesleep_stop();
                    has_lpm_enabled = false;
                    bsi->uport = NULL;
            } else {
                    /* HCI_DEV_REG */
                    if (!has_lpm_enabled) {
                            has_lpm_enabled = true;

				//we get uport in bluesleep_start() don't use this
				//bsi->uport = bluesleep_get_uart_port();
                            /* if bluetooth started, start bluesleep*/
                            bluesleep_start();
                    }
            }

            return count;
    }

    static int bluesleep_read_seq_btwrite(struct seq_file *seq, void *offset)
    {
            return seq_printf(seq, "unsupported to read\n");
    }

static int bluesleep_open_seq_btwrite(struct inode *inode,
					     struct file *file)
{
	return single_open(file, bluesleep_read_seq_btwrite,
			   NULL);
}
    static int bluesleep_write_seq_btwrite(struct file *file, const char *buffer,
					      size_t count, loff_t * ppos)
    {
            char b;
		int wake_flag = 0;
            if (count < 1)
                    return -EINVAL;

            if (copy_from_user(&b, buffer, 1))
                    return -EFAULT;

            /* HCI_DEV_WRITE */
            if (b != '0') {
                    wake_flag = bluesleep_outgoing_data();
			if(wake_flag)
				hsuart_power(1);
            }

            return count;
    }
    #else
    /**
 * Handles HCI device events.
 * @param this Not used.
 * @param event The event that occurred.
 * @param data The HCI device associated with the event.
 * @return <code>NOTIFY_DONE</code>.
 */
static int bluesleep_hci_event(struct notifier_block *this,
				unsigned long event, void *data)
{
	struct hci_dev *hdev = (struct hci_dev *) data;
	struct hci_uart *hu;
	struct uart_state *state;

	if (!hdev)
		return NOTIFY_DONE;

	switch (event) {
	case HCI_DEV_REG:
		if (!bluesleep_hdev) {
			bluesleep_hdev = hdev;
			hu  = (struct hci_uart *) hdev->driver_data;
			state = (struct uart_state *) hu->tty->driver_data;
			bsi->uport = state->uart_port;
                            /* if bluetooth started, start bluesleep*/
                            bluesleep_start();
		}
		break;
	case HCI_DEV_UNREG:
                    bluesleep_stop();
		bluesleep_hdev = NULL;
		bsi->uport = NULL;
                    /* if bluetooth stopped, stop bluesleep also */
                    break;
	case HCI_DEV_WRITE:
		bluesleep_outgoing_data();
		break;
	}

	return NOTIFY_DONE;
}
    #endif

/**
 * Handles transmission timer expiration.
 * @param data Not used.
 */
static void bluesleep_tx_timer_expire(unsigned long data)
{
	unsigned long irq_flags;

            BT_DBG("Tx timer expired");

            spin_lock_irqsave(&rw_lock, irq_flags);

	/* were we silent during the last timeout? */
	if (!test_bit(BT_TXDATA, &flags)) {
		BT_DBG("Tx has been idle");
                    if (bsi->has_ext_wake == 1)
                            gpio_set_value(bsi->ext_wake, 0);
                    clear_bit(BT_EXT_WAKE, &flags);
		bluesleep_tx_idle();
	} else {
		BT_DBG("Tx data during last period");
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL*HZ));
	}

	/* clear the incoming data flag */
	clear_bit(BT_TXDATA, &flags);

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}

/**
 * Schedules a tasklet to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t bluesleep_hostwake_isr(int irq, void *dev_id)
{
	/* schedule a tasklet to handle the change in the host wake line */
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

/**
 * Starts the Sleep-Mode Protocol on the Host.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int bluesleep_start(void)
{
	int retval;
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	/*bt sleep get uart port start*/
	bsi->uport = imx_uart_get_uart_port(1);
	if(bsi->uport == NULL){
		BT_ERR("get bt sleep uart port error");
		return -1;
	} else
		BT_ERR("get bt sleep uart port success");
	/*bt sleep get uart port end*/

	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return 0;
	}
	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		return -EBUSY;
	}

	/* start the timer */
	mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL*HZ));

	/* assert BT_WAKE */
            if (bsi->has_ext_wake == 1)
                    gpio_set_value(bsi->ext_wake, 1);
            set_bit(BT_EXT_WAKE, &flags);
    #if BT_ENABLE_IRQ_WAKE
	retval = enable_irq_wake(bsi->host_wake_irq);
	if (retval < 0) {
		BT_ERR("Couldn't enable BT_HOST_WAKE as wakeup interrupt");
		goto fail;
	}
    #endif
            set_bit(BT_PROTO, &flags);
            wake_lock(&bsi->wake_lock);
			BT_DBG("== bluesleep start ==");				// added by yjhuang 2013-08-08
	return 0;
fail:
	del_timer(&tx_timer);
	atomic_inc(&open_count);

	return retval;
}

/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void bluesleep_stop(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);
	if (!test_bit(BT_PROTO, &flags)) {

		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}
            /* assert BT_WAKE */
            if (bsi->has_ext_wake == 1)
                    gpio_set_value(bsi->ext_wake, 1);
            set_bit(BT_EXT_WAKE, &flags);

	del_timer(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		hsuart_power(1);
	}

	atomic_inc(&open_count);
            spin_unlock_irqrestore(&rw_lock, irq_flags);

    #if BT_ENABLE_IRQ_WAKE
            if (disable_irq_wake(bsi->host_wake_irq))
                    BT_ERR("Couldn't disable hostwake IRQ wakeup mode\n");
    #endif
            wake_lock_timeout(&bsi->wake_lock, HZ / 2);
}
/**
 * Read the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the
 * pin is high, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluepower_read_seq_btwake(struct seq_file *seq, void *offset)
{
            return seq_printf(seq, "btwake:%u\n",test_bit(BT_EXT_WAKE, &flags));
}

static int bluepower_open_seq_btwake(struct inode *inode,
					     struct file *file)
{
	return single_open(file, bluepower_read_seq_btwake,
			   NULL);
}
/**
 * Write the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int bluepower_write_seq_btwake(struct file *file, const char *buffer,
					      size_t count, loff_t * ppos)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}
            if (buf[0] == '0') {
                    if (bsi->has_ext_wake == 1)
                            gpio_set_value(bsi->ext_wake, 0);
                    clear_bit(BT_EXT_WAKE, &flags);
            } else if (buf[0] == '1') {
                    if (bsi->has_ext_wake == 1)
                            gpio_set_value(bsi->ext_wake, 1);
                    set_bit(BT_EXT_WAKE, &flags);
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

/**
 * Read the <code>BT_HOST_WAKE</code> GPIO pin value via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the pin
 * is high, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluepower_read_seq_hostwake(struct seq_file *seq, void *offset)
{
            return seq_printf(seq, "hostwake: %u\n", gpio_get_value(bsi->host_wake));
}


static int bluepower_open_seq_hostwake(struct inode *inode,
					     struct file *file)
{
	return single_open(file, bluepower_read_seq_hostwake,
			   NULL);
}
/**
 * Read the low-power status of the Host via the proc interface.
 * When this function returns, <code>page</code> contains a 1 if the Host
 * is asleep, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluesleep_read_seq_asleep(struct seq_file *seq, void *offset)
{
	unsigned int asleep;

	asleep = test_bit(BT_ASLEEP, &flags) ? 1 : 0;

	return seq_printf(seq, "asleep: %u\n", asleep);
}

static int bluesleep_open_seq_asleep(struct inode *inode,
					     struct file *file)
{
	return single_open(file, bluesleep_read_seq_asleep,
			   NULL);
}
/**
 * Read the low-power protocol being used by the Host via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the Host
 * is using the Sleep Mode Protocol, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluesleep_read_seq_proto(struct seq_file *seq, void *offset)
{
	unsigned int proto;

	proto = test_bit(BT_PROTO, &flags) ? 1 : 0;
	return seq_printf(seq, "proto: %u\n", proto);
}

static int bluesleep_open_seq_proto(struct inode *inode,
					     struct file *file)
{
	return single_open(file, bluesleep_read_seq_proto,
			   NULL);
}
/**
 * Modify the low-power protocol used by the Host via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int bluesleep_write_seq_proto(struct file *file, const char *buffer,
					      size_t count, loff_t * ppos)
{
	char proto;

			BT_DBG("== write bluesleep proto ==");					// added by yjhuang 2013-08-08
            if (count < 1)
                    return -EINVAL;

            if (copy_from_user(&proto, buffer, 1))
                    return -EFAULT;

		if (proto == '0'){
			BT_DBG("== call bluesleep stop ==");				// added by yjhuang 2013-08-08
			bluesleep_stop();
		}
		else{
			BT_DBG("== call bluesleep start ==");				// added by yjhuang 2013-08-08
			bluesleep_start();
		}

	/* claim that we wrote everything */
	return count;
}

    void bluesleep_setup_uart_port(struct platform_device *uart_dev)
    {
            bluesleep_uart_dev = uart_dev;
    }

//add api for bt sleep start
static int bluesleep_populate_dt_pinfo(struct platform_device *pdev)
{
	BT_DBG("");

	if (!bsi)
		return -ENOMEM;

	bsi->host_wake = of_get_named_gpio(pdev->dev.of_node,
					 "host-wake-gpio", 0);
	if (bsi->host_wake < 0) {
		BT_ERR("couldn't find host_wake gpio\n");
		return -ENODEV;
	}

	bsi->ext_wake = of_get_named_gpio(pdev->dev.of_node,
					 "ext-wake-gpio", 0);
	if (bsi->ext_wake < 0) {
		BT_ERR("couldn't find ext_wake gpio\n");
		return -ENODEV;
	}

	return 0;
}

static int bluesleep_populate_pinfo(struct platform_device *pdev)
{
	struct resource *res;

	BT_DBG("");

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_host_wake");
	if (!res) {
		BT_ERR("couldn't find host_wake gpio\n");
		return -ENODEV;
	}
	bsi->host_wake = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_ext_wake");
	if (!res) {
		BT_ERR("couldn't find ext_wake gpio\n");
		return -ENODEV;
	}
	bsi->ext_wake = res->start;

	return 0;
}
//add api for bt sleep end

    static int bluesleep_probe(struct platform_device *pdev)
{
	int ret;
	//struct resource *res;

	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

	//=========================================
	//add for bluetooth sleep start
	if (pdev->dev.of_node) {
		ret = bluesleep_populate_dt_pinfo(pdev);
		if (ret < 0) {
			BT_ERR("Failed to populate device tree info");
			goto free_bsi;
		}
	} else {
		ret = bluesleep_populate_pinfo(pdev);
		if (ret < 0) {
			BT_ERR("Failed to populate device info");
			goto free_bsi;
		}
	}

	//add to define get resource success
	bsi->has_ext_wake = 1;

	BT_DBG("host_wake_gpio: %d ext_wake_gpio: %d",
			bsi->host_wake, bsi->ext_wake);

	bsi->host_wake_irq = platform_get_irq_byname(pdev, "bt_host_wake");
	if (bsi->host_wake_irq < 0) {
		BT_ERR("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bsi;
	}

	bsi->irq_polarity = POLARITY_HIGH;	/* high edge (rasing edge) */

	//return 0;
	// add for bluetooth sleep end
	//=========================================

	/*res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_host_wake");
	if (!res) {
		BT_ERR("couldn't find host_wake gpio\n");
		ret = -ENODEV;
		goto free_bsi;
	}
	bsi->host_wake = res->start;*/

	ret = gpio_request(bsi->host_wake, "bt_host_wake");
	if (ret)
		goto free_bsi;
	ret = gpio_direction_input(bsi->host_wake);
            if (ret < 0) {
                    pr_err("gpio-keys: failed to configure input"
                                    " direction for GPIO %d, error %d\n",
                                    bsi->host_wake, ret);
                    gpio_free(bsi->host_wake);
                    goto free_bsi;
            }

	/*res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_ext_wake");

            if (!res)
                    bsi->has_ext_wake = 0;
            else
                    bsi->has_ext_wake = 1;*/

            if (bsi->has_ext_wake) {
                    //bsi->ext_wake = res->start;
	ret = gpio_request(bsi->ext_wake, "bt_ext_wake");
	if (ret)
		goto free_bt_host_wake;

                    /* configure ext_wake as output mode*/
                    ret = gpio_direction_output(bsi->ext_wake, 1);
                    if (ret < 0) {
                            pr_err("gpio-keys: failed to configure output"
                                    " direction for GPIO %d, error %d\n",
                                      bsi->ext_wake, ret);
                            gpio_free(bsi->ext_wake);
                            goto free_bt_host_wake;
                    }
            } else
                    set_bit(BT_EXT_WAKE, &flags);

           /* res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
                                                    "host_wake");
            if (!res) {
                    BT_ERR("couldn't find host_wake irq\n");
                    ret = -ENODEV;
                    goto free_bt_host_wake;
            }*/

            //bsi->host_wake_irq = res->start;
	if (bsi->host_wake_irq < 0) {
		BT_ERR("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bt_ext_wake;
	}
            //if (res->flags & IORESOURCE_IRQ_LOWEDGE)
                   // bsi->irq_polarity = POLARITY_LOW;/*low edge (falling edge)*/
           // else
                    //bsi->irq_polarity = POLARITY_HIGH;/*anything else*/

            wake_lock_init(&bsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");
            clear_bit(BT_SUSPEND, &flags);

            if (bsi->irq_polarity == POLARITY_LOW) {
                    ret = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
                                    IRQF_DISABLED | IRQF_TRIGGER_FALLING,
                                    "bluetooth hostwake", NULL);
            } else {
                    ret = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
                                    IRQF_DISABLED | IRQF_TRIGGER_RISING,
                                    "bluetooth hostwake", NULL);
            }
            if (ret  < 0) {
                    BT_ERR("Couldn't acquire BT_HOST_WAKE IRQ");
                    goto free_bt_ext_wake;
            }

	return 0;

free_bt_ext_wake:
	gpio_free(bsi->ext_wake);
free_bt_host_wake:
	gpio_free(bsi->host_wake);
free_bsi:
	kfree(bsi);
	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
            free_irq(bsi->host_wake_irq, NULL);
            gpio_free(bsi->host_wake);
            gpio_free(bsi->ext_wake);
            wake_lock_destroy(&bsi->wake_lock);
	kfree(bsi);
	return 0;
}


    static int bluesleep_resume(struct platform_device *pdev)
    {
            if (test_bit(BT_SUSPEND, &flags)) {
                    BT_DBG("bluesleep resuming...\n");
            if ((bsi->uport != NULL) &&
                    (gpio_get_value(bsi->host_wake) == bsi->irq_polarity)) {
                            BT_DBG("bluesleep resume form BT event...\n");
                            imx_uart_request_clock_on(bsi->uport);
                            imx_uart_set_mctrl(bsi->uport, TIOCM_RTS);
                    }
                    clear_bit(BT_SUSPEND, &flags);
            }
            return 0;
    }

    static int bluesleep_suspend(struct platform_device *pdev, pm_message_t state)
    {
            BT_DBG("bluesleep suspending...\n");
            set_bit(BT_SUSPEND, &flags);
            return 0;
    }

/** Device table */
static struct of_device_id bcmsleep_match_table[] = {
	{ .compatible = "imx,bcm4339_bluesleep" },
	{}
};

static struct platform_driver bluesleep_driver = {
            .probe = bluesleep_probe,
	.remove = bluesleep_remove,
            .suspend = bluesleep_suspend,
            .resume = bluesleep_resume,
	.driver = {
		.name = "bcmbtsleep",
		.owner = THIS_MODULE,
		.of_match_table = bcmsleep_match_table,
	},
};


static const struct file_operations btwake_devices_fileops = {
	.owner		= THIS_MODULE,
	.open		= bluepower_open_seq_btwake,
	.write		= bluepower_write_seq_btwake,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations hostwake_devices_fileops = {
	.owner		= THIS_MODULE,
	.open		= bluepower_open_seq_hostwake,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations proto_devices_fileops = {
	.owner		= THIS_MODULE,
	.open		= bluesleep_open_seq_proto,
	.write		= bluesleep_write_seq_proto,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations asleep_devices_fileops = {
	.owner		= THIS_MODULE,
	.open		= bluesleep_open_seq_asleep,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations lpm_devices_fileops = {
	.owner		= THIS_MODULE,
	.open		= bluesleep_open_seq_lpm,
	.write		= bluesleep_write_seq_lpm,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations btwrite_devices_fileops = {
	.owner		= THIS_MODULE,
	.open		= bluesleep_open_seq_btwrite,
	.write		= bluesleep_write_seq_btwrite,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init bluesleep_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	BT_ERR("bulesleep init");
       BT_INFO("BlueSleep Mode Driver Ver %s", VERSION);

	//platform_driver_register(&bluesleep_driver);


       retval = platform_driver_register(&bluesleep_driver);
	if (retval)
		return retval;


            if (bsi == NULL)
                    return 0;

    #if !BT_BLUEDROID_SUPPORT
	bluesleep_hdev = NULL;
    #endif

	proc_bluetooth_dir = proc_mkdir("bluetooth", NULL);
	proc_sleep_dir = proc_mkdir("sleep", proc_bluetooth_dir);

	/* Creating read/write "btwake" entry */
		ent = proc_create("btwake", S_IRGRP | S_IWGRP | S_IRUSR | S_IWUSR, proc_sleep_dir, &btwake_devices_fileops);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/sleep/btwake entry");
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	if (proc_create("hostwake",  S_IRGRP | S_IRUSR , proc_sleep_dir, &hostwake_devices_fileops) == NULL) {
		BT_ERR("Unable to create /proc/sleep/hostwake entry");
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
		ent = proc_create("proto", S_IRGRP | S_IWGRP | S_IRUSR | S_IWUSR, proc_sleep_dir, &proto_devices_fileops);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/sleep/proto entry");
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	if (proc_create("asleep", S_IRGRP | S_IRUSR, proc_sleep_dir, &asleep_devices_fileops) == NULL) {
		BT_ERR("Unable to create /proc/sleep/asleep entry");
		retval = -ENOMEM;
		goto fail;
	}

    #if BT_BLUEDROID_SUPPORT
            /* read/write lpm entries */
		ent = proc_create("lpm", S_IRGRP | S_IRUSR, proc_sleep_dir, &lpm_devices_fileops);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/sleep/lpm entry");
		retval = -ENOMEM;
		goto fail;
	}

            /* read/write proc entries */
		ent = proc_create("btwrite", S_IRGRP | S_IWGRP | S_IRUSR | S_IWUSR , proc_sleep_dir, &btwrite_devices_fileops);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/sleep/btwrite entry");
		retval = -ENOMEM;
		goto fail;
	}
    #endif

	flags = 0; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* Initialize timer */
	init_timer(&tx_timer);
	tx_timer.function = bluesleep_tx_timer_expire;
	tx_timer.data = 0;

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task, bluesleep_hostwake_task, 0);

            if (bsi->has_ext_wake == 1)
                    gpio_set_value(bsi->ext_wake, 1);
            set_bit(BT_EXT_WAKE, &flags);
    #if !BT_BLUEDROID_SUPPORT
	hci_register_notifier(&hci_event_nblock);
    #endif

	return 0;

fail:
    #if BT_BLUEDROID_SUPPORT
            remove_proc_entry("btwrite", proc_sleep_dir);
            remove_proc_entry("lpm", proc_sleep_dir);
    #endif
	remove_proc_entry("asleep", proc_sleep_dir);
	remove_proc_entry("proto", proc_sleep_dir);
	remove_proc_entry("hostwake", proc_sleep_dir);
	remove_proc_entry("btwake", proc_sleep_dir);
	remove_proc_entry("sleep", proc_bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	return retval;
}

/**
 * Cleans up the module.
 */
static void __exit bluesleep_exit(void)
{
            if (bsi == NULL)
                    return;

            /* assert bt wake */
            if (bsi->has_ext_wake == 1)
                    gpio_set_value(bsi->ext_wake, 1);
            set_bit(BT_EXT_WAKE, &flags);
            if (test_bit(BT_PROTO, &flags)) {
                    if (disable_irq_wake(bsi->host_wake_irq))
                            BT_ERR("Couldn't disable hostwake IRQ wakeup mode\n");
                    free_irq(bsi->host_wake_irq, NULL);
                    del_timer(&tx_timer);
                    if (test_bit(BT_ASLEEP, &flags))
                            hsuart_power(1);
            }

    #if !BT_BLUEDROID_SUPPORT
	hci_unregister_notifier(&hci_event_nblock);
    #endif
	platform_driver_unregister(&bluesleep_driver);

    #if BT_BLUEDROID_SUPPORT
            remove_proc_entry("btwrite", proc_sleep_dir);
            remove_proc_entry("lpm", proc_sleep_dir);
    #endif
	remove_proc_entry("asleep", proc_sleep_dir);
	remove_proc_entry("proto", proc_sleep_dir);
	remove_proc_entry("hostwake", proc_sleep_dir);
	remove_proc_entry("btwake", proc_sleep_dir);
	remove_proc_entry("sleep", proc_bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
}

module_init(bluesleep_init);
module_exit(bluesleep_exit);

MODULE_DESCRIPTION("Bluetooth Sleep Mode Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
