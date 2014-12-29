/* Copyright (c) 2009-2015, FIC Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
/*
 * WiFi AmPak(bcm4330) Power Switch Module
 * controls power to external WiFi device
 * with interface to power management device
 */

#ifndef __BCM_BT_GPIO__
#define __BCM_BT_GPIO__

void wifi_power(int nSwitch);
int get_wifi_host_wakeup_gpio(void);
int get_wifi_power_gpio(void);

#endif
