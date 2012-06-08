/*
 *  (C) Copyright 2011
 *  Ant Micro <www.antmicro.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/sizes.h>
#include "tegra2-common.h"

#define CONFIG_TEGRA_CLOCK_13MHZ // override clock to 13 MHz

/* High-level configuration options */
#define TEGRA2_SYSMEM			"mem=384M@0M nvmem=128M@384M mem=512M@512M"
#define V_PROMPT			"Colibri Tegra2 # "
#define CONFIG_TEGRA2_BOARD_STRING	"TORADEX Colibri Tegra2"

/* Board-specific serial config */
#define CONFIG_SERIAL_MULTI
#define CONFIG_TEGRA2_ENABLE_UARTA
#define CONFIG_SYS_NS16550_COM1		NV_PA_APB_UARTA_BASE
//#define CONFIG_SYS_NS16550_COM2		NV_PA_APB_UARTA_BASE

#define CONFIG_MACH_TYPE		MACH_TYPE_COLIBRI_TEGRA2
#define CONFIG_SYS_BOARD_ODMDATA	0x100d8011 /* lp1, 256MB */

#define CONFIG_SYS_TEXT_BASE		0x00108000

#define CONFIG_BOARD_EARLY_INIT_F

#undef DEBUG

#define CONFIG_CMD_NET
#define CONFIG_NET_MULTI
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP

#define CONFIG_DOS_PARTITION            1
#define CONFIG_EFI_PARTITION            1


// USB

#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_TEGRA
#define CONFIG_CMD_USB

//#define USBADDR1 0xC5000000 // USB1: USB-device
#define USBADDR2 0xc5004000   // USB2: built-in ASIX over ULPI
#define USBADDR3 0xc5008000   // USB3: USB-HOST

#ifdef CONFIG_USB_EHCI_TEGRA
#define USB_EHCI_TEGRA_BASE_ADDR         USBADDR2
#define CONFIG_USB_EHCI_DATA_ALIGN       4
#define CONFIG_EHCI_IS_TDI
#define CONFIG_USB_EHCI_TXFIFO_THRESH	10
#endif

#endif /* __CONFIG_H */
