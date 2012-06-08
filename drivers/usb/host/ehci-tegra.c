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

#include <common.h>
#include <usb.h>
#include "ehci.h"
#include "ehci-core.h"

int ehci_hcd_init(void) {
    hccr = (struct ehci_hccr *)(USB_EHCI_TEGRA_BASE_ADDR + 0x100);
    hcor = (struct ehci_hcor *)((uint32_t) hccr + HC_LENGTH(ehci_readl(&hccr->cr_capbase)));
    return 0;
}

int ehci_hcd_stop(void) {
	ehci_writel(&hcor->or_usbcmd, 0);
	udelay(800);
	ehci_writel(&hcor->or_usbcmd, 2);
	udelay(800);
	return 0;
}
