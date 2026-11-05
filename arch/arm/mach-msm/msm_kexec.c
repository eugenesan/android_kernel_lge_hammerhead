/* Copyright (c) 2010, The Linux Foundation. All rights reserved.
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

#include <linux/kexec.h>
#include <linux/io.h>

#if defined(CONFIG_MSM_WATCHDOG) || defined(CONFIG_MSM_WATCHDOG_V2)
#include <mach/msm_iomap.h>
#endif

#ifdef CONFIG_MSM_WATCHDOG
#define WDT0_EN        (MSM_TMR_BASE + 0x40)
#endif

#ifdef CONFIG_MSM_WATCHDOG_V2
extern void msm_watchdog_v2_disable(void);
#endif

void arch_kexec(void)
{
#ifdef CONFIG_MSM_WATCHDOG
	/* Prevent watchdog from resetting SoC */
	writel(0, WDT0_EN);
	pr_crit("KEXEC: MSM Watchdog Exit - Deactivated\n");
#endif
#ifdef CONFIG_MSM_WATCHDOG_V2
	/* The V2 watchdog keeps running across the jump and would reset
	 * the SoC while the new kernel is still booting/wiping. */
	msm_watchdog_v2_disable();
#endif
	return;
}
