/*
 * NaiveMIPS interrupt controller setup
 *
 * Copyright (C) 2017 Tsinghua Univ.
 * Author: Yuxiang Zhang
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/clk-provider.h>
#include <linux/irqchip.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <asm/prom.h>
#include <asm/irq_cpu.h>
#include <asm/time.h>
#include <asm/mips-cps.h>
#include <asm/irq_cpu.h>

void __init arch_init_irq(void)
{
	unsigned int sr;
	struct device_node *intc_node;
	pr_cont("arch_init_irq with ebase: 0x%x\n", (u32) ebase);
	sr = set_c0_status(ST0_BEV);
	write_c0_ebase((u32)ebase);
	write_c0_status(sr);
	intc_node = of_find_compatible_node(NULL, NULL,
			"mti,cpu-interrupt-controller");
	of_node_put(intc_node);
	irqchip_init();
}

unsigned int get_c0_compare_int(void) {
	return CP0_LEGACY_COMPARE_IRQ;
}

IRQCHIP_DECLARE(mips_cpu_intc, "mti,cpu-interrupt-controller",
		mips_cpu_irq_of_init);
