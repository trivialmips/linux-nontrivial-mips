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

void __init plat_mem_setup(void)
{
	__dt_setup_arch(__dtb_start);
	strlcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);
}

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	unflatten_and_copy_device_tree();
}

void __init plat_time_init(void) {
    struct device_node *np;
    struct clk *clk;
    of_clk_init(NULL);
    np = of_get_cpu_node(0, NULL);
    if (!np) {
        pr_err("Failed to get CPU node\n");
        return;
    }
    clk = of_clk_get(np, 0);
    if (IS_ERR(clk)) {
        pr_err("Failed to get CPU clock: %ld\n", PTR_ERR(clk));
        return;
    }

    // CP0 Count register halves the frequency of CPU
    mips_hpt_frequency = clk_get_rate(clk) / 2;
    clk_put(clk);
}


const char *get_system_type(void)
{
    const char *str;
    int err;

    err = of_property_read_string_index(of_root, "compatible", 0, &str);
    if (!err)
        return str;

    return "Unknown";
}

static int __init plat_of_setup(void)
{
	if (!of_have_populated_dt())
		panic("Device tree not present");

	if (of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL))
		panic("Failed to populate DT");

	return 0;
}
arch_initcall(plat_of_setup);

