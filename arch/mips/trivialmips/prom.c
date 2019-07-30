#include <linux/of_fdt.h>
#include <asm/prom.h>

#if IS_ENABLED(CONFIG_TRIVIALMIPS_NSCSCC)
#define EARLY_PRINTK_UART_BASE 0xbfd03000
#define TRIVIALMIPS_UART_OUTB(addr, byte) writeb((byte), (uint32_t *)(EARLY_PRINTK_UART_BASE + addr))
#endif

void prom_putchar(char c)
{
#if IS_ENABLED(CONFIG_TRIVIALMIPS_NSCSCC)
    while( (readl((uint32_t *)(EARLY_PRINTK_UART_BASE + 0x14)) & 0x40) == 0 );
    TRIVIALMIPS_UART_OUTB(0x00, c & 0xFF);
#endif
}

void __init prom_init(void)
{
#if IS_ENABLED(CONFIG_TRIVIALMIPS_NSCSCC)
    // Turn on the FIFO (8 bytes)
    TRIVIALMIPS_UART_OUTB(0x08, 0x81);
    // Set speed; requires DLAB latch
    TRIVIALMIPS_UART_OUTB(0x0C, 0x80);
    TRIVIALMIPS_UART_OUTB(0x04, 0x00);
    TRIVIALMIPS_UART_OUTB(0x00, 0x36);

    // 8 data bits, 1 stop bit, parity off; turn off DLAB latch
    TRIVIALMIPS_UART_OUTB(0x0C, 0x03);

    // No modem controls
    TRIVIALMIPS_UART_OUTB(0x10, 0);
    // Enable rcv interrupts
    TRIVIALMIPS_UART_OUTB(0x04, 0x01);
#endif
}

void __init prom_free_prom_memory(void)
{
}

