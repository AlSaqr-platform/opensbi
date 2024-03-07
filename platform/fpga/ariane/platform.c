/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019 FORTH-ICS/CARV
 *				Panagiotis Peristerakis <perister@ics.forth.gr>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/fdt/fdt_fixup.h>
#include <sbi_utils/ipi/aclint_mswi.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/serial/uart8250.h>
#include <sbi_utils/timer/aclint_mtimer.h>

#define ARIANE_UART_ADDR			0x10000000
#define ARIANE_UART_FREQ			40000000
#define ARIANE_UART_BAUDRATE			115200
#define ARIANE_UART_REG_SHIFT			2
#define ARIANE_UART_REG_WIDTH			4
#define ARIANE_PLIC_ADDR			0xc000000
#define ARIANE_PLIC_NUM_SOURCES			3
#define ARIANE_HART_COUNT			2
#define ARIANE_CLINT_ADDR			0x2000000
#define ARIANE_ACLINT_MTIMER_FREQ		1000000
#define ARIANE_ACLINT_MSWI_ADDR			(ARIANE_CLINT_ADDR + \
						 CLINT_MSWI_OFFSET)
#define ARIANE_ACLINT_MTIMER_ADDR		(ARIANE_CLINT_ADDR + \
						 CLINT_MTIMER_OFFSET)

static struct platform_uart_data uart = {
	ARIANE_UART_ADDR,
	ARIANE_UART_FREQ,
	ARIANE_UART_BAUDRATE,
};
static struct plic_data plic = {
	.addr = ARIANE_PLIC_ADDR,
	.num_src = ARIANE_PLIC_NUM_SOURCES,
};

static struct aclint_mswi_data mswi = {
	.addr = ARIANE_ACLINT_MSWI_ADDR,
	.size = ACLINT_MSWI_SIZE,
	.first_hartid = 0,
	.hart_count = ARIANE_HART_COUNT,
};

static struct aclint_mtimer_data mtimer = {
	.mtime_freq = ARIANE_ACLINT_MTIMER_FREQ,
	.mtime_addr = ARIANE_ACLINT_MTIMER_ADDR +
		      ACLINT_DEFAULT_MTIME_OFFSET,
	.mtime_size = ACLINT_DEFAULT_MTIME_SIZE,
	.mtimecmp_addr = ARIANE_ACLINT_MTIMER_ADDR +
			 ACLINT_DEFAULT_MTIMECMP_OFFSET,
	.mtimecmp_size = ACLINT_DEFAULT_MTIMECMP_SIZE,
	.first_hartid = 0,
	.hart_count = ARIANE_HART_COUNT,
	.has_64bit_mmio = TRUE,
};

/*
 * Ariane platform early initialization.
 */
static int ariane_early_init(bool cold_boot)
{
	void *fdt;
	struct platform_uart_data uart_data;
	struct plic_data plic_data;
	unsigned long aclint_freq;
	uint64_t clint_addr;
	int rc;

	if (!cold_boot)
		return 0;
	fdt = fdt_get_address();

	rc = fdt_parse_uart8250(fdt, &uart_data, "ns16550");
	if (!rc)
		uart = uart_data;

	rc = fdt_parse_plic(fdt, &plic_data, "riscv,plic0");
	if (!rc)
		plic = plic_data;

	rc = fdt_parse_timebase_frequency(fdt, &aclint_freq);
	if (!rc)
		mtimer.mtime_freq = aclint_freq;

	rc = fdt_parse_compat_addr(fdt, &clint_addr, "riscv,clint0");
	if (!rc) {
		mswi.addr = clint_addr;
		mtimer.mtime_addr = clint_addr + CLINT_MTIMER_OFFSET +
				    ACLINT_DEFAULT_MTIME_OFFSET;
		mtimer.mtimecmp_addr = clint_addr + CLINT_MTIMER_OFFSET +
				    ACLINT_DEFAULT_MTIMECMP_OFFSET;
	}

	return 0;
}

/*
 * Ariane platform final initialization.
 */
static int ariane_final_init(bool cold_boot)
{
	void *fdt;

	if (!cold_boot)
		return 0;

	fdt = fdt_get_address();
	fdt_fixups(fdt);

  /* Enable perf tracking */
	csr_write(CSR_MCOUNTEREN,-1);
  csr_write(CSR_HCOUNTEREN,-1);
  csr_write(CSR_SCOUNTEREN,-1);
  csr_write(CSR_MCOUNTINHIBIT,0);
  csr_write(CSR_MHPMEVENT3,1); // icache miss
  csr_write(CSR_MHPMEVENT4,2); // dcache miss
  csr_write(CSR_MHPMEVENT5,5); // load event
  csr_write(CSR_MHPMEVENT6,6); // store event
  csr_write(CSR_MHPMEVENT7,15); // if empty
  csr_write(CSR_MHPMEVENT8,22); // stall

  csr_write(CSR_MHPMEVENT9,23); // snoop read once
  csr_write(CSR_MHPMEVENT10,24); // snoop read shared
  csr_write(CSR_MHPMEVENT11,25); // snoop read clean
  csr_write(CSR_MHPMEVENT12,26); // snoop read no shared
  csr_write(CSR_MHPMEVENT13,27); // snoop read unique
  csr_write(CSR_MHPMEVENT14,28); // snoop clean shared
  csr_write(CSR_MHPMEVENT15,29); // snoop clean invalid
  csr_write(CSR_MHPMEVENT16,30); // snoop clean unique
  csr_write(CSR_MHPMEVENT17,31); // snoop make invalid
  csr_write(CSR_MHPMEVENT18,32); // dc hit
  csr_write(CSR_MHPMEVENT19,33); // dc write hit unique
  csr_write(CSR_MHPMEVENT20,34); // dc write hit shared
  csr_write(CSR_MHPMEVENT21,35); // dc write miss
  csr_write(CSR_MHPMEVENT22,36); // dc clean invalid hit
  csr_write(CSR_MHPMEVENT23,37); // dc clean invalid miss
  csr_write(CSR_MHPMEVENT24,38); // dc flushing
  
  return 0;
}

/*
 * Initialize the ariane console.
 */
static int ariane_console_init(void)
{
	return uart8250_init(ARIANE_UART_ADDR,
			     ARIANE_UART_FREQ,
			     ARIANE_UART_BAUDRATE,
			     ARIANE_UART_REG_SHIFT,
			     ARIANE_UART_REG_WIDTH);
}

static int plic_ariane_warm_irqchip_init(int m_cntx_id, int s_cntx_id)
{
	size_t i, ie_words = ARIANE_PLIC_NUM_SOURCES / 32 + 1;

	/* By default, enable all IRQs for M-mode of target HART */
	if (m_cntx_id > -1) {
		for (i = 0; i < ie_words; i++)
			plic_set_ie(&plic, m_cntx_id, i, 1);
	}
	/* Enable all IRQs for S-mode of target HART */
	if (s_cntx_id > -1) {
		for (i = 0; i < ie_words; i++)
			plic_set_ie(&plic, s_cntx_id, i, 1);
	}
	/* By default, enable M-mode threshold */
	if (m_cntx_id > -1)
		plic_set_thresh(&plic, m_cntx_id, 1);
	/* By default, disable S-mode threshold */
	if (s_cntx_id > -1)
		plic_set_thresh(&plic, s_cntx_id, 0);

	return 0;
}

/*
 * Initialize the ariane interrupt controller for current HART.
 */
static int ariane_irqchip_init(bool cold_boot)
{
	u32 hartid = current_hartid();
	int ret;

	if (cold_boot) {
		ret = plic_cold_irqchip_init(&plic);
		if (ret)
			return ret;
	}
	return plic_ariane_warm_irqchip_init(2 * hartid, 2 * hartid + 1);
}

/*
 * Initialize IPI for current HART.
 */
static int ariane_ipi_init(bool cold_boot)
{
	int ret;

	if (cold_boot) {
		ret = aclint_mswi_cold_init(&mswi);
		if (ret)
			return ret;
	}

	return aclint_mswi_warm_init();
}

/*
 * Initialize ariane timer for current HART.
 */
static int ariane_timer_init(bool cold_boot)
{
	int ret;

	if (cold_boot) {
		ret = aclint_mtimer_cold_init(&mtimer, NULL);
		if (ret)
			return ret;
	}

	return aclint_mtimer_warm_init();
}

/*
 * Platform descriptor.
 */
const struct sbi_platform_operations platform_ops = {
	.early_init = ariane_early_init,
	.final_init = ariane_final_init,
	.console_init = ariane_console_init,
	.irqchip_init = ariane_irqchip_init,
	.ipi_init = ariane_ipi_init,
	.timer_init = ariane_timer_init,
};

const struct sbi_platform platform = {
	.opensbi_version = OPENSBI_VERSION,
	.platform_version = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name = "ARIANE RISC-V",
	.features = SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count = ARIANE_HART_COUNT,
	.hart_stack_size = SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr = (unsigned long)&platform_ops
};
