/*
 * Copyright (c) 2018 Marvell
 * Copyright (c) 2018 Lexmark International, Inc.
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NOTE: This driver implements the GICv1 and GICv2 interfaces.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/sys/barrier.h>

static const uint64_t cpu_mpid_list[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_PATH(cpus), DT_REG_ADDR, (,))
};

BUILD_ASSERT(ARRAY_SIZE(cpu_mpid_list) >= CONFIG_MP_MAX_NUM_CPUS,
		"The count of CPU Cores nodes in dts is less than CONFIG_MP_MAX_NUM_CPUS\n");

static void xlnx_arm_gic_irq_enable(unsigned int irq)
{
	uint8_t cpu_mask = 0;
	uint8_t *itarget_reg;
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	itarget_reg = (uint8_t *)(GICD_ITARGETSRn);
	itarget_reg += irq;
	cpu_mask = BIT(cpu_mpid_list[0]);

	sys_write8(cpu_mask, (mem_addr_t)itarget_reg);

	sys_write32((1 << int_off), (GICD_ISENABLERn + int_grp * 4));
}

static void xlnx_arm_gic_irq_disable(unsigned int irq)
{
	int int_grp, int_off;
	uint8_t cpu_mask = 0, reg_val = 0;
	uint8_t *itarget_reg;

	int_grp = irq / 32;
	int_off = irq % 32;

	itarget_reg = (uint8_t *)(GICD_ITARGETSRn);
	itarget_reg += irq;
	cpu_mask = BIT(cpu_mpid_list[0]);

	reg_val = sys_read8((mem_addr_t)itarget_reg);
	reg_val &= (~cpu_mask);

	sys_write8(reg_val, (mem_addr_t)itarget_reg);
	sys_write32((1 << int_off), (GICD_ICENABLERn + int_grp * 4));
}

static bool xlnx_arm_gic_irq_is_enabled(unsigned int irq)
{
	int int_grp, int_off;
	unsigned int enabler;

	int_grp = irq / 32;
	int_off = irq % 32;

	enabler = sys_read32(GICD_ISENABLERn + int_grp * 4);

	return (enabler & (1 << int_off)) != 0;
}

static void xlnx_arm_gic_irq_set_priority(
	unsigned int irq, unsigned int prio, uint32_t flags)
{
	int int_grp, int_off;
	uint32_t val;

	/* Set priority */
	sys_write8(prio & 0xff, GICD_IPRIORITYRn + irq);

	/* Set interrupt type */
	int_grp = (irq / 16) * 4;
	int_off = (irq % 16) * 2;

	val = sys_read32(GICD_ICFGRn + int_grp);
	val &= ~(GICD_ICFGR_MASK << int_off);
	if (flags & IRQ_TYPE_EDGE) {
		val |= (GICD_ICFGR_TYPE << int_off);
	}

	sys_write32(val, GICD_ICFGRn + int_grp);
}

static unsigned int xlnx_arm_gic_get_active(void)
{
	int irq;

	irq = sys_read32(GICC_IAR) & 0x3ff;
	return irq;
}

static void xlnx_arm_gic_eoi(unsigned int irq)
{
	/*
	 * Ensure the write to peripheral registers are *complete* before the write
	 * to GIC_EOIR.
	 *
	 * Note: The completion guarantee depends on various factors of system design
	 * and the barrier is the best core can do by which execution of further
	 * instructions waits till the barrier is alive.
	 */
	barrier_dsync_fence_full();

	/* set to inactive */
	sys_write32(irq, GICC_EOIR);
}

static void xlnx_gic_dist_init(void)
{
	unsigned int gic_irqs, i;
	uint8_t cpu_mask = 0;
	uint32_t reg_val, val;

	gic_irqs = sys_read32(GICD_TYPER) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	if (gic_irqs > 1020) {
		gic_irqs = 1020;
	}

	/*
	 * Enable all global interrupts distributing to CPUs listed
	 * in dts with the count of arch_num_cpus().
	 */
	unsigned int num_cpus = arch_num_cpus();

	for (i = 0; i < num_cpus; i++) {
		cpu_mask |= BIT(cpu_mpid_list[i]);
	}
	reg_val = cpu_mask | (cpu_mask << 8) | (cpu_mask << 16)
		| (cpu_mask << 24);

	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 4) {
		val = sys_read32(GICD_ITARGETSRn + i);
		val &= ~(reg_val);
		sys_write32(val, GICD_ITARGETSRn + i);
	}

	if (cpu_mpid_list[0] != 0)
		return;

	/*
	 * Disable the forwarding of pending interrupts
	 * from the Distributor to the CPU interfaces
	 */
	sys_write32(0, GICD_CTLR);

	/*
	 * Set all global interrupts to be level triggered, active low.
	 */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 16) {
		sys_write32(0, GICD_ICFGRn + i / 4);
	}

	/*  Set priority on all global interrupts.   */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 4) {
		sys_write32(0, GICD_IPRIORITYRn + i);
	}

	/* Set all interrupts to group 0 */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 32) {
		sys_write32(0, GICD_IGROUPRn + i / 8);
	}

	/*
	 * Disable all interrupts.  Leave the PPI and SGIs alone
	 * as these enables are banked registers.
	 */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 32) {
#ifndef CONFIG_GIC_V1
		sys_write32(0xffffffff, GICD_ICACTIVERn + i / 8);
#endif
		sys_write32(0xffffffff, GICD_ICENABLERn + i / 8);
	}

	/*
	 * Enable the forwarding of pending interrupts
	 * from the Distributor to the CPU interfaces
	 */
	sys_write32(1, GICD_CTLR);
}

static void xlnx_gic_cpu_init(void)
{
	int i;
	uint32_t val;

	/*
	 * Deal with the banked PPI and SGI interrupts - disable all
	 * PPI interrupts, ensure all SGI interrupts are enabled.
	 */
#ifndef CONFIG_GIC_V1
	sys_write32(0xffffffff, GICD_ICACTIVERn);
#endif
	sys_write32(0xffff0000, GICD_ICENABLERn);
	sys_write32(0x0000ffff, GICD_ISENABLERn);

	/*
	 * Set priority on PPI and SGI interrupts
	 */
	for (i = 0; i < 32; i += 4) {
		sys_write32(0xa0a0a0a0, GICD_IPRIORITYRn + i);
	}

	sys_write32(0xf0, GICC_PMR);

	/*
	 * Enable interrupts and signal them using the IRQ signal.
	 */
	val = sys_read32(GICC_CTLR);
#ifndef CONFIG_GIC_V1
	val &= ~GICC_CTLR_BYPASS_MASK;
#endif
	val |= GICC_CTLR_ENABLE_MASK;
	sys_write32(val, GICC_CTLR);
}

#define GIC_PARENT_IRQ 0
#define GIC_PARENT_IRQ_PRI 0
#define GIC_PARENT_IRQ_FLAGS 0

/**
 * @brief Initialize the GIC device driver
 */
static int xlnx_arm_gic_init(void)
{
	/* Init of Distributor interface registers */
	xlnx_gic_dist_init();

	/* Init CPU interface registers */
	xlnx_gic_cpu_init();

	return 0;
}

void z_soc_irq_init(void)
{
	xlnx_arm_gic_init();
}

void z_soc_irq_enable(unsigned int irq)
{
	xlnx_arm_gic_irq_enable(irq);
}

void z_soc_irq_disable(unsigned int irq)
{
	xlnx_arm_gic_irq_disable(irq);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	return xlnx_arm_gic_irq_is_enabled(irq);
}

void z_soc_irq_priority_set(
	unsigned int irq, unsigned int prio, unsigned int flags)
{
	xlnx_arm_gic_irq_set_priority(irq, prio, flags);
}

unsigned int z_soc_irq_get_active(void)
{
	return xlnx_arm_gic_get_active();
}

void z_soc_irq_eoi(unsigned int irq)
{
	xlnx_arm_gic_eoi(irq);
}

//SYS_INIT(z_soc_irq_init, PRE_KERNEL_1, 40);
