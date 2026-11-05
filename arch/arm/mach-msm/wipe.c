/*
 * RAM wipe for the MSM8974 hammerhead.
 *
 * The kernel is kexec'd with "wipe=" on the command line.  This runs
 * after mm_init(), before the scheduler starts, when the new kernel has
 * a clean memblock but no devices, filesystems or userspace yet.  Every
 * page that is not reserved by this kernel is therefore either free or
 * was used by the previous kernel, so zeroing it covers the old page
 * cache, slab, stacks and process memory.
 *
 * Reserved regions (the kernel image, page tables, DTB, initrd, TZ,
 * modem and ION carveouts) are skipped: they are either live or must
 * not be touched.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/memblock.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/preempt.h>
#include <linux/irqflags.h>

#include <asm/cacheflush.h>
#include <asm/barrier.h>

#include <mach/msm_iomap.h>
#include <mach/scm.h>

#define SCM_IO_DISABLE_PMIC_ARBITER	1

#define WIPE_MODE_NONE		0
#define WIPE_MODE_ALL		1
#define WIPE_MODE_TEST		2

static int wipe_mode;
static int wipe_verify;
static unsigned long wipe_test_mb;

static int __init wipe_setup(char *str)
{
	if (!str || !*str)
		return 1;

	if (!strcmp(str, "1") || !strcmp(str, "all")) {
		wipe_mode = WIPE_MODE_ALL;
		return 1;
	}
	if (!strcmp(str, "verify")) {
		wipe_mode = WIPE_MODE_ALL;
		wipe_verify = 1;
		return 1;
	}
	if (!strncmp(str, "test", 4)) {
		wipe_mode = WIPE_MODE_TEST;
		if (str[4] == ':')
			wipe_test_mb = simple_strtoul(str + 5, NULL, 0);
		return 1;
	}

	pr_err("wipe: unknown mode '%s'\n", str);
	return 1;
}
early_param("wipe", wipe_setup);

static void __init wipe_page(phys_addr_t phys)
{
	struct page *page = pfn_to_page(phys >> PAGE_SHIFT);
	void *v = kmap_atomic(page);

	memset(v, 0, PAGE_SIZE);
	kunmap_atomic(v);
}

static void __init wipe_phys(phys_addr_t start, phys_addr_t end)
{
	phys_addr_t low_end = __pa(high_memory);
	phys_addr_t p, e;

	/* Only full pages: a partial page at a free/reserved boundary
	 * could contain reserved data. */
	p = PAGE_ALIGN(start);
	e = end & PAGE_MASK;
	if (p >= e)
		return;

	if (p < low_end) {
		phys_addr_t le = min(e, low_end);

		memset(__va(p), 0, le - p);
		p = le;
	}
	while (p < e) {
		wipe_page(p);
		p += PAGE_SIZE;
	}
}

/* Wipe the free part of [start, end), stopping after `limit' bytes when
 * limit is non-zero.  Returns the number of free bytes consumed. */
static phys_addr_t __init wipe_free_range(phys_addr_t start, phys_addr_t end,
					  phys_addr_t limit)
{
	struct memblock_region *r;
	phys_addr_t cur = start;
	phys_addr_t done = 0;

	for_each_memblock(reserved, r) {
		phys_addr_t rs = r->base;
		phys_addr_t re = r->base + r->size;

		if (re <= cur || rs >= end)
			continue;
		if (rs > cur) {
			phys_addr_t a = cur;
			phys_addr_t b = min(rs, end);

			if (limit && b - a > limit)
				b = a + limit;
			wipe_phys(a, b);
			done += b - a;
			if (limit) {
				limit -= b - a;
				if (!limit)
					return done;
			}
		}
		if (re > cur)
			cur = re;
		if (cur >= end)
			break;
	}
	if (cur < end) {
		phys_addr_t a = cur;
		phys_addr_t b = end;

		if (limit && b - a > limit)
			b = a + limit;
		wipe_phys(a, b);
		done += b - a;
	}
	return done;
}

static void __init wipe_all(void)
{
	struct memblock_region *r;
	phys_addr_t done = 0;
	phys_addr_t remaining = (phys_addr_t)wipe_test_mb << 20;

	for_each_memblock(memory, r) {
		phys_addr_t got;

		got = wipe_free_range(r->base, r->base + r->size,
				      remaining);
		done += got;
		if (remaining) {
			if (got >= remaining)
				break;
			remaining -= got;
		}
	}
	pr_info("wipe: %llu MB of free RAM wiped\n",
		(unsigned long long)(done >> 20));
}

static int __init wipe_check_page(phys_addr_t phys)
{
	struct page *page = pfn_to_page(phys >> PAGE_SHIFT);
	const u8 *v = kmap_atomic(page);
	int i, bad = 0;

	for (i = 0; i < PAGE_SIZE; i++) {
		if (v[i]) {
			bad = 1;
			break;
		}
	}
	kunmap_atomic((void *)v);
	return bad;
}

static int __init wipe_check_range(phys_addr_t start, phys_addr_t end)
{
	phys_addr_t p = PAGE_ALIGN(start);
	phys_addr_t e = end & PAGE_MASK;
	int bad = 0;

	if (p >= e)
		return 0;
	if (wipe_check_page(p))
		bad++;
	if (e - p > PAGE_SIZE && wipe_check_page(e - PAGE_SIZE))
		bad++;
	if (e - p > 2 * PAGE_SIZE &&
	    wipe_check_page((p + e) / 2 & PAGE_MASK))
		bad++;
	return bad;
}

static void __init wipe_verify_all(void)
{
	struct memblock_region *r;
	int bad = 0;

	for_each_memblock(memory, r) {
		phys_addr_t start = r->base;
		phys_addr_t end = r->base + r->size;
		struct memblock_region *res;
		phys_addr_t cur = start;

		for_each_memblock(reserved, res) {
			phys_addr_t rs = res->base;
			phys_addr_t re = res->base + res->size;

			if (re <= cur || rs >= end)
				continue;
			if (rs > cur)
				bad += wipe_check_range(cur, min(rs, end));
			if (re > cur)
				cur = re;
			if (cur >= end)
				break;
		}
		if (cur < end)
			bad += wipe_check_range(cur, end);
	}
	if (bad)
		pr_err("wipe: verification FAILED (%d sampled pages non-zero)\n",
		       bad);
	else
		pr_info("wipe: verification OK\n");
}

static void __init wipe_power_off(void)
{
	pr_crit("wipe: powering off\n");

#ifdef CONFIG_MSM_RESTART_V2
	/* Avoid an SPMI transaction when PS_HOLD is lowered. */
	scm_call_atomic1(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER, 0);
	__raw_writel(0, MSM_MPM2_PSHOLD_BASE);
#else
	__raw_writel(0, MSM_TLMM_BASE + 0x820);
#endif
	mdelay(10000);
}

void __init wipe_main(void)
{
	if (!wipe_mode)
		return;

	local_irq_disable();
	local_fiq_disable();
	preempt_disable();

	pr_crit("wipe: start (mode %d, test %lu MB, verify %d)\n",
		wipe_mode, wipe_test_mb, wipe_verify);

	if (num_online_cpus() > 1)
		smp_send_stop();

	wipe_all();

	flush_cache_all();
	dsb();
	isb();

	if (wipe_verify)
		wipe_verify_all();

	if (wipe_mode == WIPE_MODE_TEST) {
		pr_crit("wipe: test complete, continuing boot\n");
		preempt_enable();
		return;
	}

	wipe_power_off();
	while (1)
		barrier();
}
