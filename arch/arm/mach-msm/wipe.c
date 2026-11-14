/*
 * RAM wipe for the MSM8974 hammerhead.
 *
 * The kernel is kexec'd with "wipe=" on the command line.  This runs at
 * the top of mem_init(), after the bootmem bitmap and mem_map exist but
 * before free_all_bootmem() hands the free pages to the page allocator.
 * The bootmem bitmap is therefore the authoritative record of which
 * low-memory pages are free; highmem is not covered by bootmem and is
 * walked through memblock instead.
 *
 * This is deliberately not "wipe everything except _stext.._end": the
 * page tables, mem_map and other bootmem allocations live outside the
 * kernel image and would crash the machine if they were zeroed.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/memblock.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/bootmem.h>
#include <linux/highmem.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/irqflags.h>
#include <linux/reboot.h>

#include <asm/cacheflush.h>
#include <asm/barrier.h>
#include <asm/highmem.h>
#include <asm/memory.h>

#include <mach/msm_iomap.h>
#include <mach/scm.h>

#define SCM_IO_DISABLE_PMIC_ARBITER	1

#define WIPE_MODE_NONE		0
#define WIPE_MODE_ALL		1
#define WIPE_MODE_TEST		2
#define WIPE_MODE_INFO		3

#ifdef CONFIG_MEMORY_WIPE_ON_POWEROFF
/*
 * A poweroff writes this marker into the spare tail of the persistent RAM
 * the bootloader preserves across a warm reset (the ram console uses the
 * first 256 KB; the whole 1 MB is reserved).  The next boot reads it at
 * mem_init(), wipes, and powers off before userspace exists.
 *
 * The marker is RAM only: if the recovery boot does not happen, it is gone
 * with the RAM (and so are the secrets).  Nothing is written to NAND.
 */
#define KRUX_MARKER_PHYS	0x100F0000UL
#define KRUX_MARKER_MAGIC	0x4b525857	/* "KRXW" */

struct krux_wipe_marker {
	u32 magic;
	u32 magic_inv;
	u32 seq;
	u32 crc;
};

int wipe_from_marker;

static struct krux_wipe_marker *krux_marker(void)
{
	return (struct krux_wipe_marker *)__va(KRUX_MARKER_PHYS);
}

static u32 krux_crc32(const void *buf, size_t len)
{
	const u8 *p = buf;
	u32 crc = 0xffffffff;
	int i;

	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
	}
	return ~crc;
}

static bool krux_marker_valid(void)
{
	struct krux_wipe_marker *m = krux_marker();

	return m->magic == KRUX_MARKER_MAGIC &&
	       m->magic_inv == ~KRUX_MARKER_MAGIC &&
	       m->crc == krux_crc32(m, offsetof(struct krux_wipe_marker, crc));
}

static void krux_marker_write(u32 magic)
{
	struct krux_wipe_marker *m = krux_marker();

	m->magic = magic;
	m->magic_inv = ~magic;
	m->seq++;
	m->crc = krux_crc32(m, offsetof(struct krux_wipe_marker, crc));
	/* The reset does not flush the cache; push it to DRAM ourselves. */
	flush_cache_all();
}

void krux_wipe_reboot(void)
{
	pr_crit("wipe: marking RAM and restarting into recovery\n");
	krux_marker_write(KRUX_MARKER_MAGIC);
	kernel_restart("recovery");
}
#endif

static int wipe_mode;
static int wipe_verify;
static unsigned long wipe_test_mb;

/* 0 means "no limit". */
static phys_addr_t wipe_budget;
static phys_addr_t wipe_done;
static int wipe_poweroff_pending;

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
	if (!strcmp(str, "info")) {
		wipe_mode = WIPE_MODE_INFO;
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

static bool __init wipe_take(phys_addr_t len)
{
	if (wipe_budget) {
		if (len > wipe_budget)
			return false;
		wipe_budget -= len;
	}
	wipe_done += len;
	return true;
}

static void __init wipe_low_page(unsigned long pfn)
{
	if (!wipe_take(PAGE_SIZE))
		return;
	memset(__va(PFN_PHYS(pfn)), 0, PAGE_SIZE);
}

#ifdef CONFIG_HIGHMEM
static void __init wipe_high_page(unsigned long pfn)
{
	void *v;

	if (!wipe_take(PAGE_SIZE))
		return;
	v = kmap_atomic_pfn(pfn);
	memset(v, 0, PAGE_SIZE);
	kunmap_atomic(v);
}
#endif

static void __init wipe_high_range(phys_addr_t start, phys_addr_t end)
{
#ifdef CONFIG_HIGHMEM
	unsigned long pfn, last;

	pfn = PFN_UP(start);
	last = PFN_DOWN(end);
	for (; pfn < last; pfn++) {
		void *v;

		if (!wipe_take(PAGE_SIZE))
			return;
		v = kmap_atomic_pfn(pfn);
		memset(v, 0, PAGE_SIZE);
		kunmap_atomic(v);
	}
#else
	(void)start;
	(void)end;
#endif
}

static void __init wipe_highmem(void)
{
#ifdef CONFIG_HIGHMEM
	struct memblock_region *r;

	for_each_memblock(memory, r) {
		phys_addr_t start = r->base;
		phys_addr_t end = r->base + r->size;
		struct memblock_region *res;
		phys_addr_t cur;

		if (end <= PFN_PHYS(max_low_pfn))
			continue;
		if (start < PFN_PHYS(max_low_pfn))
			start = PFN_PHYS(max_low_pfn);
		if (start >= end)
			continue;

		cur = start;
		for_each_memblock(reserved, res) {
			phys_addr_t rs = res->base;
			phys_addr_t re = res->base + res->size;

			if (re <= cur || rs >= end)
				continue;
			if (rs > cur)
				wipe_high_range(cur, min(rs, end));
			if (re > cur)
				cur = re;
			if (cur >= end)
				break;
		}
		if (cur < end)
			wipe_high_range(cur, end);
		if (wipe_budget && !wipe_budget)
			break;
	}
#endif
}

static void __init wipe_lowmem(void)
{
	bootmem_data_t *bdata = NODE_DATA(0)->bdata;
	unsigned long pfn, last;
	unsigned long idx;

	if (!bdata || !bdata->node_bootmem_map)
		return;

	pfn = bdata->node_min_pfn;
	last = bdata->node_low_pfn;
	idx = 0;

	while (pfn < last) {
		unsigned long run;

		if (wipe_budget && !wipe_budget)
			return;

		/* Skip allocated pages.  The bitmap is word-sized, so
		 * look for a zero bit rather than testing every page. */
		run = find_next_zero_bit(bdata->node_bootmem_map,
					 last - bdata->node_min_pfn, idx);
		if (run >= last - bdata->node_min_pfn)
			break;
		idx = run;
		pfn = bdata->node_min_pfn + idx;
		wipe_low_page(pfn);
		idx++;
	}
}

static void __init wipe_all(void)
{
	if (wipe_test_mb)
		wipe_budget = (phys_addr_t)wipe_test_mb << 20;

	wipe_lowmem();
	wipe_highmem();
	pr_info("wipe: %llu MB wiped\n",
		(unsigned long long)(wipe_done >> 20));
}

static int __init wipe_check_page(unsigned long pfn)
{
	const u8 *v;
	int i, bad = 0;

#ifdef CONFIG_HIGHMEM
	v = kmap_atomic_pfn(pfn);
#else
	v = __va(PFN_PHYS(pfn));
#endif
	for (i = 0; i < PAGE_SIZE; i++) {
		if (v[i]) {
			bad = 1;
			break;
		}
	}
#ifdef CONFIG_HIGHMEM
	kunmap_atomic((void *)v);
#endif
	return bad;
}

static int __init wipe_check_lowmem(void)
{
	bootmem_data_t *bdata = NODE_DATA(0)->bdata;
	unsigned long pfn, last;
	unsigned long idx;
	int bad = 0;

	if (!bdata || !bdata->node_bootmem_map)
		return 0;
	pfn = bdata->node_min_pfn;
	last = bdata->node_low_pfn;
	idx = 0;
	while (pfn < last) {
		unsigned long run = find_next_zero_bit(bdata->node_bootmem_map,
						       last - bdata->node_min_pfn,
						       idx);
		if (run >= last - bdata->node_min_pfn)
			break;
		idx = run;
		pfn = bdata->node_min_pfn + idx;
		bad += wipe_check_page(pfn);
		idx++;
	}
	return bad;
}

static void __init wipe_verify_all(void)
{
	int bad = wipe_check_lowmem();

	if (bad)
		pr_err("wipe: verification FAILED (%d sampled pages non-zero)\n",
		       bad);
	else
		pr_info("wipe: verification OK\n");
}

static void __init wipe_force_power_off(void)
{
	pr_crit("wipe: forcing power off\n");

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
#ifdef CONFIG_MEMORY_WIPE_ON_POWEROFF
	if (!wipe_mode && krux_marker_valid()) {
		wipe_from_marker = 1;
		wipe_mode = WIPE_MODE_ALL;
		pr_crit("wipe: poweroff marker found, wiping this boot\n");
	}
#endif
	if (!wipe_mode)
		return;

	/* IRQs are already disabled here.  Do not touch preempt_count:
	 * the scheduler is not initialised yet. */
	local_irq_disable();
	local_fiq_disable();

	pr_crit("wipe: start (mode %d, test %lu MB, verify %d)\n",
		wipe_mode, wipe_test_mb, wipe_verify);

	if (wipe_mode == WIPE_MODE_INFO) {
		bootmem_data_t *bdata = NODE_DATA(0)->bdata;
		unsigned long pfn, last, idx = 0, free = 0;

		pr_info("wipe: memblock memory=%llu MB reserved=%llu MB\n",
			(unsigned long long)(memblock.memory.total_size >> 20),
			(unsigned long long)(memblock.reserved.total_size >> 20));
		pr_info("wipe: max_low_pfn=%lu max_pfn=%lu\n",
			max_low_pfn, max_pfn);
		pr_info("wipe: mem_map phys=0x%llx size=%lu KB\n",
			(unsigned long long)__pa(mem_map),
			(unsigned long)(max_pfn * sizeof(struct page)) >> 10);
		if (bdata && bdata->node_bootmem_map) {
			pfn = bdata->node_min_pfn;
			last = bdata->node_low_pfn;
			while (pfn < last) {
				unsigned long run = find_next_zero_bit(
					bdata->node_bootmem_map,
					last - bdata->node_min_pfn, idx);
				if (run >= last - bdata->node_min_pfn)
					break;
				if (free < 8)
					pr_info("wipe: free pfn %lu phys 0x%llx\n",
						bdata->node_min_pfn + run,
						(unsigned long long)PFN_PHYS(
							bdata->node_min_pfn + run));
				free++;
				idx = run + 1;
			}
			pr_info("wipe: bootmem min=%lu low=%lu free=%lu pages\n",
				bdata->node_min_pfn, bdata->node_low_pfn,
				free);
			{
				unsigned long p, lim, s = 0;
				int in = 0;

				lim = min(bdata->node_low_pfn,
					  (unsigned long)(64 << 20) >> PAGE_SHIFT);
				for (p = bdata->node_min_pfn; p < lim; p++) {
					int used = test_bit(
						p - bdata->node_min_pfn,
						bdata->node_bootmem_map);
					if (used && !in) {
						s = p;
						in = 1;
					} else if (!used && in) {
						pr_info("wipe: used 0x%llx..0x%llx\n",
							(unsigned long long)PFN_PHYS(s),
							(unsigned long long)PFN_PHYS(p));
						in = 0;
					}
				}
				if (in)
					pr_info("wipe: used 0x%llx..0x%llx\n",
						(unsigned long long)PFN_PHYS(s),
						(unsigned long long)PFN_PHYS(lim));
			}
		}
		return;
	}

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
		return;
	}

	/* Power off from kernel_init(), after the PMIC/restart drivers
	 * have initialised, so the normal poweroff path is used. */
	wipe_poweroff_pending = 1;
}

void wipe_poweroff(void)
{
	if (!wipe_poweroff_pending)
		return;

#ifdef CONFIG_MEMORY_WIPE_ON_POWEROFF
	if (wipe_from_marker)
		krux_marker_write(0);
#endif
	pr_crit("wipe: powering off\n");
	kernel_power_off();

	/* kernel_power_off() should not return. */
	wipe_force_power_off();
	while (1)
		barrier();
}
