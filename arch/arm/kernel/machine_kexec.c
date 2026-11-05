/*
 * machine_kexec.c - handle transition of Linux booting another kernel
 */

#include <linux/mm.h>
#include <linux/kexec.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/mmu_context.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/system_misc.h>
#include <asm/highmem.h>

extern const unsigned char relocate_new_kernel[];
extern const unsigned int relocate_new_kernel_size;

extern unsigned long kexec_start_address;
extern unsigned long kexec_indirection_page;
extern unsigned long kexec_mach_type;
extern unsigned long kexec_boot_atags;

static atomic_t waiting_for_crash_ipi;

/*
 * Provide a dummy crash_notes definition while crash dump arrives to arm.
 * This prevents breakage of crash_notes attribute in kernel/ksysfs.c.
 */

int machine_kexec_prepare(struct kimage *image)
{
	return 0;
}

void machine_kexec_cleanup(struct kimage *image)
{
}

void machine_crash_nonpanic_core(void *unused)
{
	struct pt_regs regs;

	crash_setup_regs(&regs, NULL);
	printk(KERN_DEBUG "CPU %u will stop doing anything useful since another CPU has crashed\n",
	       smp_processor_id());
	crash_save_cpu(&regs, smp_processor_id());
	flush_cache_all();

	atomic_dec(&waiting_for_crash_ipi);
	while (1)
		cpu_relax();
}

static void machine_kexec_mask_interrupts(void)
{
	unsigned int i;
	struct irq_desc *desc;

	for_each_irq_desc(i, desc) {
		struct irq_chip *chip;

		chip = irq_desc_get_chip(desc);
		if (!chip)
			continue;

		if (chip->irq_eoi && irqd_irq_inprogress(&desc->irq_data))
			chip->irq_eoi(&desc->irq_data);

		if (chip->irq_mask)
			chip->irq_mask(&desc->irq_data);

		if (chip->irq_disable && !irqd_irq_disabled(&desc->irq_data))
			chip->irq_disable(&desc->irq_data);
	}
}

void machine_crash_shutdown(struct pt_regs *regs)
{
	unsigned long msecs;

	local_irq_disable();

	atomic_set(&waiting_for_crash_ipi, num_online_cpus() - 1);
	smp_call_function(machine_crash_nonpanic_core, NULL, false);
	msecs = 1000; /* Wait at most a second for the other cpus to stop */
	while ((atomic_read(&waiting_for_crash_ipi) > 0) && msecs) {
		mdelay(1);
		msecs--;
	}
	if (atomic_read(&waiting_for_crash_ipi) > 0)
		printk(KERN_WARNING "Non-crashing CPUs did not react to IPI\n");

	crash_save_cpu(regs, smp_processor_id());
	machine_kexec_mask_interrupts();

	printk(KERN_INFO "Loading crashdump kernel...\n");
}

/*
 * Function pointer to optional machine-specific reinitialization
 */
void (*kexec_reinit)(void);

/*
 * Find the DTB segment loaded by kexec-tools.  The generic ARM entry
 * assumes an ATAGS block 0x7000 bytes below the zImage, which is too
 * small for a modern DTB (hammerhead's is ~176 KB).  kexec-tools puts
 * the DTB in its own segment, so pass that address in r2 instead.
 */
static unsigned long kexec_dtb_address(struct kimage *image)
{
	int i;
	unsigned long low = __pa(high_memory);

	for (i = 0; i < image->nr_segments; i++) {
		unsigned long mem = image->segment[i].mem;
		unsigned long size = image->segment[i].memsz;
		u32 magic;

		if (size < 8)
			continue;
		if (mem + 4 <= low) {
			magic = *(u32 *)__va(mem);
		} else {
			unsigned long pfn = PFN_DOWN(mem);
			unsigned long off = mem & (PAGE_SIZE - 1);
			void *v = kmap_atomic_pfn(pfn);

			magic = *(u32 *)((char *)v + off);
			kunmap_atomic(v);
		}
		if (magic == 0xd00dfeed)
			return mem;
	}
	return 0;
}

void machine_kexec(struct kimage *image)
{
	unsigned long page_list;
	unsigned long reboot_code_buffer_phys;
	void *reboot_code_buffer;

	arch_kexec();

	page_list = image->head & PAGE_MASK;

	/* we need both effective and real address here */
	reboot_code_buffer_phys =
	    page_to_pfn(image->control_code_page) << PAGE_SHIFT;
	reboot_code_buffer = page_address(image->control_code_page);

	/* copy our kernel relocation code to the control code page */
	memcpy(reboot_code_buffer,
	       relocate_new_kernel, relocate_new_kernel_size);

	/*
	 * Prepare the parameters in the *copied* block, not in the
	 * original symbols: with CONFIG_STRICT_MEMORY_RWX the kernel
	 * text is read-only, and writing kexec_start_address there
	 * faults.  relocate_new_kernel_size covers the parameter words
	 * at the end of the copied code.
	 */
	{
		unsigned long *p;
		unsigned long dtb = kexec_dtb_address(image);

#define KEXEC_PARAM(name, value)					\
	do {								\
		p = (void *)((char *)reboot_code_buffer +		\
			((char *)&name - (char *)relocate_new_kernel));	\
		*p = (value);						\
	} while (0)

		KEXEC_PARAM(kexec_start_address, image->start);
		KEXEC_PARAM(kexec_indirection_page, page_list);
		KEXEC_PARAM(kexec_mach_type, machine_arch_type);
		KEXEC_PARAM(kexec_boot_atags,
			    dtb ? dtb : (image->start - KEXEC_ARM_ZIMAGE_OFFSET +
					 KEXEC_ARM_ATAGS_OFFSET));
#undef KEXEC_PARAM
	}

	flush_icache_range((unsigned long) reboot_code_buffer,
			   (unsigned long) reboot_code_buffer + KEXEC_CONTROL_PAGE_SIZE);
	printk(KERN_INFO "Bye!\n");

	if (kexec_reinit)
		kexec_reinit();

	soft_restart(reboot_code_buffer_phys);
}

void arch_crash_save_vmcoreinfo(void)
{
#ifdef CONFIG_ARM_LPAE
	VMCOREINFO_CONFIG(ARM_LPAE);
#endif
}
