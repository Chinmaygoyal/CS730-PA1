#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x8e6402a9, "module_layout" },
	{ 0xec646bf1, "pci_unregister_driver" },
	{ 0x988d943b, "__pci_register_driver" },
	{ 0x9c5429ca, "dma_alloc_attrs" },
	{ 0xde80cd09, "ioremap" },
	{ 0xcf2d6503, "pci_enable_device" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x41136265, "device_destroy" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x64b60eb0, "class_destroy" },
	{ 0xe8129999, "device_create" },
	{ 0xa946dcde, "__class_create" },
	{ 0xcb720829, "__register_chrdev" },
	{ 0xffee12e9, "try_module_get" },
	{ 0xe78dfe6d, "kmem_cache_alloc_trace" },
	{ 0x595451f1, "kmalloc_caches" },
	{ 0x56470118, "__warn_printk" },
	{ 0x360b6b7a, "pv_ops" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xc959d152, "__stack_chk_fail" },
	{ 0x12a38747, "usleep_range" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0x4842a737, "dma_free_attrs" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x4a453f53, "iowrite32" },
	{ 0xa78af5f3, "ioread32" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xc5850110, "printk" },
	{ 0x7aa1756e, "kvfree" },
	{ 0x49b3b968, "module_put" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v00001234d0000DEBAsv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "865567EA9D184072777F56C");
