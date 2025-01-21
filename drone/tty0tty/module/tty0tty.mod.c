#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

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



static const char ____versions[]
__used __section("__versions") =
	"\x1c\x00\x00\x00\xa9\x08\x6c\x9a"
	"__tty_alloc_driver\0\0"
	"\x18\x00\x00\x00\xc1\x7e\xb2\x67"
	"tty_std_termios\0"
	"\x18\x00\x00\x00\xc9\x32\xc6\xf9"
	"tty_port_init\0\0\0"
	"\x20\x00\x00\x00\xef\xfc\xac\xbb"
	"tty_port_link_device\0\0\0\0"
	"\x1c\x00\x00\x00\x9d\x79\x08\xa4"
	"tty_register_driver\0"
	"\x10\x00\x00\x00\x7e\x3a\x2c\x12"
	"_printk\0"
	"\x1c\x00\x00\x00\xc0\x6d\xb0\xbd"
	"tty_driver_kref_put\0"
	"\x1c\x00\x00\x00\x91\x47\x3f\x67"
	"tty_port_destroy\0\0\0\0"
	"\x20\x00\x00\x00\xc3\x76\xec\x43"
	"tty_unregister_device\0\0\0"
	"\x20\x00\x00\x00\x49\x02\x30\x5c"
	"tty_unregister_driver\0\0\0"
	"\x10\x00\x00\x00\xba\x0c\x7a\x03"
	"kfree\0\0\0"
	"\x14\x00\x00\x00\x65\x93\x3f\xb4"
	"ktime_get\0\0\0"
	"\x28\x00\x00\x00\x7b\xf9\xe9\x1f"
	"__tty_insert_flip_string_flags\0\0"
	"\x20\x00\x00\x00\x7b\xd4\xcb\xd7"
	"tty_flip_buffer_push\0\0\0\0"
	"\x1c\x00\x00\x00\x20\x5d\x05\xc3"
	"usleep_range_state\0\0"
	"\x1c\x00\x00\x00\xcb\xf6\xfd\xf0"
	"__stack_chk_fail\0\0\0\0"
	"\x18\x00\x00\x00\xe1\xbe\x10\x6b"
	"_copy_to_user\0\0\0"
	"\x14\x00\x00\x00\xcf\xe2\xb6\x02"
	"pcpu_hot\0\0\0\0"
	"\x20\x00\x00\x00\xd6\xc7\xd8\xaa"
	"default_wake_function\0\0\0"
	"\x18\x00\x00\x00\x38\x22\xfb\x4a"
	"add_wait_queue\0\0"
	"\x14\x00\x00\x00\x51\x0e\x00\x01"
	"schedule\0\0\0\0"
	"\x1c\x00\x00\x00\x88\x00\x11\x37"
	"remove_wait_queue\0\0\0"
	"\x18\x00\x00\x00\xe0\x0c\x6f\xea"
	"param_ops_short\0"
	"\x14\x00\x00\x00\xbb\x6d\xfb\xbd"
	"__fentry__\0\0"
	"\x1c\x00\x00\x00\xca\x39\x82\x5b"
	"__x86_return_thunk\0\0"
	"\x20\x00\x00\x00\xd8\x94\xd3\x0b"
	"tty_termios_baud_rate\0\0\0"
	"\x10\x00\x00\x00\xca\xaf\x26\x66"
	"down\0\0\0\0"
	"\x0c\x00\x00\x00\x66\x69\x2a\xcf"
	"up\0\0"
	"\x1c\x00\x00\x00\x63\xa5\x03\x4c"
	"random_kmalloc_seed\0"
	"\x18\x00\x00\x00\x10\x03\x98\x24"
	"kmalloc_caches\0\0"
	"\x18\x00\x00\x00\xeb\x9d\x19\x1d"
	"kmalloc_trace\0\0\0"
	"\x14\x00\x00\x00\x45\x3a\x23\xeb"
	"__kmalloc\0\0\0"
	"\x10\x00\x00\x00\xc5\x8f\x57\xfb"
	"memset\0\0"
	"\x18\x00\x00\x00\xd7\xd3\x75\x6d"
	"module_layout\0\0\0"
	"\x00\x00\x00\x00\x00\x00\x00\x00";

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "B0109AAEA80098AA5685A4B");
