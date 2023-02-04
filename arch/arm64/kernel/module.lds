SECTIONS {
	.plt 0 : { BYTE(0) }
	.init.plt 0 : { BYTE(0) }
	.text.ftrace_trampoline 0 : { BYTE(0) }

	/* Undo -fdata-sections and -ffunction-sections */
	.bss : { *(.bss .bss.[0-9a-zA-Z_]*) }
	.data : { *(.data .data.[0-9a-zA-Z_]*) }
	.rela.data : { *(.rela.data .rela.data.[0-9a-zA-Z_]*) }
	.rela.text : { *(.rela.text .rela.text.[0-9a-zA-Z_]*) }
	.rodata : { *(.rodata .rodata.[0-9a-zA-Z_]*) }
	.text : { *(.text .text.[0-9a-zA-Z_]*) }
}
