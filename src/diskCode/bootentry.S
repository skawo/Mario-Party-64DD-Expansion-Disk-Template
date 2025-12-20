.equ SR_CU1, 0x20000000
.equ SR_FR,  0x04000000
.equ SR_KX,  0x00000080
.equ SR_SX,  0x00000040
.equ SR_UX,  0x00000020
.equ SR_PE,  0x00100000

.extern Disk_Main

.set noreorder
.section .text
.globl boot_entry
.ent boot_entry

boot_entry:
    la  $gp, _gp

    li   $v0, SR_CU1|SR_PE|SR_FR|SR_KX|SR_SX|SR_UX
    mtc0 $v0, $12

    # 8MB RAM
    li  $t0, 0x00800000
    sw  $t0, %gprel(__boot_memsize)($gp)

    # NTSC
    li  $t0, 0
    sw  $t0, %gprel(__boot_tvtype)($gp)

    # Cold boot
    li  $t0, 1
    sw  $t0, %gprel(__boot_resettype)($gp)

    # Retail N64 (not iQue)
    li  $t0, 0
    sw  $t0, %gprel(__boot_consoletype)($gp)

    la   $t0, debug_assert_func
    la   $t1, __assert_func_ptr
    sw   $t0, 0($t1)


	li $a0, 0
	jal Disk_Main					/* call main app */
	li $a1, 0

    jr   $ra
    nop

.end boot_entry
