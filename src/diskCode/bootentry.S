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
    # install new inthandler

    la      $t0, inthandler
    srl     $t0, $t0, 2         
    lui     $t1, 0x03FF          
    ori     $t1, $t1, 0xFFFF
    and     $t0, $t0, $t1        
    lui     $t1, 0x0800          
    or      $t0, $t0, $t1        
    li      $t1, 0xA0000000
    sw      $t0, 0($t1)
    li      $t1, 0xA0000080
    sw      $t0, 0($t1)
    li      $t1, 0xA0000100
    sw      $t0, 0($t1)
    li      $t1, 0xA0000180
    sw      $t0, 0($t1)

    la  $gp, _gp

    li   $v0, SR_CU1|SR_PE|SR_FR|SR_KX|SR_SX|SR_UX
    mtc0 $v0, $12

    li      $t3, 0xA0000000

    lw      $t0, 0x0318($t3)
    sw      $t0, %gprel(__boot_memsize)($gp)

    #lbu    $t0, 0x0300($t3)
    
    li      $t0, 1 
    lbu     $t1, 0x030C($t3)
    lbu     $t2, 0x0388($t3)

    sw      $t0, %gprel(__boot_tvtype)($gp)
    sw      $t1, %gprel(__boot_resettype)($gp)
    sw      $t2, %gprel(__boot_consoletype)($gp)

    la   $t0, debug_assert_func
    la   $t1, __assert_func_ptr
    sw   $t0, 0($t1)

	jal __do_global_ctors		/* call global constructors */
	nop    

	li $a0, 0
	jal Disk_Main					/* call main app */
	li $a1, 0

    jr   $ra
    nop

.end boot_entry
