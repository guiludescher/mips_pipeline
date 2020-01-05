# test2.asm
# 23 March 2006 S. Harris sharris@hmc.edu
# Test MIPS instructions.
# Assembly Code

main: ori $t0, $0, 0x8000    # 34088000   # $t0 recebe 1000 0000 0000 0000
addi $t1, $0, -32768         # 20098000   # $t1 recebe -32768
ori $t2, $t0, 0x8001         # 350a8001   # $t2 recebe 1000 0000 0000 0001
beq $t0, $t1, there          # 11090005   # NÃO realiza branch
slt $t3, $t1, $t0            # 0128582a   # $t3 recebe 1
bne $t3, $0, here            # 15600001   # realiza branch (vai para 'here') 1=>
j there                      # 08000009   # 2=> pula (vai pra 'there') 3=>
here: sub $t2, $t2, $t0      # 01485022   # 1=> $t2 recebe 1
ori $t0, $t0, 0xFF           # 350800ff   # t0 recebe com 1000 0000 1111 1111‬ 2=>
there: add $t3, $t3, $t2     # 016a5820   # 3=> $t3 recebe 2
sub $t0, $t2, $t0            # 01484022   # $t0 recebe -33022‬
sw $t0, 82($t3)              # ad680052   # guarda -33022 em 84