.data
    n: .word 11
.text
.globl __start


jal x0, __start
#----------------------------------------------------Do not modify above text----------------------------------------------------
JUMP:
add a1,x0,x15
jalr x0,0(x1)

FUNCTION3:
sll x15,x15,x20

add a1,x15,x23
add a1,a1,x13
#slli x24,x24,1
#slli x22,x22,1
#add a1,a1,x24
#add a1,a1,x22
jalr x0,0(x1)
#beq x8,x0,x1
FUNCTION1: #n<10«á¶]FUNCTION2
addi x20,x20,1
#addi x6,a0,0

#0.875n
addi x21,x18,0
addi x22,x21,0
slli x21,x21,1
add x22,x22,x21
slli x21,x21,1
add x22,x22,x21
srli x22,x22,3
addi x21,x20,-1
sll x22,x22,x21
add x23,x23,x22 #0.875n total

#137 part
addi x24,x0,-137
sll x24,x24,x21
add x13,x13,x24


slli x19,x18,1
add x19,x19,x18
srli x19,x19,2
addi x18,x19,0
bge x19,x16,FUNCTION1

FUNCTION2: #n<10
slli x15,x15,1
addi x14,x14,1
beq x14,a0,JUMP
beq x14,x19,FUNCTION3
jal x0, FUNCTION2

FUNCTION:
# Todo: Define your own function
# We store the input n in register a0, and you should store your result in register a1
addi x14,x0,0
addi x15,x0,7
addi x16,x0,10
addi x18,a0,0
addi x19,x0,0
addi x20,x0,0
addi x21,x0,0
addi x22,x0,0
addi x23,x0,0
addi x24,x0,0
addi x13,x0,0
beq a0,x0,JUMP
blt a0,x16,FUNCTION2
jal x17, FUNCTION1




#----------------------------------------------------Do not modify below text----------------------------------------------------
__start:
    la   t0, n
    lw   a0, 0(t0)
    jal  x1, FUNCTION
    la   t0, n
    sw   a1, 4(t0)
    li a7, 10
    ecall
