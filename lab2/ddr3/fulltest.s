#Register r0 will indicate a failure, register r1 will indicate the failure type.
#0: Branch failure, 1: Data instruction failure, 2: Memory instruction failure
#3: Barrel Shifter failure, 4: Conditional Execution failure

#Place 0xF into Register 0 - this will serve as an error indicator that we were unable to report errors correctly
mov r0, #0xF
#Do the same for r1, to indicate a failure which did not properly report.
mov r1, #0xF



#We know from manual testing that mov works, so it will not be tested here.
#Having at least one known-good component makes testing much easier.

#We'll use branching for most of this testing, so lets start with a simple
#branching test
b skipped
mov r1, #0x0
#swi isn't actually implemented in the hardware, but it should cause a bunch of undefined?
#swi #10 
skipped:

#We will also use conditional execution with branching,
#so lets test conditional execution with a simple compare
#and bne
cmp r1, #0x0
bne pass1
b conditionalfail
pass1:
#We know it passed when it should, but does it not pass when
#it shouldn't?
cmp r1, #0xF
bne trap
b trapdisarm
trap:
b conditionalfail
trapdisarm:
#bne works accurately with cmp if we got here.
#Now we know our basic test instructions work, so begin using them
#to quickly test the rest.



#Data Instruction Tests
mov r2, #0xF
mov r3, #0
mov r4, #0x8
and r5, r2, r3
cmp r5, #0
bne datafail
and r5, r2, r4
cmp r5, #0x8
bne datafail
#AND works
eor r5, r2, r3
cmp r5, #0xF
bne datafail
eor r5, r2, r4
cmp r5, #0x7
bne datafail
#EOR works
SUB r5, r2, r3
cmp r5, r2
bne datafail
SUBS r5, r2, r2
bne datafail
#SUB/SUBS works
ADD r5, r2, r2
cmp r5, #0x1E
bne datafail
ADDS r5, r3, r3
bne datafail
#ADD/ADDS works
adds r5, r2, r2
adc r5, r2, r2
cmp r5, #0x1E
bne datafail
mov r4, #1
ror r4, #1
adds r5, r4, r4
adc r5, r2, r2
cmp r5, #0x1F
bne datafail
#ADC works
tst r2, r3
bne datafail
mov r4, #0x8
tst r2, r4
beq datafail
#tst works
teq r4, r4
bne datafail
teq r2, r3
beq datafail
#teq works
cmp r2, r2
bne datafail
cmp r2, r4
beq datafail
#cmp works
mov r4, #1
ror r4, #1
asr r4, #31
mov r3, #1
cmn r3, r4
bne datafail
cmn r3, r2
beq datafail
mov r6, r4
mov r4, #0x8
mov r3, #0
#cmn, asr w/ msb 1, and ror work
orr r5, r2, r4
cmp r5, #0xF
bne datafail
orr r5, r4, r3
cmp r5, #8
bne datafail
#orr works
lsl r5, r3, #1
cmp r5, #0
bne datafail
lsl r5, r4, #30
cmp r5, #0
bne datafail
lsl r5, r2, #1
cmp r5, #0x1E
bne datafail
#lsl works
asr r5, r2, #1
cmp r5, #7
bne datafail
#asr w/ msb 0 works
bic r5, r2, r4
cmp r5, #7
bne datafail
bic r5, r2, r3
cmp r5, #0xF
bne datafail
#bic works
mvn r5, r6
cmp r5, #0
bne datafail
mvn r5, #0
add r5, r5, #1
cmp r5, #0
bne datafail
#mvn works



#Now we test the Memory instructions
mov r3, #0x1000
str r2, [r3, #4] 
str r3, [r3, #8] 
str r4, [r3, #12] 
str r6, [r3, #16] 

ldr r5, [r3, #4] 
cmp r5, r2
bne memfail
ldr r5, [r3, #8] 
cmp r5, r3
bne memfail
ldr r5, [r3, #12] 
cmp r5, r4
bne memfail
ldr r5, [r3, #16] 
cmp r5, r6
bne memfail
#ldr/str work

#In this process we tested some general conditional execution
#The rest of them are pretty much the same logic, so we'll assume
#They're probably good.

#Testing the barrel shifter was also accomplished by performing LSL/ROR/ASR/MOV
#which all just directly use the barrel shifter.
#So we don't end up needing separate testing for the shifter in the end.


#If we finish testing without branching to a failure, we're gtg.
b good



#Branches for result handling.

memfail:
mov r1, #2
b bad

conditionalfail:
mov r1, #4
b bad

datafail:
mov r1, #1
b bad

good:
#indicate no failure, then end
mov r0, #0
b end

bad:
#indicate failure, then end.
mov r0, #0x1
b end

end:
#swi #10
