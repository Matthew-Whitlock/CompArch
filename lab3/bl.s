.text
main:	bl branch
	sub r1, r2, r2
	sub r2, r1, r1
	sub r3, r2, r2
branch:	sub r0, r15, r15
	