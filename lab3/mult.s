.text
	mov	r6, #0x100	@ base memory address
	mov	r0, #0		@ r0 = low-order word of result
	mov 	r1, #0		@ r1 = high-order word of result
	mov	r2, #0x69	@ r2<-low-order word of multiplicand
	str	r2, [r6, #0x10]	@ store multiplicand
	mov	r3, #0x0	@ r3<-high-order word of multiplicand
	mov	r4, #0x5a	@ load multiplier
	str	r4, [r6, #0x24]	@ store multiplicand
	mov	r5, #0		
loop:	tst	r4, #1		@ is y odd?
	addnes	r0, r0, r2	@ add and set flags if y is odd
	tst	r4, #1		@ previous add may have changed flags
	adcne	r1, r1, r3	@ add and use carry flag if y is odd
	lsls	r2, r2, #1	@ shift lsw of x left into carry bit
	lsl	r3, r3, #1	@ make room for the carry bit is msw
	adc	r3, r3, #0	@ add carry bit to msw of x
	lsrs	r4, r4, #1	@ shift y right
	bne	loop		@ if y==0, we are done
	str	r0, [r6, #8]	@ store result

