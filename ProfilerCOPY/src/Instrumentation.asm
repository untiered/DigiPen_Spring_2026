.data
; declare externs for our Profiler functions
EXTERNDEF EnterFunc:PROC
EXTERNDEF ExitFunc:PROC

.code

; Setup _penter
_penter PROC EXPORT
	
	; push the volatile registers onto the stack
	push RAX
	push RCX
	push RDX
	push R8
	push R9
	push R10
	push R11
	push RBX ; push additional 8 bytes for 16 byte allignment

	; move the return address 
	mov RCX, [RSP + 40h]

	; allocate 8 bytes of padding, 6 * 16 bytes of memory for the XMM registers, and 32 bytes of shadow space
	sub RSP, 88h

	; move the XMM register contents onto the stack
	movdqu [RSP + 20h], XMM0
	movdqu [RSP + 30h], XMM1
	movdqu [RSP + 40h], XMM2
	movdqu [RSP + 50h], XMM3
	movdqu [RSP + 60h], XMM4
	movdqu [RSP + 70h], XMM5

	; call the C++ function for saving data
	call OFFSET EnterFunc

	; restore the XMM registers
	movdqu XMM0, [RSP + 20h]
	movdqu XMM1, [RSP + 30h]
	movdqu XMM2, [RSP + 40h]
	movdqu XMM3, [RSP + 50h]
	movdqu XMM4, [RSP + 60h]
	movdqu XMM5, [RSP + 70h]

	; move RSP back to unmake the shadow space
	add RSP, 88h

	; clean up the stack
	pop RBX
	pop R11
	pop R10
	pop R9
	pop R8
	pop RDX
	pop RCX
	pop RAX

	; return
	ret

_penter ENDP

; Setup _pexit
_pexit PROC EXPORT

	; push the volatile registers onto the stack
	push RAX
	push RCX
	push RDX
	push R8
	push R9
	push R10
	push R11
	push RBX ; push additional 8 bytes for 16 byte allignment

	; move the return address 
	mov RCX, [RSP + 40h]

	; allocate 8 bytes of padding, 6 * 16 bytes of memory for the XMM registers, and 32 bytes of shadow space
	sub RSP, 88h

	; move the XMM register contents onto the stack
	movdqu [RSP + 20h], XMM0
	movdqu [RSP + 30h], XMM1
	movdqu [RSP + 40h], XMM2
	movdqu [RSP + 50h], XMM3
	movdqu [RSP + 60h], XMM4
	movdqu [RSP + 70h], XMM5

	; call the C++ function for saving data
	call OFFSET ExitFunc

	; restore the XMM registers
	movdqu XMM0, [RSP + 20h]
	movdqu XMM1, [RSP + 30h]
	movdqu XMM2, [RSP + 40h]
	movdqu XMM3, [RSP + 50h]
	movdqu XMM4, [RSP + 60h]
	movdqu XMM5, [RSP + 70h]

	; move RSP back to unmake the shadow space
	add RSP, 88h

	; clean up the stack
	pop RBX
	pop R11
	pop R10
	pop R9
	pop R8
	pop RDX
	pop RCX
	pop RAX

	; return
	ret

_pexit ENDP

END