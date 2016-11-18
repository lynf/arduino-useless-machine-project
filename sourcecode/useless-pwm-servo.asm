;
; useless-pwm-servo.asm
;
; Created: 10/18/2016 9:31:15 PM
; Author : lynf
;
;
;######################################################################################
; This software is Copyright by Francis Lyn and is issued under the following license:
;
; Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License
;
;######################################################################################
;
; Change Log:
; ===========
; 1. Revise <initz:> to enable pull-ups on all unused ports as precaution
;	to reduce unnecessary device current consupmtion.
; 
;
;
;
; BRIEF PROJECT DESCRIPTION
; =========================
; Uses R/C servo to operate a SPDT switch. User moves switch lever to closed position.
; Servo moves an arm with a wooden finger shape to push the switch back to the open
; position. A useless but nonetheless interesting and amusing project that demonstrates
; the versatility of the Atmel microcontroller for providing intelligence for a wide
; range of automatic control applications.
;
; The number of parts for the project are is small and readily available. You need an
; Atmel ATmega328P microcontroller, preferably any of the smaller Pro-mini or Nano boards,
; any hobby R/C servo, a SPDT toggle switch with a long enough lever, and some hardware for
; mounting the project and for making the servo moving arm/finger.
;
; Hardware TCNT1 module is used for generating the PWM servo control signal. TCNT1 is
; configured as a phase and frequency generator, running continuously at a rate of
; 41.667 Hz (24 ms period). The PWM servo control pulse width is set by loading output
; compare register OCR1A with a variable whose value determines the pulse's on time.
;
; The TCNT1 runs with a prescaler setting of 8, resulting in a counter clock rate of
; 16/2 = 8 MHz.
; 
; Because this project uses electromechanical interfaces, the controller must be adjusted
; to match the actual physical dimensions of the interface. This means you need to tweak
; the servo position settings so that the servo arm will move the correct distances
; to flick the switch open, and to fully retract to the parked position.
;
; You have to adjust the values of PWMmin and PWM equates until the servo arm winds up
; moving the required stroking distance. The code contains "tweak" routines to help you
; tune the servo control values. You invoke the tweak routine by grounding the 

; debug from false to true. Change the values in look-up table servotabl:, assemble
; and run the program to try out various values while observing the servo position.
; When you find values that give the desired servo positions, you can plug these values
; into the PWMmin and PWMmax equates, change debug to false, then assemble and run
; the final program.
;
; The program contains many general purpose routines some of which are not needed
; by the final program. Routines for serial communications and video control routines 
; are handy for debugging but can be removed when the program debugging is
; completed and things run the way you want.
;
; The servo has three wires:
; Brown - Gnd
; Red - +5 V
; Orange - PWM servo input signal
;
; Spdt switch input on PC0, A0 output on Arduino board.
; Tune servo pin on PC1, A1 input on Arduino board. Ground pin to invoke tuning.
; Servo PWM output control signal on PB1, OC1A output
;
; ATmega328P cpu, 16.0000 MHz external crystal
;
; LFUSE: F7
; HFUSE: DF
;
;

.list						; Listing on

.equ	 F_CPU = 16000000

;
;
; General equates
;
.equ	FALSE = 0x0			; Logical 0
.equ	TRUE = !FALSE		; Logical 1
.equ	Vcc_low = false		; False for 5 V controller board
;.equ	Vcc_low = true		; True for 3.3 V controller board
;.equ	debug = true		; Turn on debugging routines
.equ	debug = false		; Turn on debugging routines
;.equ	baud_low = true		; 9600 baud for later boards
.equ	baud_low = false	; 19200 baud for 1st board
;
;
; UART definitions
;
.if		baud_low
.equ	BAUD = 9600			; Baud rate
.if		Vcc_low
.equ	BAUD_PRE = 51		; Baud rate prescaler - 8.00 MHz clock
.else
.equ	BAUD_PRE = 103		; Baud rate prescaler - 16.00 MHz clock
.endif
;
.else
;
.equ	BAUD = 19200		; Baud rate
.if		Vcc_low
.equ	BAUD_PRE = 25		; Baud rate prescaler - 8.00 MHz clock
.else
.equ	BAUD_PRE = 51		; Baud rate prescaler - 16.00 MHz clock
.endif
;
.endif
;
.equ	NULL = 0x0			; Null terminator
.equ	BELL = 0x07			; Bell
.equ	BS = 0x08			; Backspace
.equ	HT = 0x09			; Tab
.equ	LF = 0x0a			; Linefeed
.equ	CR = 0x0d			; Carriage return
.equ	ctlW = 0x17			; Control W
.equ	ctlX = 0x18			; Control X
.equ	ctlZ = 0x1a			; Control Z
.equ	SP = 0x20			; Space
.equ	ESC = 0x1b			; Escape
.equ	DEL = 0x7f			; Delete
.equ	CMA	= 0x2c			; Comma
;
.equ	ctlA = 0x01			; Control A, SOH
.equ	ctlS = 0x13			; Control S, DC3
;


.if		Vcc_low

; Timer0, Timer1 and Timer2 parameters for 8.00 MHz clock
;
; Prescaler:	1		8		64			256			1024
; TCNTn clk:	8 MHz	1 MHz	125 kHz		31.25 kHz	7.8125 kHz
; Period:				1 us	8 us		32 us		128 us
;
; TCNT0 prescaler = 1024, clk_T0 = 8 MHz/1024 = 7.8125 kHz, 128 us
;
.equ	OCR0Aload = 125		; OCR0A 8 bit register, 125 x 128 us = 16 ms
;

.else

;
; Timer0 parameters for 16.00 MHz clock
;
; Prescaler:	1		8		64			256			1024
; TCNTn clk:	16 MHz	2 MHz	250 kHz		62.5 kHz	15.625 kHz
; Period:				0.5 us	4 us		16 us		64 us
;
; TCNT0 prescaler = 1024, clk_T0 = 16 MHz/1024 = 15.625 kHz, 64 us
;
;
.equ	OCR0Aload = 250		; OCR0A 8 bit register, 250 x 64 us = 16 ms

.endif
;
;
; Servo controller
;
.equ	PWMfreq = 24000		; ICR1HL value for 41.6667 Hz (24 ms)
.equ	PWMmin = 800		; open position
.equ	PWMmax = 1440		; close position
.equ	PWMmid = 1200		; Mid-position
;

;
;
; Flag register flaga
;
.equ	tmrf = 0			; Timer tick flag
.equ	numfl = 3			; Valid byte number flag
.equ	crf = 4				; Carriage return key flag
.equ	escf = 5			; Escape key flag
.equ	kyf	= 6				; Control key flag
.equ	xclinf = 7			; Delayed line clear flag
;
; Flag register flagb
;
; Software timer reload values, count cycles for 4.096 ms rate
;
.equ	t25ms = 6			; 25 ms delay
.equ	t100ms = 24			; 100 ms delay
.equ	t500ms = 122		; 500 ms delay

;
.equ	sblen = 0x1f		; Serial input buffer length
;
.equ	linsz = 0x10		; Line buffer size
.equ	ndec = 5			; Digits to display/convert
;
; Register definitions
;
.def	count = R2			; Counter for line buffer
.def	asav = R3			; rga save register
.def	SRsav = R4			; SREG save
.def	rtmr = R5			; Temporary timer
;
.def	res0 = R6			; result register 0
.def	res1 = R7			; result register 1
.def	res2 = R8			; result register 2
;
;
; High registers
;
.def	rmp = R16			; Multipurpose register
.def	rga = R17			; GP register RGA
.def	rgb = R18			; GP register RGB
.def	rgc = R19			; GP register RGC
.def	rgd = R20			; GP register RGD
.def	rge	= R21			; GP register RGE
.def	rgv	= R22			; Variable register
.def	flaga = R23			; Flag A register, 8 flags
.def	flagb = R24			; Flag B register, 8 flags
;
; --- Macro definitions ---
;
.macro	ldzptr				; Load ZH:ZL pointer with address*2
		ldi		ZH,high(@0*2)
		ldi		ZL,low(@0*2)
.endm
;
.macro	ldxptr				; Load XH:XL pointer with address to access data memory
		ldi		XH,high(@0)
		ldi		XL,low(@0)
.endm
;
.macro	ldyptr				; Load YH:YL pointer with address to access data memory
		ldi		YH,high(@0)
		ldi		YL,low(@0)
.endm
;
; Exchange contents of registers
;
.macro	xchreg				; Exchange registers
		push	@0
		push	@1
		pop		@0
		pop		@1
.endm
;
;
;
; ============================================
;       S R A M   D E F I N I T I O N S
; ============================================
;
.DSEG
.ORG	0x0100
;
;
linbuf:
.byte	linsz		; Character input line buffer
;
; Data buffer for word and byte
;
wdbuf:
.byte	2			; Word byte buffer
dba:
.byte	1			; Data byte buffer (don't move position relative to wdbuf)
;
; UART serial input buffer
;
uart_st:
sinb:						; Serial input buffer
.byte		sblen
;
getcnt:
.byte		1				; Input buffer getbyte counter
;
putcnt:
.byte		1				; Input buffer putbyte counter
;
uart_end:
;
;
; ============================================
;   R E S E T   A N D   I N T   V E C T O R S
; ============================================
;
;
.CSEG
.ORG	$0000
		rjmp		Main		; Int vector 1 - Reset vector

.ORG	OC0Aaddr
		jmp			Timer0_COMPA	; Timer 0 Output Compare A handler
;
;.ORG	OC1Aaddr
;		jmp			Timer1_COMPA	; Timer 1 Output Compare A handler
;
;
.ORG	URXCaddr
		jmp			URXCint		;  USART Rx Complete
;
;
; End of interrupt vectors, start of program code space
;
;
.ORG	0x0034					; Program begins here
;
;
;
;#################################################################################################
;
verm:	.db	"*** Version 0.0 - 2016 Oct 21 ***",ctlZ

license: .db	"Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License"

;
;#################################################################################################
;
;
;
;
;
; ============================================
;     I N T E R R U P T   S E R V I C E S
; ============================================
;
;
; --- Timer 0 interrupt handler ---
;
; Used for I/O scanning
; 
; TCNT0 run in Output Compare mode, using OCR0A register to
; generate output compare interrupt every 250 x 64 us = 16 ms.
;
; TCNT0 operates in Clear Timer on Compare Match (WGM02:0 = 2).
; On Compare Match, TCNT0 counter is cleared.
; OCR0A (set to 64) defines the counter's TOP value.
;
; Clk_T0 = 16 MHz/1024 = 15.625 kHz, 64 us period
;
Timer0_COMPA:
;
		push	rmp					; Save registers
		in		SRsav,SREG
;
; Software timer
;
		sbr		flaga,(1<<tmrf)		; Set flag every cycle
;
		out		SREG,SRsav			; Restore SREG
		pop		rmp
		reti
;
;
;
; USART Receive complete interrupt handler. Interrupt invoked by RXC0 set
; when UDRE buffer has received character ready. Routine puts the received
; character into the SINB serial input buffer and increments the PUTCNT counter.
; When SBLEN+1 counts reached, PUTCNT is rolled back to 0 counts. SINB acts
; as a circular buffer.
;
URXCint:
		in		SRsav,SREG		; Save SREG
		push	rga
		push	rmp
		push	XH
		push	XL
;
		ldxptr	sinb			; Sinb base address
		lds		rmp,putcnt		; Get current putcnt offset
		clr		rga
		add		XL,rmp			; Add putcnt offset to sinb pointer
		adc		XH,rga			; Update pointer (16 bit addition)
		lds		rga,UDR0		; rga <-- UDR0
		st		X,rga			; Store to SINB buffer
		inc		rmp				; Increment putcnt
		cpi		rmp,sblen		; Past end of buffer?
		brne	URXCint1
		clr		rmp				;	Yes, reset putcnt
URXCint1:
		sts		putcnt,rmp		; Update putcnt to next free location
;
		pop		XL
		pop		XH
		pop		rmp
		pop		rga
		out		SREG,SRsav			; Restore SREG
		reti

;
;
;
;###########################################################################
;
;
; ============================================
;         Initialization Routines
; ============================================
;
;
; Turn off watchdog
;
wdt_off:
		cli							; Clear global interrupts
;
; Reset WD timer
;
		wdr
;
		in		rmp,MCUSR				; Clear WDRF bit
		andi	rmp,(0xff & (0<<WDRF))	; WDRF bit = 0
		out		MCUSR,rmp
;
; Set WDCE and WDE bits, keep old prescaler setting
;
		lds		rmp,WDTCSR
		ori		rmp,(1<<WDCE)|(1<<WDE)
		sts		WDTCSR,rmp
;
; Turn off WDT
;
		ldi		rmp,(0<<WDE)			; Clear WD system reset enable
		sts		WDTCSR,rmp
;
		sei								; Set global interrupts
		ret
;
;
; Initialize the controller hardware
;
initz:	
;
		rcall	zregs			; Clear lower registers R0,..,R15
		clr		flaga			; Clear flag registers
		clr		flagb
;
; Activate pull-up resistors on all input pins, used and unused
;
		ldi		rmp,0b00011101
		out		PORTB,rmp
;
		ldi		rmp,0b00111111
		out		PORTC,rmp
;
		ldi		rmp,0b11111100
		out		PORTD,rmp
;
; Define output pins		
;
		sbi		DDRB,PB1		; OC1A, output comparator 1A PWM output
		sbi		DDRB,PB5		; LED
;
;
; --- Timers Initialization ----
;
;
; === TCNT0 Initialization === 
;
; Setup TCNT0 prescaler = 1024, clock period = 64 us
;
InitTimer0:
		ldi		rmp,(1<<CS02)|(1<<CS00)	; Divide by 1024 prescaler, Fclk = 15.625 kHz
		out		TCCR0B,rmp				; Timer/Counter0 control register B
;
; Setup TCNT0 for CTC mode
;
		ldi		rmp,(1<<WGM01)			; CTC mode
		out		TCCR0A,rmp				; Timer/Counter0 control register A
;
; Initialize OCR0A output compare register
;
		ldi		rmp,OCR0Aload			; Set OCR0A = 64 for 4.096 ms period
		out		OCR0A,rmp
;
; Enable Timer/Counter0 Compare A Match Interrput in TIMSK0
;
		lds		rmp,TIMSK0
		sbr		rmp,(1<<OCIE0A)			; Enable Timer/Counter0 Output Compare A Match Interrupt
		sts		TIMSK0,rmp
;
;
;
; === TCNT1 Initialization ===
;
; Setup 16 bit Timer/Counter1 for phase and frequency correct PWM mode. The Servo
; PWM output is on OC1A (PB3) pin. The counting starts from BOTTOM to TOP, and then
; from TOP to BOTTOM. The output pin OC1A switches low on the upcount when count
; equals OCR1A value. OC1A switches high on the downcount direction when count equals
; OCR1A again.
;
; The TOP value is defined by the value in ICR1HL register. The PWM frequency is
; fclk_I/O / 2.N.TOP, where N is the prescaler value.
;
; To change the output pulse width duration, change the value loaded into the OCR1HL
; register.
;
; Set up ICR1HL register to define PWM frequency of 41.666 Hz, 24 ms period
;
; Minimum = 300, 0%, 1.0 ms pulse width
; Maximum = 1600, 50%, 1.5 ms pulse width
;
; 
		ldi		rmp,high(PWMfreq)		; Load 16 bit ICR1
		ldi		rga,low(PWMfreq)
		sts		ICR1H,rmp				; 16 bit write - write to ICR1H first
		sts		ICR1L,rga
;
		rcall	srv_open				; Servo to retracted position
;
		ldi		rmp,(1<<COM1A1)				; Clear OC1A on compare match, upcounting
		sts		TCCR1A,rmp					; Write to control register A
		ldi		rmp,(1<<WGM13)|(1<<CS11)	; Phase and frequency correct, TOP=ICR1, prescaler = 8
		sts		TCCR1B,rmp
;
;
; Enable global interrupt and exit
;
		sei							; Enable global interrupt 
		ret
;
;
; Initialize the UART for 19200 baud asynchronous operation
;
inzuart:
		cli							; Clear global interrupts
		ldi		rmp,high(BAUD_PRE)
		sts		UBRR0H,rmp			; Load baud rate register high
		ldi		rmp,low(BAUD_PRE)
		sts		UBRR0L,rmp			; Load baud rate register low
;
; Setup frame for 1 start, 8 data, 1 stop and no parity
;
		ldi		rmp,(1<<UCSZ00)|(1<<UCSZ01)
		sts		UCSR0C,rmp
;
; Enable the UART RX, TX, and RXC interrupt
;
		ldi		rmp,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0) ; |(1<<TXCIE0)|(1<<UDRIE0)
		sts		UCSR0B,rmp			; Enable RX, TX and RXC interrupt
;
		sei							; Set global interrupts
		ret
;
; Zero lower registers R0...R15
;
zregs:
		ldi		rga,16
		clr		rmp
		ldxptr	0x0			; Register file base address
zregs1:
		st		X+,rmp
		dec		rga
		brne	zregs1
		ret
;
;
;###########################################################################
;
;
;
; ================================
;     M A I N    P R O G R A M  
; ================================
;
Main:
;
; Initialize the stack pointer to end of SRAM
;
		ldi		rmp, HIGH(RAMEND) ; Init MSB stack
		out		SPH,rmp
		ldi		rmp, LOW(RAMEND)  ; Init LSB stack
		out		SPL,rmp
;
; Initialze the rest of the hardware
;
		rcall	wdt_off			; Disable watchdog. Must be done soon after a reset
		rcall	initz			; Initialize the controller
;
		sbic	PINC,PC1		; Tune input grounded?
		rjmp	start			;	No, Start normal operation
;
; ===================
; Servo motor tuning
; ===================
;
; The "tweak:" routine is activated by grounding the tune PC1 (A1) pin.
; The routine allows you to input various values to the TCNT1's OCR1A register to
; change the PWM output signal and hence the servo motor position. Values are
; entered via a terminal interface program (PuTTY serial terminal)
; connected to the controller USB port on a host PC.
;
; On startup a nominla OCR1A default value of 1200, or about 50% servo position
; is used. Newly entered values should be within the range of 600 to 2400.
;
; Try various settings until the servo arm is positioned to match the actual
; mechanical locations of the servo motor, actuating rod and spdt switch.
;
; After you obtain the proper values to stroke the arm to the required end
; positions, edit the code to change the equates PWMmin and PWM max, re-assemble
; the program and flash the executable back to the controller's flash memory.
; Remove the ground from input pin A1 to resort back to the normal operating program.
;
; Initialize the UART
;
tweak:
		rcall	inzuart			; Initialize the UART
;
; Display sign-on banner
;
		rcall	clrscn			; Clear screen
		ldzptr	scrnm			; Signon message
		rcall	pptr			; Print screen
;
		ldxptr	wdbuf			; Pointer to wdbuf
		ldi		rmp,high(PWMmid)
		st		X+,rmp			; Store high byte, midrange position
		ldi		rmp,low(PWMmid)
		st		X,rmp			; Store low byte, midrange position
		rcall	indx			; 'Index: ' prompt
		rcall	pdbuf			; Show contents of wordbuf
		rcall	setpos			; Load servo setting
;
cml1:
		rcall	enter			; Enter prompt
		rcall	getpos			; Load servo position with wordb contents
;
cml2:
		rcall	glin			; User line input
		sbrc	flaga,escf		; Escape sequence?
		rjmp	cml2			;	Yes, skip cursor commands if ESC
;
;
; Arrive here if input was not a valid escape sequence. Process
; possible number entry, leaving result in dba or wdbuf buffers
;
getpos:
		rcall	gchr			; Process line input
		tst		rga				; Line empty?
		breq	cml2			; Get new enter line
		rcall	decdg			; Test for 0....9
		brcc	cml2			; No decimal digit, redo entry
		rcall	gnum			; Get possible number, set numfl if good
;
		sbrs	flaga,numfl		; numfl set?
	;	rjmp	cml2			;	No, input error, redo entry
		rjmp	cml1			;	No, input error, redo entry
		cbr		flaga,(1<<numfl) ;	Yes, clear numfl
;
		rcall	indx			; 'OC1AHL: ' value
		rcall	pdbuf			; Show contents of wordbuf
		rcall	setpos			; Load servo setting from wdbuf
		rjmp	cml1
;
; Load wdbuf contents to servo motor PWM generator
; 
setpos:
		lds		rmp,wdbuf			; Load 16 bit OCR1A for open position
		lds		rga,wdbuf+1
		sts		OCR1AH,rmp			; 16 bit write - write to OCR1AH first
		sts		OCR1AL,rga
		ret
;
;
; Display 'Index:' prompt
;
indx:
		rcall	pxy
		.db		4,2
		rcall	ceol
		ldzptr	indxm
		rcall	pptr
		ret
;
indxm:
		.db		"OC1AHL: ",ctlZ
;
; Display 'Enter:' prompt
;
enter:
		call	pxy
		.db		5,2
		call	ceol
		ldzptr	entm
		call	pptr
		ret
;
entm:
		.db	" Enter: ",ctlZ
;
;
;
;
; ==========================
;  Normal program operation
; ==========================
;
start:
		rcall	srv_open			; Retract finger
		ldi		rgv,32				; Wait 1/2 s (16 ms x 32 = 512)
start1:
		sbrs	flaga,tmrf
		rjmp	start1
		cbr		flaga,(1<<tmrf)
		dec		rgv
		brne	start1

;
; Check and wait until switch is closed
;
start3:
		sbic	PINC,PC0			; Test if input PC0 low (switch closed)
		rjmp	start3				;	No, still open
;
; Wait for switch to settle in closed position - switch debounce
;
		ldi		rgv,63				; Wait 1 s
start4:
		sbrs	flaga,tmrf
		rjmp	start4
		cbr		flaga,(1<<tmrf)
		dec		rgv
		brne	start4
;
; Extend finger to open the switch
;
		rcall	srv_close			; Extend finger
;
;
; Check and wait until switch is opened
;
start5:
		sbis	PINC,PC0			; Test if input PC0 high (switch open)
		rjmp	start5				;	No, still closed
;
; Finally re-opens
;
		ldi		rgv,8				; Wait 0.128 s
start2:
		sbrs	flaga,tmrf
		rjmp	start2
		cbr		flaga,(1<<tmrf)
		dec		rgv
		brne	start2
		rjmp	start

;
; Extend (close) and Retract (open) servo finger 
;
srv_open:
		ldi		rmp,high(PWMmin)	; Load 16 bit OCR1A for open position
		ldi		rga,low(PWMmin)
		sts		OCR1AH,rmp			; 16 bit write - write to OCR1AH first
		sts		OCR1AL,rga
		ret
;
srv_close:
		ldi		rmp,high(PWMmax)	; Load 16 bit OCR1A for close position
		ldi		rga,low(PWMmax)
		sts		OCR1AH,rmp			; 16 bit write - write to OCR1AH first
		sts		OCR1AL,rga
		ret
;
;
;
;###########################################################################
;
; General purpose timer using TCNT0 interrupt routine rate 4.096 ms
;
; Entry:	rmp = timer count value
; Exit:		tmrf set
;
; User to test for tmrf flag to determine end of timing cycle.
;
start_timer:
		mov		rtmr,rmp
		cbr		flaga,(1<<tmrf)		; Clear timer flag
		ret
;
;
;###########################################################################
;
;
; --- Line input and initialization routines ---	(OK)
;
;  A 20 byte line input buffer is supported. The buffer is initially
;  cleared to zeroes, and pointed to by XH:XL. COUNT maintains a
;  count of characters entered. Entry is terminated by <'CR'>, <^X> 
;  erases current line and starts over, and <BS> or <DEL> erases
;  previous character. XH:XL is reserved for use as LINBUF pointer
;  to allow multiple GCHR calls.
;
;	Registers used:
;	rmp, rga, rgb, rgc, X
;
glin:
		rcall	inzln			; Zero the line buffer and count register
glin1:
		rcall	ci				; Get a character
		cpi		rga,CR			; Test if <CR>
		brne	glin2			;	No, look for next special key
		ldxptr	linbuf			;	Yes, reset linbuf pointer
		sbr		flaga,(1<<crf)	; And set CR flag
		ret
;
; Look for a ^X key, if so do a line delete
;
glin2:
		cpi		rga,ctlX		; Test if <^X>
		brne	glin3			;	No, look for next special key
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin
glin2a:
		call	bksp			; Move cursor back one space
		dec		rgb
		brne	glin2a			; back to start
		rjmp	glin			; Restart
;
; Look for a BS key, if so do a delete character at cursor
;
glin3:
		cpi		rga,BS			; Test if backspace
		brne	glin3b			;	No, look for next special key
glin3a:
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin1			;	Yes, fetch another character
		dec		rgb
		mov		count,rgb
		call	bksp			; Move cursor back one space
		ldi		rmp,0			; Backup pointer and insert 0 
		st		-X,rmp
		rjmp	glin1
;
; Look for a DEL key, if so do a backspace
;
glin3b:
		cpi		rga,DEL			; Test if DEL
		brne	glin5			;	No, look for next special key
		rjmp	glin3a
;
; Look for a Tab key, if so expand tab to spaces
;
glin5:
		cpi		rga,HT			; Test if tab
		brne	glin6			;	No,  look for next special key
		ldi		rgc,7			; Temp counter
		ldi		rga,SP			; Space character
glin5a:
		rcall	ldlin
		dec		rgc
		brne	glin5a
		rjmp	glin1
;
; Look for a Escape key, if so set escf
;
glin6:
		cpi		rga,ESC			; Test if esc
		brne	glin7			;	No, look for other control key
		sbr		flaga,(1<<escf)	; Set esc flag
		ret
;
; Look for other control key. Check for ^S key and do return if
; found.
;
glin7:
		cpi		rga,ctlS		; ^S?
		brne	glin7a			;	No, continue
		ret						;	Yes, exit
glin7a:
		rcall	fctl			; Test for other control key
		sbrs	flaga,kyf
		rjmp	glin8			;	kyf = 0
		rjmp	glin1			; Ignore other control keys
;
; Arrive here is valid key entry
;
glin8:
		rcall	ldlin			; Load the input buffer and show
		rjmp	glin1
;
; Load character in rga to LINBUF, update pointer and character counter		(OK)
;
ldlin:
		mov		rgb,count		; Get current count
		cpi		rgb,linsz		; End of buffer?
		brne	ldlin1			;	No
		ret						;	Yes, exit
ldlin1:
		inc		rgb
		mov		count,rgb		; Update count
		st		X+,rga			; Store entered key to buffer
		rcall	co				; Show it
		ret
;
;  Get linbuf character, increment XH:XL pointer and set C if
;  not 'CR', else clear C, rga = 0. 
;
gchr:
		ld		rga,X+			; Get character from line buffer, advance pointer
		cpi		rga,0			; Test for 0
		brne	gchr1			;	rga >= 0, means ascii printable character
		clc
		ret
gchr1:
		sec
		ret
;
; Clear input line buffer	(OK)
;
inzln:
		clr		rmp				; Fill byte
		clr		count			; Initialize count to 0
		ldi		rgb,linsz		; Buffer size
		ldxptr	linbuf			; Point to line buffer
inzln1:
		st		X+,rmp
		dec		rgb
		brne	inzln1
		ldxptr	linbuf			; Point to line buffer
		cbr		flaga,(1<<crf)|(1<<escf)	; Clear exit flags
		ret
;
;  Test rga for control key, 0...19H, 7FH..FFH, and set KYF		(OK)
;  if true, else clear KYF. rga preserved
;
fctl:
		sbr		flaga,(1<<kyf)
		cpi		rga,SP				; rga < SP?
		brcs	fctl1				;	Yes
		cpi		rga,DEL				; rga >= SP?
		brcc	fctl1				;	No
		cbr		flaga,(1<<kyf)		; Clear kyf
fctl1:
		ret
;
;  Show contents of Data Buffer		(OK)
;
pdbuf:
		ldxptr	wdbuf			; Pointer to wdbuf
		ld		YH,X+			; Get hi byte
		ld		YL,X			; Get lo byte
		call	bn2bcd			; Convert to packed BCD
		call	p5dg			; Show the data word
		ret
;
; Print 5 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p5dg:
		mov		rga,res2		; Fetch 1st of 3 bytes
		rcall	pdg				; Show digit 5 only
;
; Print 4 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p4dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pacc			; Show digits 4,3
;
; Print 2 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p2dg:
		mov		rga,res0		; Fetch 3rd of 3 bytes
		call	pacc
		ret
;
; Print 3 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p3dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pdg				; Show digit 3 only
		rjmp	p2dg			; Show digits 2,1
;
; Convert packed BCD digits in rga to ascii and display		(OK)
;
pacc:
		mov		asav,rga		; Save the data
		swap	rga				; Hi nibble first
		call	pdg				; Convert to ascii and display
		mov		rga,asav
		call	pdg				; Show lo nibble
		ret
;
; Display bcd low nibble as ascii digit			(OK)
;
pdg:
		andi	rga,0x0f		; Mask hi nibble
		ldi		rmp,'0'			; Convert by adding '0'
		add		rga,rmp
		call	co				; Show the digit
		ret
;
;
;
;###########################################################################
;
; --- Conversion and testing routines ---
;
;  Convert rga to upper case
;
case:
		cpi		rga,0x61		; Ascii 'a'
		brcs	case1			; < a
		cpi		rga,0x7b		; Ascii 'z'
		brcc	case1			; > z
		clc
		sbci	rga,SP			; Subtract SP to convert to UC
case1:
		ret
;
; gnum used to process possible good ascii number from linbuf, convert it to		(OK)
; binary number in YH:YL using gdbn.
;
; Save data byte in DBA if number less than 256, or data word
; in WDBUF if number is 256 to 65,535. The numfl is set if the input
; data is good byte value. User to clear numfl after test.
;
; Registers:	flaga, rgv, YH, YL, rmp, XH:XL
;
gnum:
		cbr		flaga,(1<<numfl)	; Clear the good number flag
		mov		rgv,count			; Load counter with number of characters in linbuf
		cpi		rgv,ndec+1			; Compare to limit
		brcc	gnum2				; Too many digits error
		call	gdbn				; Convert to binary in YH:YL
		brcc	gnum2				; Error
		ldxptr	wdbuf				; Point to data word buffer
		st		X+,YH				; Save result high byte
		st		X+,YL				; Save result low byte
		st		X,YL				; Save result low to data byte buffer
		tst		YH					; Test result high byte if zero
		breq	gnum1				;	Yes, word value
		ldi		rmp,0xff			;	No, cap byte value to 0xff
		st		X,rmp				;   Save in dba
gnum1:
		sbr		flaga,(1<<numfl)	; Mark as byte value
gnum2:
		ret
;
; Convert ASCII decimal string in LINBUF to binary number in  YH:YL.		(OK)
; The number to be converted is maximum allowed 5 ascii digits long, 
; set by ndec, equivalent to (0xffff) binary number.
;
; This routine called by gnum to convert ascii numbers for data entry
;
; Entry: rgv = ndec, number of ascii digits to convert
; Exit:  YH:YL <-- 16 bit result if ok, C = 1
;        YH:YL <-- 00, C = 0 if error
; Regs:  rga, rgb, rgc, YH, YL, rgv, XH:XL
;
gdbn:
		clr		YH			; Clear result registers
		clr		YL
		ldxptr	linbuf		; Setup line buffer pointer
gdbn1:
		ld		rga,X+		; Fetch a character
		call	decdg		; Convert to BCD
		brcc	gdbnx		; Error exit
		mov		asav,rga	; Save character
		call	dex10		; Value * 10, result in YH:YL
		brcs	gdbnov		; Overflow
;
		mov		rgc,asav	; Add original digit in
		clr		rgb
		call	adebc		; YH:YL = YH:YL + rgb:rgc
		brcs	gdbnov		; Overflow error
		dec		rgv			; All characters processed?
		brne	gdbn1		;	No, continue
		sec
		ret					;	Yes, normal exit
;
gdbnx:
		clc					; Error exit
		clr		YH
		clr		YL
		ret
;
gdbnov:
		sec					; Overflow condition
		ldi		YH,0xff
		ldi		YL,0xff	; Limit to 0xFFFF
		ret
;
; --- Data Buffer control and Math routines ---
;
; Multiply YH:YL by 10, called by gdbn ascii to binary converter routine		(OK)
; YH:YL = YH:YL * 10, C = 0 if ok, C = 1 on error
; Registers:	rga, rgb, rgc, YH, YL
;
dex10:
		call	dex2		; YH:YL * 2
		brcs	dexx		; Error exit, overflow and C=1
		push	YH			; Copy YH:YL to rgb:rgc
		pop		rgb
		push	YL
		pop		rgc
;
		rcall	dex2		; * 4
		brcs	dexx
		rcall	dex2		; * 8
		brcs	dexx
		call	adebc		; YH:YL = YH:YL + rgb:rgc
dexx:
		ret
;
; YH:YL: = YH:YL * 2
; 
dex2:
		clc
		rol		YL
		rol		YH
		ret
;
; YH:YL = YH:YL + rgb:rgc, C = 0 if ok else C = 1 on overflow
;
adebc:
		add		YL,rgc
		adc		YH,rgb
		ret
;
; Convert ASCII 0.....9 to BCD, C = 1 if ok, else		(OK)
; C = 0 and rga unchanged
; Registers:	rga, asav
;
decdg:
		mov		asav,rga	; Save ascii digit
		call	case		; Fold to UC
		subi	rga,'0'		; Char less than char '0'?
		brcs	ddgx		;	Yes, error exit
		cpi		rga,LF		;  Char from 0...9?
		brcc	ddgx		;	No, error exit
		ret					; Is 0...9
ddgx:
		clc					; Not 0...9
		mov		rga,asav
		ret
;
; Convert 16 bit binary in YH:YL to packed bcd in res2:res1:res0		(OK)
;
; Registers: rgb, rgc, YH, YL, res0, res1, res2
;
bn2bcd:
		ser		rgc					; rgc = 0xff
		mov		res2,rgc
;
; Process 10,000's digit
;
cvde_L10k:
		inc		res2
		subi	YL,low(10000)
		sbci	YH,high(10000)
		brcc	cvde_L10k			; Loop until C set
		subi	YL,low(-10000)		; Correct last subtraction
		sbci	YH,high(-10000)
		ldi		rgb,(256-16)
;
; Process 1000's digit
;
cvde_L1k:
		subi	rgb,(-16)
		subi	YL,low(1000)
		sbci	YH,high(1000)
		brcc	cvde_L1k			; Loop until C set
		subi	YL,low(-1000)		; Correct last subtraction
		sbci	YH,high(-1000)
		mov		res1,rgc
;
; Process 100's digit
;
cvde_L100:
		inc		res1
		subi	YL,low(100)
		sbci	YH,high(100)
		brcc	cvde_L100			; Loop until C set
		subi	YL,low(-100)		; Correct last subtraction
		or		res1,rgb
		ldi		rgb,(256-16)
;
; Process 10's digit
;
cvde_L10:
		subi	rgb,(-16)
		subi	YL,10
		brcc	cvde_L10			; Loop until C set
		subi	YL,-10				; Correct last subtraction
		mov		res0,rgb
		or		res0,YL
		ret
;
;
; Convert rga to hex digit and set C, else clear C if character
; not an ascii hexadecimal digit. On error, return with character in rga
;
; Registers:	rga
;
hexdg:
		mov		asav,rga	; Save char
		rcall	case		; Fold to UC
		subi	rga,'0'		; rga < '0'?
		brcs	hexdg1		;	Yes, exit
		cpi		rga,LF		; rga from 0...9?
		brcs	hexdg2		;	Yes
		subi	rga,7		; Dump funny chars
		cpi		rga,LF		; Char from 9...A?
		brcs	hexdg1
		cpi		rga,0x10	; Char above F?
		brcc	hexdg1		;	Yes
hexdg2:
		ret					; Normal exit, C=1
hexdg1:
		mov		rga,asav	; Restore char
		clc
		ret
;
;
;
; ==============================
;	U A R T Serial I/O Module
; ==============================
;
;
; --- Position cursor at row, column immediately following call to pxy ---
;
; Row & column values must be given as ascii decimal.			(OK)
;
pxy:
	rcall	vpxy1			; Lead-in sequence
	pop		ZH				; Point to string start address
	pop		ZL
	clc
	rol		ZL				; 16 bit multiply by 2 for word address
	rol		ZH
;
	lpm		rga,Z+			; Pick up row value
	rcall	pdec			; Print it and ..
	rcall	vpxy2			; Middle sequence			+++++ Uses Z pointer, must save Z +++
	lpm		rga,Z+			; Pick up column value
	rcall	pdec			; Print it and ..
	rcall	vpxy3			; End sequence
;
	clc
	ror		ZH
	ror		ZL
	push	ZL				; Return to caller
	push	ZH
	ret
;
;  Position cursor at (RGD)-->row, (RGE)-->col		(OK)
;
gotoxy:
	rcall	vpxy1			; Send lead-in string
	mov		rga,rgd			; Get row value
	rcall	pdec			; Send row
	rcall	vpxy2			; Send middle string
	mov		rga,rge			; Get col value
	rcall	pdec			; Send col
	rcall	vpxy3			; Send trailing string
	ret
;
;
; Show in-line string message, zero terminated. Memory organized as 16 bit words		(OK)
; 
pmsg:
	pop		ZH				; Point to string start address
	pop		ZL
	clc
	rol		ZL				; 16 bit multiply by 2 for word address
	rol		ZH
pmsg1:
	lpm		rga,Z+			; String byte to rga, Z+
	cpi		rga,ctlZ		; ^Z byte?
	brne	pmsg2			; Print if not zero
;
; Check for and skip extra null byte in case string did not end on word boundary
;
	lpm		rga,Z
	cpi		rga,NULL
	brne	pmsg10
	adiw	ZH:ZL,1			; Go past extra null byte
pmsg10:
	clc
	ror		ZH
	ror		ZL
	push	ZL				; Return to pmsg caller
	push	ZH
	ret
pmsg2:
	rcall	co
	rjmp	pmsg1
;
;
;***************************************************************************
;
; "div8u" - 8/8 Bit Unsigned Division				(OK)
;
; This subroutine divides the two register variables "rga" (dividend) and 
; "rgb" (divisor). The result is placed in "rga" and the remainder in "rgc".
;  
; High registers used:	4 (rga,rgb,rgc,rgv)
;
;                                  
; Register Variables:
;	rgc	remainder
;	rga	dividend & result
;	rgb divisor
;	rgv	loop counter
;
; Entry:	(rga) = dividend
;			(rgb) = divisor
; Exit:		(rga) = integer part of quotient
;			(rgb) = integer remainder 
;                                    
div8u:	
		push	rgc
		push	rgv
		sub		rgc,rgc			; clear remainder and carry
        ldi		rgv,9			; init loop counter
d8u_1:	rol		rga				; shift left dividend
        dec		rgv				; decrement counter
        brne	d8u_2			; if done
		mov		rgb,rgc			; move remainder to rgb
		pop		rgv
		pop		rgc
        ret						;    return
;
d8u_2:	rol		rgc				; shift dividend into remainder
        sub		rgc,rgb			; remainder = remainder - divisor
        brcc	d8u_3			; if result negative
        add		rgc,rgb			;    restore remainder
        clc						;    clear carry to be shifted into result
        rjmp	d8u_1			; else
d8u_3:	sec						;    set carry to be shifted into result
        rjmp	d8u_1
;
;
; --- Low level video drivers ---
;
; Register rga used to pass data to write to console output routine
;
; Print rga data as two hexadecimal digits.			(OK)
;
pahex:
		push	rga
		swap	rga				; Show MSD nibble first
		rcall	pahex1
		pop		rga
pahex1:
		andi	rga, 0x0f		; Mask off higher nibble
		ldi		rgv, 0x30 		; Add ascii '0' to convert
		add		rga, rgv		; Convert to ascii
		cpi		rga, 0x3a		; Check if > 9
		brcs	pahex2			;  No, it is 0 ... 9
		ldi		rgv, 0x07		;  Yes, convert to A ... F
		add		rga, rgv
pahex2:
		rcall	co
		ret
;
;  Print rga contents as decimal (0...255). Leading			(OK)
;  zero suppression is provided only on the 100's
;  digit, so at least two digits are always printed.
;
; Registers rga, rgb not saved
;
pdec:
		ldi		rgb,100			; Get 100's digit
		rcall	div8u
		tst		rga				; Do leading zero suppression
		breq	pdec1
		rcall	pnum
pdec1:
		ldi		rga,10			; Get 10's digit
		xchreg	rga,rgb
		rcall	div8u			; rgb has units
		rcall	pnum
		xchreg	rga,rgb
pnum:
		ori		rga,0x30		; Ascii "0"
		rcall	co				; Show ascii decimal
		ret
;
;###########################################################################
;
;
; Scan for console character and return with character if any,
; else return with rga = 0. Data is available in sinb when putcnt
; is greater than getcnt.
;
; Registers:
;	rmp, rga, rgb, XHL (preserved across routine)
;
; Exit:	rga <-- character, if any, else 0
;
getc:
		lds		rmp,getcnt
		lds		rga,putcnt
		cp		rga,rmp			; Compare getcnt to putcnt
		breq	getc2			; Same, no new data
getc0:
		push	XH
		push	XL				; Save X registers
		ldxptr	sinb			; sinb base address
		lds		rmp,getcnt		; Get current getcnt offset
		clr		rga
		add		XL,rmp			; Add getcnt offset to sinb pointer
		adc		XH,rga			; Update pointer (16 bit addition)
;		
		ld		rga,X			; rga <-- @XHL
		pop		XL
		pop		XH				; Restore X registers
		inc		rmp				; Increment getcnt
		cpi		rmp,sblen		; Past end of buffer?
		brne	getc1
		clr		rmp				;	Yes, reset getcnt
getc1:
		sts		getcnt,rmp		; Update getcnt to next buffer location
		ret
getc2:
		clr		rga
		ret
;
; Wait for a received data byte, return received data in rga.
;
ci:	
		lds		rmp,getcnt
		lds		rga,putcnt
		cp		rga,rmp			; Compare getcnt to putcnt
		breq	ci				; No incoming data
		rjmp	getc0			; Get new data
;
; Load UDR0 from rga. Wait until transmitter is empty before loading.		(OK)
;
co:	
		lds		rmp,UCSR0A		; Get UART control status register
		sbrs	rmp,UDRE0		; Test if UDR0 is empty
		rjmp	co
;
; Send data
;
		sts		UDR0,rga		; UDR0 <-- rga
		ret
;
;
;###########################################################################
;
; Print CR and LFs	(OK)
;
crllf:
		rcall	crlf			; Two CRLF
crlf:
		push	rga
		ldi		rga,CR			; Carriage return
		rcall	co
		ldi		rga,LF			; Linefeed
		rcall	co
		rjmp	cco
;
; Print spaces	(OK)
;
dblsp:
		rcall	space
space:
		push	rga
		ldi		rga,SP			; Space
		rjmp	cco
;
; Print tab	(OK)
;
prtab:
		push	rga
		ldi		rga,HT
		rjmp	cco
;
; Ring bell
;
beep:
		push	rga
		ldi		rga,BELL
cco:
		rcall	co
		pop		rga
		ret
;
; Print comma	(OK)
;
prcma:
		push	rga
		ldi		rga,cma
		rjmp	cco
;
; Print backspace/delete pair	(OK)
;
bksp:
		push	rga
		ldi		rga,BS			; Backspace
		rcall	co
		rcall	vdcc			; Delete character at cursor
		rjmp	cco
;
; Print message string, zero terminated. Routine is called with		(OK)
; code address of string loaded in ZH:ZL.
;
pptr:
		push	rga
pptr1:
		lpm		rga,Z+			; String byte to rga, Z+
		cpi		rga,ctlZ		; byte ^Z?
		brne	pptr2			; Print if not ^Z
		pop		rga
		ret
pptr2:
		cpi		rga,NULL		; Skip any nulls in string
		breq	pptr1
		rcall	co
		rjmp	pptr1
;
; --- Video and Cursor control routines ---
;
; Clear screen	(OK)
;
clrscn:
		push	zh
		push	zl
		ldzptr	scrn		; Home cursor
		rcall	pptr
		ldzptr	clrs		; Clear entire screen
		rjmp	video
;
; --- Clear to end of line ---
;
ceol:
		push	zh
		push	zl
		ldzptr	eol			; Clear to end of screen
		rjmp	video
;
; --- Delete character at ccursor ---
;
vdcc:
		push	zh
		push	zl
		ldzptr	dcc			; Delete character at cursor
		rjmp	video
;
; --- Highlight on ---
;
vhi:
		push	zh
		push	zl
		ldzptr	hi			; Highlight on
		rjmp	video
;
; --- Normal ---
;
vlo:
		push	zh
		push	zl
		ldzptr	lo			; Normal - attributes off
		rjmp	video
;
; --- Reverse ---	(OK)
;
vrev:
		push	zh
		push	zl
		ldzptr	rev			; Reverse on
video:
		rcall	pptr
		pop		zl
		pop		zh
		ret
;
; --- Video position cursor sequences ---
; Lead-in sequence
;
vpxy1:
		push	zh
		push	zl
		ldzptr	pxy1			; Lead-in sequence
		rjmp	video
;
; Middle sequence
;
vpxy2:
		push	zh
			push	zl
	ldzptr	pxy2			; Middle sequence
		rjmp	video
;
; End sequence
;
vpxy3:
		push	zh
		push	zl
		ldzptr	pxy3			; Trailing sequence
		rjmp	video
;
; --- Save cursor position ---
;
vscp:
		push	zh
		push	zl
		ldzptr	scp			; Save cursor position
		rjmp	video
;
; --- Restore cursor position ---
;
vrcp:
		push	zh
		push	zl
		ldzptr	rcp					; Restore cursor position
		rjmp	video

;
; --- Message strings data area ---
;
; Terminal control sequences
;
scrn:	.db	ESC,"[H",ctlZ		; Home cursor and
eos:	.db	ESC,"[0J",ctlZ		; Clear to end of screen
clrs:	.db	ESC,"[2J",ctlZ		; Clear entire screen
eol:	.db	ESC,"[0K",ctlZ		; Erase to end of line
hi:		.db	ESC,"[1m",ctlZ		; Highlight on
lo:		.db	ESC,"[m",ctlZ		; Normal - attributes off
rev:	.db	ESC,"[7m",ctlZ		; Reverse on
pxy1:	.db	ESC,"[",ctlZ		; Lead-in sequence
pxy2:	.db	";",ctlZ			; Middle sequence
pxy3:	.db	"H",ctlZ			; Trailing sequence
dcc:	.db	ESC,"[1P",ctlZ		; Delete character at cursor
dlc:	.db	ESC,"[1M",ctlZ		; Delete line at cursor
scp:	.db	ESC,"7",ctlZ		; Save cursor position
rcp:	.db	ESC,"8",ctlZ		; Restore cursor position
;
;
;
scrnm:
		.db		"===<<<The Useless Project >>>===",cr,lf
		.db		" Tweak the servo position ... ",ctlZ
;
;
;##########################################################################
;
;
; End of source code
;
.EXIT
;
;##########################################################################
;

