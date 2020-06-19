
#include	"extern.h"

FPPA_Duty		=>	_SYS(INC.FPPA_NUM);	// Single FPPA = 1, Mult FPPA = 2 or 4/8/...

//	T_xxx		=>	xxx nS
T_High			=>	4700;
T_Low			=>	4700;
T_Start			=>	4700;
T_Stop			=>	4700;
T_Buf			=>	4700;
Delay_High		=>	T_High ?  (((System_Clock / FPPA_Duty) / (1000000000 / T_High)) + 1) : 0;
Delay_Low		=>	T_Low ?   (((System_Clock / FPPA_Duty) / (1000000000 / T_Low)) + 1) : 0;
Delay_Start		=>	T_Start ? (((System_Clock / FPPA_Duty) / (1000000000 / T_Start)) + 1) : 0;
Delay_Stop		=>	T_Stop ?  (((System_Clock / FPPA_Duty) / (1000000000 / T_Stop)) + 1) : 0;
Delay_Buf		=>	T_Buf ?   (((System_Clock / FPPA_Duty) / (1000000000 / T_Buf)) + 1) : 0;

//#if	(Delay_High >= 256) || (Delay_Low >= 256)
//	.ERROR	I2C_Delay is %Delay_High, %Delay_Low, and >= 256
//	.ENDP
//#endif


// PA4=ADC in (via comparator)
// PA0=PWM out
#define	PWMpin	0	// PA.0=PWM
#define	ADCpin	4	// PA.4=ADC
#define Dout0	0	// PB.0=LEDS0
#define Dout1	1	// PB.1=LEDS1
#define Dout2	2	// PB.2=LEDS2
#define Dout3	3	// PB.3=LEDS3

#define Sw1		6	// PA.6 switch 1
#define Sw2		7	// PA.7	switch 2
#define	SDA_BIT	5	// PA.5 I2C data
#define	SCL_BIT	3	// PA.3 I2C clock
#define	Dev_Address 0x50
#define	WriteCmd	Dev_Address << 1
#define	ReadCmd		(Dev_Address << 1) | 1

#define Din0	4	// PB.4
#define Din1	5	// PB.5
#define Din2	6	// PB.6
#define Din3	7	// PB.7
I2C_SDA	BIT PA.SDA_BIT
I2C_SCL	BIT	PA.SCL_BIT
I2C_SDA_DDR	BIT PAC.SDA_BIT
I2C_SCL_DDR	BIT	PAC.SCL_BIT

byte	adr,running,eebyte,pwm_width;
word	millis;

pchar	macro	ch
		A=ch;
		Outch();
		endm



digitalRead	macro	result,port,rbit
	mov		a,0
	swapc	port.rbit
	slc	a
	mov		result,a
	endm

Easy_Delay	macro	val, cmp
	#IF	val > cmp
		.delay	val - cmp;
	#ENDIF
	endm
I2C_Dat_In	macro
	PAC.SDA_BIT=0;
	Easy_Delay (Delay_Low, 4);
	endm

I2C_Dat_Out	macro
	PAC.SDA_BIT=1;
	Easy_Delay (Delay_Low, 4);
	endm

I2C_Clk_Low	macro
	PA.SCL_BIT=0;
	Easy_Delay (Delay_Low, 4);
	endm

I2C_Clk_Hi	macro
	PA.SCL_BIT=1;
	Easy_Delay (Delay_Low, 4);
	endm

I2C_Dat_Low	macro
	PA.SDA_BIT=0;
	Easy_Delay (Delay_Low, 4);
	endm

I2C_Dat_Hi	macro
	PA.SDA_BIT=1;
	Easy_Delay (Delay_Low, 4);
	endm



void	delayXms(void);
void	EEread(void);
void	EEwrite(void);
void	delVals(void);
void	RunProg(void);
void	programming(void);
void	analogRead(void);
void	I2C_Read_Byte (void);
void	I2C_Write_Byte (void);
void	I2C_Start(void);
void	I2C_Stop(void);
void	tstprog(void);
void	Servotab();
void	PWMtab(void);
void	doServo();
void	doPWM(void);

// use comparator as 4 bit A/D
// by changing the reference voltage ...
// ... and checking the comparator result bit
void	analogRead(void){
byte	adcval=15;
	do{
		gpcs=adcval;
		nop
		nop
		if(gpcc.6) break;
		adcval--;
	}while(adcval);
	A=adcval;
}
// I2C stuff for EEPROM follows
void	I2C_Start(void){
	I2C_Dat_Hi
	I2C_Clk_Hi
	I2C_Dat_Low;
	Easy_Delay (Delay_Start,7)
}

void	I2C_Stop(void){
	I2C_Dat_Low
	I2C_Clk_Hi
	I2C_Dat_Hi
	Easy_Delay (Delay_Stop,1);
	I2C_Clk_Low
}
/*
enter assuming read command and address has been set
exit A=data byte read
*/

static void	I2C_Read_Byte (void){
byte	data=0;
byte	count=8;
	I2C_Dat_in				// set data line to input
	do{
		I2C_Clk_Hi			// set clock high
		data <<=1;
		if(I2C_SDA)	data |=1;
		I2C_Clk_Low
	}while(--count);
	I2C_Dat_Out				// set SDA to output for...
	I2C_Dat_Hi
	I2C_Clk_Hi				// ...ACK
	I2C_Clk_Low
	A=data;					// return data byte
}
/*
enter A=byte to write
assuming write command and address has been set
*/
static void	I2C_Write_Byte (void)
{
BYTE	data=A;						// save input data
byte	count	=	8;				// bit counter
	I2C_Clk_Low						// set clock low
	do{
		sl		data;				// test MSB and move next one in
		if(CF){						// was it a 1 ?
			I2C_Dat_Hi				// yes so set it high
		}else{
			I2C_Dat_Low				// else set it to 0
			 }
		I2C_Clk_Hi					// set clock hi to toggle it
		I2C_Clk_Low					
	} while (--count);				// doit 8 times ....
	I2C_Dat_Hi
	I2C_Dat_In
	I2C_Clk_Hi						// set clock hi get ACK
	I2C_Clk_Low						// clock=0
	I2C_Dat_Out
	I2C_Dat_Hi
//	if (I2C_SDA) trap;				//	Pleass add you code, when no Ack ..
	Easy_Delay (Delay_High, 2);
}									//	2
/*
 enter adr ->byte to read return in A
 */

void	EEread(){
byte rtmp;
	I2C_Start();
	A=WriteCmd;				// first set up address from adr
	I2C_Write_Byte();
	A=adr;
	I2C_Write_Byte();
	I2C_Stop();
	I2C_Start();			// now read that data
	A=ReadCmd;
	I2C_Write_Byte();
	I2C_Read_Byte();
	rtmp=A;					// save data read
	I2C_Stop();
	A=rtmp;					// restore data to return in A
}


/*
 enter A= byte to write at adress adr
 */
void	EEwrite(){
byte wtmp=A;  				// save entry paramter
	I2C_Start();
	A=WriteCmd;				// first set up address from adr
	I2C_Write_Byte();
	A=adr;
	I2C_Write_Byte();
	A=wtmp;					// now send data to write
	I2C_Write_Byte();
	I2C_Stop();
 }
//
// main evaluation/execution spin loop
//
void	RunProg() {
  byte  Areg = 0;			// the 'TPS CPU' internal registers
  byte  Breg = 0;
  byte  Creg = 0;
  byte  Dreg = 0;
  byte	adrhi=0;			// EEPROM 'page' address
  byte	com,dat,addret,rtmp,stmp;	// various tmep variable ...
  adr = 0;
  while (1) {
	A=adr;
    EEread();				// read one instruction
	eebyte = A;
	adr++;
    com = eebyte >> 4;		// split into command ..
    dat = eebyte & 0x0f;	// .. and data
    switch (com) {			// the just use the commands as a lookup
      case 1:				// 1: A=Data
        PB=dat;
        break;
      case 2:				// 2: Wait according to data 1,2,5,10 mS  etc
		A=dat;
        delayXmS();
        break; 
      case 3:				// 3: relative jump backwards data instrcutions
        adr--;				//  .... 0 endless loop
        adr -= dat;
        break;
      case 4:				// 4: immedite load of A with data
        Areg = dat;
        break;
      case 5:				// 5: various moves FROM A data says where to ..
        if (dat == 1) Breg = Areg;		// A->B
        if (dat == 2) Creg = Areg;		// A->C
        if (dat == 3) Dreg = Areg;		// A->D
        if (dat == 4) PB=Areg;			// A->Dout 

        if (dat == 5){					// 5-8 bits 0-3 of A->pins 0-3
			if(Areg.0)
				PB.Dout0=1;
			else 
				PB.Dout0=0;
			}
        if (dat == 6){
			if(Areg.0)
				PB.Dout1=1;
			else
				PB.Dout1=0;
			}
        if (dat == 7){
			if(Areg.0)
				PB.Dout2=1;
			else
				PB.Dout2=0;
			}
        if (dat == 8){
			if(Areg.0)
				PB.Dout3=1;
			else
				PB.Dout3=0;
			}
        if (dat == 9) {				// data is used as PWM value to pin PA0
			A=Areg;
			doPWM();
			}
		if (dat == 10){				// 10 data is use for servo
			A=Areg;
			doServo();				// do the servo stuff
			}
        break;
      case 6:						// 6x: data is put into A
        if (dat == 1) Areg = Breg;	// B->A
        if (dat == 2) Areg = Creg;	// C->A
        if (dat == 3) Areg = Dreg;	// D->A
        if (dat == 4) Areg = PB >> 4;	// Din->A
        if (dat == 5){					// 5-8 reads Pins Din0-3 to bit 0 of A
			digitalRead Areg,PB,Din0	
			}
        if (dat == 6){
			digitalRead Areg,PB,Din1
			Areg &=1;
			}
        if (dat == 7){
			digitalRead Areg,PB,Din2
			Areg &=1;
			}
        if (dat == 8){
			digitalRead Areg,PB,Din3
			Areg &=1;
			}
        if (dat == 9){				// 9: reads 'adc' (via comparator) to A
			analogRead();
			Areg=A;
			}
        break;
      case 7:						// aritmetic instructions
        if (dat == 1) Areg++;		// A=A=1
        if (dat == 2) Areg--;		// A=A-1
        if (dat == 3) Areg += Breg;	// A=A+B
        if (dat == 4) Areg -= Breg;	// A=A-B
        if (dat == 5){				// A=A*B
			rtmp=Breg;
			stmp=0;
			while(rtmp){
				stmp +=Areg;
				rtmp--;
				}
				Areg=stmp;
			} 
        if (dat == 6){				// A=A/B
			stmp=0xff;
			rtmp=Areg;
			while(!rtmp.7){
				stmp++;
				rtmp-=Breg;
				}
			Areg=stmp;
			}
        if (dat == 7) Areg &= Breg;	// A=A and B
        if (dat == 8) Areg |= Breg;	// A=A or B
        if (dat == 9) Areg ^= Breg;	// A=A xor B	
        if (dat == 10) Areg ^= 0x0f;	// A=not A
        Areg &= 0x0f;				// always leave A=0000xxxx
        break;
      case 8:
        adrhi = dat;				// 8: set address high nibble (EEPROM page)
        break;
      case 9:
        adr = (adrhi << 4) + dat;	// 9: Jump set instruction pointer to addrh*16 + data
        break;
      case 10:						// 10: decrement C and jump if non zero (paged jump
        Creg--;
        Creg &= 0x0f;
        if (Creg > 0)
          adr = (adrhi << 4) + dat;
        break;
      case 11:						// 11:decrement D and jump if non zero (paged jump
        Dreg--;
        Dreg &= 0x0f;
        if (Dreg > 0)
          adr = (adrhi << 4) + dat;
        break;
      case 12:						// 12: conditional skip instructions
        if (dat == 1 && Areg > Breg) adr++;	// skip if A>B
        if (dat == 2 && Areg < Breg) adr++;	// skip if A<B
        if (dat == 3 && Areg == Breg) adr++;	// skip if A=B
        if (dat == 4){							// skip if Pin0=1
			digitalRead rtmp,PB,Din0
			if(A) adr++;
			}
        if (dat == 5){							// skip if Pin1=1
			digitalRead rtmp,PB,Din1
			if(A) adr++;
			}
        if (dat == 6){							// skip if Pin2=1
			digitalRead rtmp,PB,Din2
			if(A) adr++;
			}
        if (dat == 7){							// skip if Pin3=1
			digitalRead rtmp,PB,Din3
			if(A) adr++;
			}
        if (dat == 8){							// skip if Pin0=0
			digitalRead rtmp,PB,Din0
			if(!A) adr++;
			}
        if (dat == 9){							// skip if Pin1=0
			digitalRead rtmp,PB,Din1
			if(!A) adr++;
			}
        if (dat == 10){							// skip if Pin2=0
			digitalRead rtmp,PB,Din2
			if(!A) adr++;
			}
        if (dat == 11){							// skip if Pin3=0
			digitalRead rtmp,PB,Din3
			if(!A) adr++;
			}
        if (dat == 12){							// skip if Switch1=0 (pressed)
			digitalRead rtmp,PA,Sw1
			if(!A) adr++;
			}
        if (dat == 13){							// skip if Switch2=0 (pressed)
			digitalRead rtmp,PA,Sw2
			if(!A) adr++;
			}
        if (dat == 14){							// skip if Switch1=1 (NOT pressed)
			digitalRead rtmp,PA,Sw1
			if(A) adr++;
			}
        if (dat == 15){							// skip if Switch2=1 (NOT pressed
			digitalRead rtmp,PA,Sw2
			if(A) adr++;
			}
        break;
      case 13:									// single level subroutine call
        addret = adr;							// to addrh*16+data
        adr = (adrhi << 4) + dat;
        break;
      case 14:									// subroutine return
        adr = addret;
        break;
    }
  }
}



void	programming() {
byte prog = 0;
byte com,dat;
  adr = 0;
  while (1) {
    PB = adr;
	A=8;
    delayXmS();			// delay 500 mS
	A=adr;
	EEread();
    eebyte = A;
    dat = eebyte & 0x0f;
    com = eebyte >> 4;
    PB = com;
	while (!PA.Sw2){
		A=1;
		delayXmS();	// delay 2mS
	}
	A=5;
    delayXmS();			// delay(50);
    prog = 1;
    while(prog){
      if (!PA.Sw1) {
        if (prog == 1) {
          prog = 2;
          com = 15;
        }
        if (prog == 2) {
          com++;
          com &= 0x0f;
		  PB = com;
        }
        if (prog == 3) {
          prog = 5;
          dat = 15;
        }
        if (prog == 4) {
          prog = 5;
          dat = 15;
        }
        if (prog == 5) {
          dat++;
          dat &= 0x0f;
		  PB = dat;
        }
		A=5;
        delayXmS();			// delay 50 mS
       while (!PA.Sw1){
		   A=2;
		   delayXmS();	// delay 5 mS
		}
      }
    if (!PA.Sw2) {
      if (prog == 3) prog = 7;
      if (prog == 1) {
	    PB = dat;
        prog = 3;
      }
      if (prog == 4) {
	    PB = dat;
        prog = 6;
      }
      if (prog == 2) {
	    PB = dat;
        prog = 4;
      }
      if (prog == 6) {
        dat &= 0x0f;
        eebyte = (com << 4) + dat;
			// write to program
			A=eebyte;
	        EEwrite();
	    PB = 0;
		A=8;
        delayXmS();		// delay for 500 mS
		A=6;
        delayXmS();		// + delay for 100 mS =600mS
        adr++;
        prog = 0;
      }
      if (prog == 5) {
        dat &= 0x0f;
        eebyte = (com << 4) + dat;
	    PB = adr;
		A=9;
        delayXmS();		// delay(1000);
		A=eebyte;
	    EEwrite();
	    PB = 0;
		A=8;
        delayXmS();		// delay for 500 mS
		A=6;
        delayXmS();		// + delay for 100 mS =600mS
        adr++;
        prog = 0;
      }
      if (prog == 7) {
        adr++;
        prog = 0;
      }
      while(!PA.Sw2){
		  A=2;
		  delayXmS();	// delay 5mS
		}
	  A=5;
      delayXmS();		// delay(50);
		}
    }
  }
}


void	FPPA0 (void)
{
	.ADJUST_IC	SYSCLK=IHRC/2		//	SYSCLK=IHRC/2
//	set1 CLKMD.0
	//	Insert Initial Code
	disgint;						// disable in ts for now
	millis=392;						// timer 16 value
	$ T16M	IHRC, /1, BIT13;		// timer16 uses IHRC divide by one....
	Inten.T16=1;					// ... and interrupts on bit 13
	stt16	millis					// set timer count
	engint;							// now we can interrupt
	PAC=1 << SDA_BIT | 1 << SCL_BIT | 1 << PWMpin;		// setup port A	FOR I2C and PWM
	PAPH=1 << Sw1 | 1 << Sw2 | 1 << ADCpin  ; 	// set pullups
	PBC=1 << Dout0 | 1 << Dout1 | 1 << Dout2 | 1 << Dout3;	// set Data out and inports
	PBPH=1 << Din0 | 1 << Din1 | 1 << Din2 | 1 << Din3 ;
	pwmg0dtl=0;						// turn off PWM
	pwmg0dth=0;
	mov	a,0b11101000				// set ADC pin high
	mov	PADIER,a
	gpcs=0b00_0_0_00111;			// setup comparator
	gpcc=0b10000111;
	while (1)						// main spins loop
	{
	if(!PA.Sw2)					// we started with switch 2 pressed so input new prog
		programming();
	else
		RunProg();				// else run .......
	}
}
//
// enter A=servo position value 0-15 0=turn off else 100-200 Ms
// for servo
//
void doServo(void){
byte atmp=A;
byte ltmp;
	pwmg0c=0b1_00_1_0111;		// set PWM up
	pwmg0s=0b0_11_00100;
	pwmg0cubl=0b000_0000;
	pwmg0cubh=0b11111010;
	pwmg0dtl=0;		// duty cycle low...
	pwmg0dth=0;		// ... high
	a=atmp;
	Servotab();			// read PWM value
	atmp=A;				// save two copies
	ltmp=A;
	ltmp &=0x07;		// mask lower 3 bits
	ltmp <<=5;			// move to bits 5..7
	atmp >>=3;			// shift top 5 bits to 0..4
	pwmg0dtl=ltmp;		// duty cycle low...
	pwmg0dth=atmp;		// ... high
}

//
// enter A=PWM value 0-15 0=turn off else 100-200 Ms
// for servo
//
void doPWM(void){
byte patmp=A;
byte pltmp;
	pwmg0c=0b10000111;		// set PWM up
	pwmg0s=0b00000000;		
	pwmg0cubl=0b00100001;
	pwmg0cubh=0b11111010;
	pwmg0dtl=0;			// duty cycle low...
	pwmg0dth=0;			// ... high
	A=patmp;
	pwmtab();			// read PWM value low
	pltmp=A;
	A=patmp;
	pwmtab();			// and high value
	patmp=A;	
	pwmg0dtl=pltmp;		// duty cycle low...
	pwmg0dth=patmp;		// ... high
}

// timer 16 interrupt used to count milli seconds

void	Interrupt (void)
{
	pushaf;
	if (Intrq.T16)
	{
		running +=0x10;				// bump 'millis' counter
		stt16	millis				// reset base timer offset
		Intrq.T16	=	0;			// clear interrupt
		//...
	}	
	popaf;
}

// delay X millsecosnds enter A=index to # of millis#

void	delayXmS(){
word ldel;
byte tmp=A;						// save index
	delVals();					// get real numebr of millis to delay
	ldel$0=A;					// save low byte ....
	A =tmp+16;					// ... index to high bytes 
	delVals();					// ... get it ...
	ldel$1=A;					// save in word
	do{							// now just hang about for ldel millis
		tmp=running;			// get current counter
		while(tmp==running){	// wai for next tick
			;
		}
	}while(--ldel);				// doit till ldel==0
}





// delay value table as lo high bytes 

void	delVals(void)
{
	A++;
	_pcadd
	{
// low bytes
		ret	1
		ret	2
		ret	5
		ret	10
		ret	20
		ret	50
		ret	100
		ret	200
		ret	500 & 0xff
		ret	1000 & 0xff
		ret	2000 & 0xff
		ret	5000 & 0xff
		ret	10000 & 0xff
		ret	20000 & 0xff
		ret	30000 & 0xff
		ret	60000 & 0xff
// high bytes
		ret	0
		ret	0
		ret	0
		ret	0
		ret	0
		ret	0
		ret	0
		ret	0
		ret	500 >> 8
		ret	1000 >> 8
		ret	2000 >> 8
		ret	5000 >> 8
		ret	10000 >> 8
		ret	20000 >> 8
		ret	30000 >> 8
		ret	60000 >> 8
	}
}
//
// enter A=Servo value 0=off 1-15=1->2 Ms
//
void	Servotab(void){
	A++;
	_pcadd{
	ret	0		// turn PWM off
	ret	100		// 1Ms
	ret	107
	ret	114
	ret	121
	ret	128
	ret	135
	ret	142
	ret	150		// 1.5MS
	ret	157
	ret	164
	ret	171
	ret	178
	ret	185
	ret	192
	ret	200		// 2 Ms
	}
}
/*
 enter A=PWM value or value+16
 to get low and high bytes of bounds value
*/
void	pwmtab(void)
{
	A++;
	_pcadd
	{
// low bytes
		ret		0x00
		ret		0x00
		ret		0x20
		ret		0x20
		ret		0x40
		ret		0x40
		ret		0x60
		ret		0x60
		ret		0x80
		ret		0x80
		ret		0xA0
		ret		0xA0
		ret		0xC0
		ret		0xC0
		ret		0xE0
		ret		0xe0
//high bytes
		ret		0x08
		ret		0x11
		ret		0x22
		ret		0x33
		ret		0x44
		ret		0x55
		ret		0x66
		ret		0x77
		ret		0x88
		ret		0x99
		ret		0xAA
		ret		0xBB
		ret		0xCC
		ret		0xDD
		ret		0xEE
		ret		0xFF
	}
}
