/***********************************************************************
    2011-2013 (C) Alex Dobrianski memory storage/camera/backup comm/command controller module
    works with OpenLog https://github.com/nseidle/OpenLog/wiki

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

    Design and development by Team "Plan B" is licensed under 
    a Creative Commons Attribution-ShareAlike 3.0 Unported License.
    http://creativecommons.org/licenses/by-sa/3.0/ 
************************************************************************/
/***********************************************************************
see www.adobri.com for communication protocol spec
************************************************************************/
// [1 GYRO]->[2 MEM]-> [3 POW] -> [4 STM] -> [5 BT] -| 
//  A                                                |
//  --------------------------------------------------
////////////////////////////////////////////
// listing in C30
// -Wa,-ahlsnd="$(BINDIR_)$(INFILEBASE).lst"
// -g - debugging information
// -O1 - optimization looks good
// -02 and -O3 does something that code is not recognizable - can be dangerouse
// -fpack-struct pack struct
// -save-temps 
// -funroll-loops this will make loops big, ugly but fast
////////////////////////////////////////////
// -g -Wall -save-temps -O1 -Wa,-ahlsnd="$(BINDIR_)$(INFILEBASE).lst"
////////////////////////////////////////////

#define MY_UNIT '2' 
#define MAX_COM_TOPOLOGY_UNIT '2'

//#define ALLOW_RELAY_TO_NEW
#ifdef __18CXX
#ifdef __16F88
#define _16F88 1
#endif
#ifdef __16F884
#define _16F884 1
#endif
#ifdef __16F724 // the same as 16F884
#define _16F884 1
#endif
#ifdef __18F2321
#define _18F2321 1
#endif
#endif
#ifdef __18F25K20
#define _18F25K20 1
#endif

////////////////////////////////////////////
// listing in C30
// -Wa,-ahlsnd="$(BINDIR_)$(INFILEBASE).lst"
// -g - debugging information
// -O1 - optimization looks good
// -02 and -O3 does something that code is not recognizable - can be dangerouse
// -fpack-struct pack struct
// -save-temps 
// -funroll-loops this will make loops big, ugly but fast
////////////////////////////////////////////
// -g -Wall -save-temps -O1 -Wa,-ahlsnd="$(BINDIR_)$(INFILEBASE).lst"
////////////////////////////////////////////

// it can be only master support: pic works in master mode only=> uncomment this line if 
//     no multimaster support on a bus
#define I2C_ONLY_MASTER 1

// master support done via interrupts and firmware - commenting next line and I2C will be a software work
#define I2C_INT_SUPPORT 1


#ifdef _PIC16F88
#include "int16CXX.H"
#define RAM_BANK_0 0
#define RAM_BANK_1 1
#define RAM_BANK_2 2
#define RAM_BANK_3 3
#define TIMER0_CONTROL_REG OPTION_REG
#define TIMER0_BYTE TMR0
#define TIMER0_INT_ENBL TMR0IE
#define TIMER0_INT_FLG  TMR0IF
#define FSR_REGISTER FSR
#define PTR_FSR INDF
#define INT0_EDG INTEDG
#define INT0_FLG INT0IF
#define INT0_ENBL INT0IE
#define I2CPORT PORTB
#define I2CTRIS TRISB
#define I2C_SDA 1
#define I2C_SCL 4
#define TIMER0_INT_ENBL TMR0IE
#define TIMER0_INT_FLG  TMR0IF
#endif

#ifdef _PIC16F884
#include "int16CXX.H"
#define RAM_BANK_0 0
#define RAM_BANK_1 1
#define RAM_BANK_2 2
#define RAM_BANK_3 3
#define TIMER0_CONTROL_REG OPTION_REG
#define TIMER0_BYTE TMR0
#define TIMER0_INT_ENBL T0IE
#define TIMER0_INT_FLG  T0IF
#define FSR_REGISTER FSR
#define PTR_FSR INDF
#define INT0_EDG INTEDG
#define INT0_FLG INTF
#define INT0_ENBL INTE
#define I2CPORT PORTB
#define I2CTRIS TRISB
#define I2C_SDA 1
#define I2C_SCL 4
#endif


#ifdef __PIC24H__

#define INIT_CAMERA PutsCom2("\x56\x00\x26\x00");
#define TAKE_PICTURE PutsCom2("\x56\x00\x36\x01\x00");
#define GET_PICTURE_LEN PutsCom2("\x56\x00\x34\x01\x00");
#define PICTURE_640_480 PutsCom2("\x56\x00\x31\x05\x04\x01\x00\x19\x00");
#define PICTURE_320_240 PutsCom2("\x56\x00\x31\x05\x04\x01\x00\x19\x11");
#define PICTURE_160_120 PutsCom2("\x56\x00\x31\x05\x04\x01\x00\x19\x22");

#define HD_Camera_On   bset(PORTAbits,RA8);
#define HD_Camera_Off  bclr(PORTAbits,RA8);

#define HD_Camera_Click_Press   bset(PORTAbits,RA9); 
#define HD_Camera_Click_Release bclr(PORTAbits,RA9); 

#define HD_Camera_Video_Press   bset(PORTCbits,RC3); 
#define HD_Camera_Video_Release bclr(PORTCbits,RC3); 

#define FLASH_DISK_to_MEM       bset(PORTCbits,RC4); 
#define FLASH_DISK_to_HD_camera bclr(PORTCbits,RC4); 

#define GPS_On                  bset(PORTCbits,RC5); 
#define GPS_Off                 bclr(PORTCbits,RC5); 

#define COM_to_Camera  bset(PORTBbits,RB12); bset(PORTBbits,RB11);
#define COM_to_MEM     bclr(PORTBbits,RB12); bset(PORTBbits,RB11);
#define COM_to_backup  bset(PORTBbits,RB13); bclr(PORTBbits,RB11);
#define COM_to_GPS     bclr(PORTBbits,RB13); bclr(PORTBbits,RB11);

// SSCLOCK RA0(pin2), SSDATA_IN RA1(pin3), SSDATA_OUT RA2(pin9), SSCS RA3(pin10)
//#define SSPORT  LATAbits
//#define SSCLOCK LATA0
//#define SSDATA_IN LATA1
//#define SSDATA_OUT LATA2
//#define SSCS       LATA3

#define SSPORT  PORTAbits
#define SSCLOCK RA0
#define SSDATA_IN RA1
#define SSDATA_OUT RA2
#define SSCS       RA3

#else
// SSCLOCK RA7(pin9), SSDATA_IN RA6(pin10), SSDATA_OUT RA4(pin6), SSCS RA3(pin5)
#define SSPORT PORTA
#define SSCLOCK 7
#define SSDATA_IN 6
#define SSDATA_OUT 4
#define SSCS       3
#endif



#include "commc0.h"


//
// additional code:
#pragma rambank RAM_BANK_3
struct _DataB3{
unsigned FlashWrite:1;
unsigned FlashWriteLen:1;
unsigned FlashRead:1;
} DataB3;
unsigned char CountWrite;

#pragma rambank RAM_BANK_1


#include "commc1.h"
unsigned char CallBkComm(void); // return 1 == process queue; 0 == do not process; 
                                // 2 = do not process and finish process 3 == process and finish internal process
                                // in case 0 fucntion needs to pop queue byte by itself

unsigned char CallBkI2C(void); // return 1 == process queue; 0 == do not process; 
                               // 2 = do not process and finish process 3 == process and finish internal process
                               // in case 0 fucntion needs to pop queue byte by itself
unsigned char CallBkMain(void); // 0 = do continue; 1 = process queues
void Reset_device(void);
void ShowMessage(void);
void ProcessCMD(unsigned char bByte);
unsigned char getchI2C(void);
void putch(unsigned char simbol);
void putchWithESC(unsigned char simbol);
unsigned char getch(void);

void main()
{
    unsigned char bWork;
    /*if (POR_) // this is can be sync of a timer from MCLR
    {
        if (SetSyncTime)
        {
            TMR1L = 0; // must be delay in 2MHz clock
            TMR1H = 0;
            TMR130 = setTMR130;
            TMR1SEC = setTMR1SEC;
            TMR1MIN = setTMR1MIN;
            TMR1HOUR = setTMR1HOUR;
            TMR1DAY = setTMR1DAY;
        }
    }*/
    
    Reset_device();
    // needs to check what is it:

    //if (TO) // Power up or MCLR
    {
       // Unit == 1 (one) is ADC and unit == 2 (two) is DAC 
        // Unit == 3 Gyro
        //UnitADR = '1';
        UnitADR = '2'; // mem/ camera/ backup comm/ unit 2
        //UnitADR = '4';
#include "commc6.h"

    }
#ifndef __PIC24H__
    PEIE = 1;    // bit 6 PEIE: Peripheral Interrupt Enable bit
                 // 1 = Enables all unmasked peripheral interrupts
                 // 0 = Disables all peripheral interrupts
    GIE = 1;     // bit 7 GIE: Global Interrupt Enable bit
                 // 1 = Enables all unmasked interrupts
                 // 0 = Disables all interrupts
    RBIF = 0;
#endif
    ShowMessage();
    bitset(PORTA,3);
    //bitset(SSPCON,4);  // set clock high;
#include "commc7.h"
///////////////////////////////////////////////////////////////////////

} // at the end will be Sleep which then continue to main


#define SPBRG_9600 51
#define SPBRG_19200 25
#define SPBRG_38400 12
#define SPBRG_57600 8
#define SPBRG_SPEED SPBRG_9600

#include "commc2.h"
// additional code:

void ProcessCMD(unsigned char bByte)
{
    unsigned char bWork;
    long wWork;
    long *FileLen;
    if (!Main.getCMD) // CMD not receved et.
    {

#include "commc3.h"
       
// additional code:

#include "commc4.h"
// additional code:
        //else if (bByte == 'F') // set file name
        
SKIP_BYTE:;
    } // do not confuse: this is a else from Main.getCMD == 1
}

unsigned char CallBkComm(void) // return 1 == process queue; 0 == do not process; 
                               // 2 = do not process and finish process 3 == process and finish internal process
{                              // in case 0 fucntion needs to pop queue byte by itself
    unsigned char bBy;
    //if (_Flags_Ctrl.READSend) // is this retransmit over I2C the file?
    //{
    //    //bBy = AInQu.Queue[AInQu.iExit]; // function called when Queue is not empty; check next pop byte in queue
    //}
    return 1; // this will process next byte 
}
unsigned char CallBkI2C(void)
{
    return 1;
}
unsigned char CallBkMain(void) // 0 = do continue; 1 = process queues
{
    //if (Timer0Waiting)
    //{
    //    if (Timer0Fired)
    //        Timer0Waiting = 0;
    //    else
    //        return 0;
    //}
    return 1;
}
#pragma codepage 1
void Reset_device(void)
{
#ifdef __PIC24H__
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
	PLLFBD=42; // 42// M=40
           // for value = 50 it can be 92.125MHz and 46MIPS needs to make shure that FIN will be more then 4MHz and less 8MHz
           // OCTUN set more then 8MHz can be wrong
	CLKDIVbits.PLLPOST=0;		// N2=2 PLL VCO Output Divider Select bits (also denoted as µN2¦, PLL postscaler)
                                // 00 = Output/2
	CLKDIVbits.PLLPRE=0;		// N1=2 PLL Phase Detector Input Divider bits (also denoted as µN1¦, PLL prescaler)
                                // 00000 = Input/2 (default)
	OSCTUN=0;//0x14;                // Tune FRC oscillator, if FRC is used
                                // bit 5-0 TUN<5:0>: FRC Oscillator Tuning bits(1)
                                // 111111 = Center frequency -0.375% (7.345 MHz)
                                // 100001 = Center frequency -11.625% (6.52 MHz)
                                // 100000 = Center frequency -12% (6.49 MHz)
                                // 011111 = Center frequency +11.625% (8.23 MHz)
                                // 011110 = Center frequency +11.25% (8.20 MHz)
                                // 000001 = Center frequency +0.375% (7.40 MHz)
                                // 000000 = Center frequency (7.37 MHz nominal)
                                // 010100 = 8MHz

	RCONbits.SWDTEN=0;          // Disable Watch Dog Timer ???

    // Clock switch to incorporate PLL
	__builtin_write_OSCCONH(0x01);		// Initiate Clock Switch to Primary
                                        // Oscillator with PLL (NOSC=0b011) 
	__builtin_write_OSCCONL(0x01);		// Start clock switching
                                        // bit 7 CLKLOCK: Clock Lock Enable bit 0 = Clock switching is enabled, system clock source can be modified by clock switching
                                        // bit 6 IOLOCK: Peripheral Pin Select Lock bit 0 = Peripherial pin select is not locked, write to peripheral pin select registers allowed
                                        // bit 5 LOCK: PLL Lock Status bit (read-only)
                                        // bit 4 Unimplemented: Read as µ0¦
                                        // bit 3 CF: Clock Fail Detect bit (read/clear by application)
                                        // bit 2 Unimplemented: Read as µ0¦
                                        // bit 1 LPOSCEN: Secondary (LP) Oscillator Enable bit 0 = Disable secondary oscillator
                                        // bit 0 OSWEN: Oscillator Switch Enable bit 1 = Request oscillator switch to selection specified by NOSC<2:0> bits
	while (OSCCONbits.COSC != 0b001);	// Wait for Clock switch to occur check for:
                                        // 011 = Primary oscillator (XT, HS, EC) with PLL

	while(OSCCONbits.LOCK!=1) {};       //  Wait for PLL to lock
                                        // bit 5 LOCK: PLL Lock Status bit (read-only)
                                        //    1 = Indicates that PLL is in lock, or PLL start-up timer is satisfied
                                        //    0 = Indicates that PLL is out of lock, start-up timer is in progress or PLL is disabled

//                                                       PIC24HJ128GP504
// I2C SDA            SDA1/RP9(1)/CN21/PMD3/RB9 |pin1              pin44| SCL1/RP8(1)/CN22/PMD4/RB8           I2C SCL
//                        RP22(1)/CN18/PMA1/RC6 |pin2              pin43| INT0/RP7(1)/CN23/PMD5/RB7           COM2 =>
//                        RP23(1)/CN17/PMA0/RC7 |pin3              pin42| PGEC3/ASCL1/RP6(1)/CN24/PMD6/RB6 <= COM1 RX
//                        RP23(1)/CN17/PMA0/RC7 |pin4              pin41| PGED3/ASDA1/RP5(1)/CN27/PMD7/RB5    COM1 TX =>
//                        RP25(1)/CN19/PMA6/RC9 |pin5              pin40| VDD                   3.3
// GND                                      VSS |pin6              pin39| VSS                   GND
// capasitor                            VCAP(2) |pin7              pin38| RP21(1)/CN26/PMA3/RC5 ON/OFF power GPS
// =>COM2 RX PGED2/EMCD2/RP10(1)/CN16/PMD2/RB10 |pin8              pin37| RP20(1)/CN25/PMA4/RC4 Switch MicroSD -> HDcam / MEM
// <=SW3 COM2      PGEC2/RP11(1)/CN15/PMD1/RB11 |pin9              pin36| RP19(1)/CN28/PMBE/RC3 HD Camera VIDEO record
// <=SW1 COM2       AN12/RP12(1)/CN14/PMD0/RB12 |pin10             pin35| TDI/PMA9/RA9          HD Camera click1   
// <=SW2 COM2       AN11/RP13(1)/CN13/PMRD/RB13 |pin11             pin34| SOSCO/T1CK/CN0/RA4    32.kHz quartz

//                               TMS/PMA10/RA10 |pin12             pin33| SOSCI/RP4(1)/CN1/RB4  32.kHz quartz
//                                 TCK/PMA7/RA7 |pin13             pin32| TDO/PMA8/RA8          HD Camera ON/OFF
//                  AN9/RP15(1)/CN11/PMCS1/RB15 |pin14             pin31| OSC2/CLKO/CN29/RA3    SSCS =>
//                  AN9/RP15(1)/CN11/PMCS1/RB15 |pin15             pin30| OSC1/CLKI/CN30/RA2    SSDATA_OUT =>
// GND                                     AVSS |pin16             pin29| VSS                   GND
// 3.3                                     AVDD |pin17             pin28| VDD                   3.3 V
// RESET(brown)                            MCLR |pin18             pin27| AN8/CVREF/RP18(1)/PMA2/CN10/RC2 <==> TEMP (PCB)
// <=SSCLOCK                  AN0/VREF+/CN2/RA0 |pin19             pin26| AN7/RP17(1)/CN9/RC1              <= analog Q4 IR
// =>SSDATA_IN                AN1/VREF-/CN3/RA1 |pin20             pin25| AN6/RP16(1)/CN8/RC0              <= analog Q3 IR
// (white)       PGED1/AN2/C2IN-/RP0(1)/CN4/RB0 |pin21             pin24| AN5/C1IN+/RP3(1)/CN7/RB3         <= analog Q2 IR
// (gray)        PGEC1/AN3/C2IN+/RP1(1)/CN5/RB1 |pin22             pin23| AN4/C1IN-/RP2(1)/CN6/RB2         <= analog Q1 IR


    // disable analog
	AD1CON1bits.ADON = 0; 
    // and switch analog pins to digital
    AD1PCFGL = 0xffff;
    //AD1PCFGH = 0xffff;
    // porta is not re-mappable and on 502 device it is RA0-RA4
    // SPI output in FLASH mem terminoligy:
    // SSCLOCK RA0(pin2), SSDATA_IN RA1(pin3), SSDATA_OUT RA2(pin9), SSCS RA3(pin10)
    //          0            0                        IN                  1
    TRISA = 0b00000100;  //0 = Output, 1 = Input 
    PORTA = 0b00001000;
    // this kaind funny, and for PIC24 and up = PRX pins can be reassigned to differrent preferias
    // additionaly needs to remember on which pin sits which PRX : // VSIAK SVERCHOK ZNAI SVOI SHESTOK
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock port remapping
    // INT0 == pin 16 
    // INT1 == pin 21 == PR10 
    IN_FN_PPS_INT1 = IN_PIN_PPS_RP10; // RPINR0 
    // INT2 == pin 22 == PR11
    IN_FN_PPS_INT2 = IN_PIN_PPS_RP11;
    
    // PR5 - Serial RX  Pin 14
    IN_FN_PPS_U1RX = IN_PIN_PPS_RP5;
	// RR6 - Serial TX  Pin 15
    OUT_PIN_PPS_RP6 = OUT_FN_PPS_U1TX;

    // I2C:
    // SCL1 = I2C clock Pin 17 (this is NOT alernative I2c set as FPOR = 1 in configuration) 
    // SDA1 = I2C data Pin 18  this two pins permamet
    __builtin_write_OSCCONL(OSCCON | 0x40); //lock back port remapping
     
    //RBPU_ = 0;
    //bitclr(OPTION,RBPU_); //Each of the PORTB pins has a weak internal pull-up. A
                          //single control bit can turn on all the pull-ups. This is
                          //performed by clearing bit RBPU (OPTION_REG<7>).
                          //The weak pull-up is automatically turned off when the
                          //port pin is configured as an output. The pull-ups are
                          //disabled on a Power-on Reset.

    INT0_ENBL = 0; // disable external interrupt for GYRO 1
    INT1IE = 0;    // disable external interrupt for GYRO2
    enable_uart(); //Setup the hardware UART for 20MHz at 9600bps
    // next two bits has to be set after all intialization done
    //PEIE = 1;    // bit 6 PEIE: Peripheral Interrupt Enable bit
                 // 1 = Enables all unmasked peripheral interrupts
                 // 0 = Disables all peripheral interrupts
    //GIE = 1;     // bit 7 GIE: Global Interrupt Enable bit
                 // 1 = Enables all unmasked interrupts
                 // 0 = Disables all interrupts
    enable_I2C();
    TIMER0_INT_FLG = 0; // clean timer0 interrupt
    TIMER0_INT_ENBL = 0; // diasable timer0 interrupt
    TMR1IF = 0; // clean timer0 interrupt
    TMR1IE = 0; // diasable timer0 interrupt
    INT0_EDG = 1; // 1 = Interrupt on negative edge
    INT0_FLG = 0; // clean extrnal interrupt RB0 pin 6

    INT1IF = 0;
    INTEDG1 = 1;    

    INT2IF = 0;
    INTEDG2 = 1;    

#else
#endif
}


void ShowMessage(void)
{
    // if message initiated by unit needs to check then it is possible to do:
    while(!Main.prepStream) // this will wait untill no relay message
    {
    }
    // in a case of a CMD replay it is safly to skip that check - unit allow to send message in CMD mode
    putch(UnitADR);  // this message will circle over com and will be supressed by unit
    Puts("~");
    putch(UnitADR);
}
#include "commc8.h"
