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

// use serial 2 port
#define USE_COM2 1


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
unsigned FlashCmd:1;
unsigned FlashCmdLen:1;
unsigned FlashRead:1;
} DataB3;
unsigned char CountWrite;

#pragma rambank RAM_BANK_1


//#include "commc1.h"
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// begin of COPY 1
///////////////////////////////////////////////////////////////////////
//#pragma rambank RAM_BANK_2
//unsigned char CMD[4];
//bit CmdFetch;
//bit ExecCMD;
//bit ReqSendMsg;
//#pragma rambank RAM_BANK_0

//unsigned char CommLinesOld;
#ifdef __PIC24H__
#ifdef HI_TECH_C
#define IF_SSPIF void interrupt _MI2C1Interrupt(void) @ MI2C1_VCTR
#define IF_RCIF void interrupt _U1RXInterrupt(void) @ U1RX_VCTR
#define IF_TXIE void interrupt _U1TXInterrupt(void) @ U1TX_VCTR
#define IF_TIMER0_INT_FLG void interrupt _T1Interrupt(void) @ T1_VCTR
#define IF_TMR1IF void interrupt _T2Interrupt(void) @ T2_VCTR
#ifdef BT_TIMER3
#define IF_TMR3IF void interrupt _T3Interrupt(void) @ T4_VCTR
#endif
#define IF_INT0_FLG void interrupt _INT0Interrupt(void) @ INT0_VCTR
#define IF_INT1IF void interrupt _INT1Interrupt(void) @ INT1_VCTR
#define IF_INT2IF void interrupt _INT2Interrupt(void) @ INT2_VCTR
#else // hitech ends
//#define _ISR __attribute__((interrupt))
//#define _ISRFAST __attribute__((interrupt, shadow))
#define _ISR __attribute__((interrupt))
#define INTERRUPT void __attribute__((interrupt, auto_psv))
#define IF_SSPIF void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
#define IF_RCIF void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
#define IF_TXIE void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
#define IF_TIMER0_INT_FLG void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
#define IF_TMR1IF void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
#ifdef BT_TIMER3
#define IF_TMR3IF void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
#endif
#define IF_INT0_FLG void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
#define IF_INT1IF void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
#define IF_INT2IF void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
#define IF_RCERRORCOM1 void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void)

#ifdef USE_COM2
#define IF_RCIFCOM2 void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void)
#define IF_TXIECOM2 void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void)
#define IF_RCERRORCOM2 void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void)
#endif

#define IF_SLAVEI2C void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void)

#endif // C30 ends
#else // end of C30 support
#define IF_SSPIF if (SSPIF)
#define IF_BCLIF if (BCLIF)
#define IF_RCIF  if (RCIF)
#define IF_TXIE  if (TXIE)
#define IF_TIMER0_INT_FLG if (TIMER0_INT_FLG)
#define IF_TMR1IF if (TMR1IF)
#ifdef BT_TIMER3
#define IF_TMR3IF if (TMR3IF)
#endif
#define IF_INT0_FLG if (INT0_FLG)
#define IF_INT1IF if (INT1IF)
#ifdef __18CXX
#pragma interrupt int_server
#define INTERRUPT void
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
    _asm
         goto int_server
    _endasm
}

#else // CC5 CC8

#define INTERRUPT interrupt
#pragma codeLevel 1
#pragma optimize 1
 #ifdef _18F2321_18F25K20 
 #else 
#include "inline.h"
 #endif
#pragma origin 4
#pragma rambank RAM_BANK_0
/////////////////////////////////////////////BANK 0//////////////////////
#ifdef _18F2321_18F25K20  // comaptability mode == branch on 0ffset 8 and no priority
#pragma origin 8
#endif

#endif // __18CXX || CC5 || CC8

#endif // end of NOT __C30
#ifndef __PIC24H__
INTERRUPT int_server( void)
{
//    unsigned char CommLines;
//    unsigned char DeltaTime;
//    unsigned char TransferBits;
#ifdef __18CXX
#else
    int_save_registers ;   // W, STATUS (and PCLATH)
   
 #ifdef _18F2321_18F25K20
   unsigned long sv_FSR = FSR_REGISTER;
 #else
   unsigned char sv_FSR = FSR_REGISTER;  // save FSR if required
 #endif
#endif

   unsigned char work1;
   unsigned char work2;
#endif

   //////////////////////////////////////////////////////////////////////////////////////////
   IF_SSPIF    //if (SSPIF)    // I2C interrupt
   {
#ifdef __PIC24H__
       unsigned int work2;
       register unsigned int W;
#endif
       SSPIF = 0; // clean interrupt
       if (BF)     // receved salve mode byte - buffer is full
       {
#ifdef I2C_INT_SUPPORT
          
          if (I2C_B1.NeedReceive) // needs to receive data
          {
                if (I2C_B1.SendACK_NAK)
                   goto RECV_NEXTBYTE;

               //  work2 = SSPBUF_RX;
               //ACKDT = 0;
               ACKEN = 1;  // send ACK/NACK it will be another SSPIF set at the end of ACK 
                           // but delay in storage will deal with it 
               I2C_B1.SendACK_NAK = 1;
               goto STORE_BYTES;
          }
//          if (I2C_B1.NeedSend) // master need to send + buffer is full - we in wronng place in wrong time
//              goto CHECK_ANOTHER;
#endif
           if (DA_) // 1 = this is byte of data  
           {
               //work2 = SSPBUF;
STORE_BYTES:
               if (AInI2CQu.iQueueSize < BUFFER_LEN)
               {
                   AInI2CQu.Queue[AInI2CQu.iEntry] = SSPBUF_RX;//work2;
                   if (++AInI2CQu.iEntry >= BUFFER_LEN)
                       AInI2CQu.iEntry = 0;
                   AInI2CQu.iQueueSize++;
               }
               else
                   W = SSPBUF_RX; // this byte is skipped to process = no space in a queue
#ifdef I2C_INT_SUPPORT
 //              if (I2C_B1.NeedReceive) // master asked to receive data
 //              {
               if (SSPIF)
               {
                   SSPIF = 0;
                   goto RECV_NEXTBYTE;
               }

               goto MAIN_EXIT;

//WAIT_FOR_SSPIF:
//                   if (!SSPIF)               // must be another interrupt flag set after ACKEN = 1;
//{
//work1 =22;
//                       goto WAIT_FOR_SSPIF;
//}
//                   SSPIF = 0;
//                   //if (!ACKEN)               // must be cleared
//                   //    SSPIF = 0;
//
//                   if (--LenI2CRead)
//                   {
//                       if (LenI2CRead == 1)  // for a last receiving byte prepear NAK
//                           ACKDT = 1;
//                       RCEN = 1; // Receive Enable for a next byte
//                       goto ExitIntr;
//                   }
//                   else
//                   {
//                       work1 =456;
//                       goto ENFORCE_STOP;
//                   } 
//               }
#endif
               //DA_ = 0;
               //bitset(PORTA,2);
           }
           else    // this is a address
           {
               W = SSPBUF_RX;
               I2C.I2CGettingPKG =1;
           }
           //BF = 0;      
           goto MAIN_EXIT;
       }
#ifdef I2C_INT_SUPPORT
       else // for send it is an empty buffer // for receive it is first byte
       {
           if (I2C_B1.NeedReceive) // master needs to receive data == adress was send
           {
RECV_NEXTBYTE:
               //bitset(ddebug,3);
               //if (S)
               //    bitset(ddebug,5); 
               if (P)
               {
               //    //bitset(ddebug,6); 
                   goto ENFORCE_STOP;
               }

               I2C_B1.SendACK_NAK = 0;  // ACK or NAK was not send yet
               if (ACKDT)
                   goto ENFORCE_STOP;

               //I2C_BRG = FAST_DELAY;//0x5;
               RCEN = 1; // Receive Enable
               if (--LenI2CRead == 0)
               {
                   //bitclr(ddebug,1);
                   ACKDT = 1;
               }
               goto MAIN_EXIT;
           }
           if (I2C_B1.NeedSend)
           {
               if (ACKSTAT) // ack was not receved == error!!!!
               {
                   //ddebug++;                   
                   AOutI2CQu.iQueueSize = 0;
                   LenI2CRead = 0;
#ifdef _Q_PROCESS
                   if (AllGyro.GyroDataIs)
                   {
                        //bitset(ddebug,7); 
                        AllGyro.GyroDataIs  = 0;
                   }
#endif
                   //if (I2Caddr == 0x68)
#ifdef USE_INT
                       Main.ExtInterrupt = 1;
#endif
                   //else
                   //    Main.ExtInterrupt1 = 1;

                   goto ENFORCE_STOP;
               }
               if (AOutI2CQu.iQueueSize)
               {
                   SSPBUF_TX = AOutI2CQu.Queue[AOutI2CQu.iExit];
                   if (++AOutI2CQu.iExit >= OUT_BUFFER_LEN)
                       AOutI2CQu.iExit = 0;
                   AOutI2CQu.iQueueSize--;
               }
               else // done I2C transmit
               {
                   I2C_B1.NeedSend = 0;
                   if (LenI2CRead) // will be receive from I2C after write == restart condition
                   {
                       I2C_BRG = SLOW_DELAY;//0x9; // back to speed 
                       RSEN = 1;
                       I2C_B1.NeedRestart = 1;
                       I2C_B1.I2Cread = 1;
                       I2C.I2CGettingPKG =1;
//                       goto CHECK_SSPIF_AGAIN;
                   }
                   else   // done with transmit and that it
                   {

ENFORCE_STOP:
                       I2C_BRG = SLOW_DELAY;//0x9;
                       PEN = 1;
//WAIT_FOR_SSPIF4:
//                   if (!SSPIF)               // must be another interrupt flag set after ACKEN = 1;
//                       goto WAIT_FOR_SSPIF4;

                       I2C_B1.NeedStop = 1;
                       I2C_B1.NeedRestart = 0;
                       I2C_B1.NeedReceive = 0;
                       I2C_B1.NeedSend = 0;
                       I2C_B1.NeedMaster = 0;
//CHECK_SSPIF_AGAIN:
//WAIT_FOR_SSPIF3:
//                   if (!SSPIF)               // must be another interrupt flag set after ACKEN = 1;
//                       goto WAIT_FOR_SSPIF3;
//
//
//                       if (SSPIF)               // if PEN done then SSPIF set
//                       {
//                           SSPIF = 0;
//                           goto CHECK_ANOTHER;
//                       }
                   }
               }
               goto MAIN_EXIT;
           }
       }
#endif
//CHECK_ANOTHER:
       if (S)  // this is a Start condition for I2C
       {
           //dddddebug = ddddebug;
           //ddddebug = dddebug;
           //dddebug = ddebug;
           //bitset(ddebug,0);
           I2C_B1.I2CBusBusy = 1;
           //S = 0;
           //bitset(PORTA,2);
#ifdef I2C_INT_SUPPORT
           if (BCLIF)
           {
               BCLIF = 0;
               goto MAIN_EXIT;
           }
           if (I2C_B1.NeedRestart)
           {
           //dddddebug = ddddebug;
           //ddddebug = dddebug;
           //dddebug = ddebug;
           //bitset(ddebug,1);
           //bitclr(ddebug,0);
               I2C_B1.NeedRestart = 0;
               I2C_B1.NeedReceive = 1;
               
//               if (LenI2CRead == 1)
//                   ACKDT = 1;
//               else
               goto SENTI2C_ADDR;
           }
           if (I2C_B1.NeedMaster)
           {
           //dddddebug = ddddebug;
           //ddddebug = dddebug;
           //dddebug = ddebug;
           //ddebug = 1;
           //bitset(ddebug,1);
               //I2C.I2CGettingPKG =1;  // blocking - anyway it will be some packege
               I2C_B1.NeedMaster = 0;
               if (AOutI2CQu.iQueueSize)  // in output Que there are something then it will be req Send first
                   I2C_B1.NeedSend = 1;
               else
                   I2C_B1.NeedReceive = 1;
SENTI2C_ADDR:
               I2C_BRG = FAST_DELAY;//0x5;         // twise faster
               work2 = I2Caddr<<1;   // first will be address it is shifted by 1 bit left
               if (I2C_B1.I2Cread)
                   bitset(work2,0);  // thi can be wriye
               SSPBUF_TX = work2;       // send adress byte (tranmit/receive operation - does not matter)
               ACKDT = 0;            // ACK set for all transmission ; on last byte it will be NAK
               goto MAIN_EXIT;
           }
           //work1 =30;

           goto MAIN_EXIT;
#endif
       }
       if (P)  // I2C bus is free
       {
           //dddddebug = ddddebug;
           //ddddebug = dddebug;
           //dddebug = ddebug;
           //ddebug = 0;
#ifdef I2C_INT_SUPPORT
           if (I2C_B1.NeedStop) // needs to close MASTER mode and go back to SLAVE 
           {
               I2C_B1.NeedStop = 0;
#ifndef __PIC24H__
               BCLIE = 0; // disable collision interrupt
#endif
#ifndef I2C_ONLY_MASTER
               SSPCON1 =0b00011110;
              
               //0        -WCOL: Write Collision Detect bit
               //          1 = An attempt to write the SSPBUF register failed because the SSP module is busy
               //              (must be cleared in software)
               //          0 = No collision
               // 0       -SSPOV: Receive Overflow Indicator bit
               //          1 = A byte is received while the SSPBUF register is still holding the previous byte. SSPOV is
               //              a “don’t care” in Transmit mode. SSPOV must be cleared in software in either mode.
               //          0 = No overflow
               //  1      -SSPEN: Synchronous Serial Port Enable bit
               //          1 = Enables the serial port and configures the SDA and SCL pins as serial port pins
               //          0 = Disables serial port and configures these pins as I/O port pins
               //   1     -CKP: Clock Polarity Select bit.SCK release control
               //          1 = Enable clock
               //          0 = Holds clock low (clock stretch). (Used to ensure data setup time.)
               //    1110-SSPM<3:0>: Synchronous Serial Port Mode Select bits
               //          0110 = I2C Slave mode, 7-bit address
               //          0111 = I2C Slave mode, 10-bit address
               //          1011 = I2C Firmware Controlled Master mode (Slave Idle)
               //          1110 = I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
               //          1111 = I2C Slave mode, 10-bit address with Start and Stop bit interrupts enabled
               //          1000 = I2C MASTER mode
               SSPCON2 = 0b00000000;
               SSPADD = UnitADR<<1;  // ready to get something to device
#endif
               // out qu must be cleaned
               AOutI2CQu.iQueueSize = 0;
               AOutI2CQu.iEntry = 0;
               AOutI2CQu.iExit = 0;
               I2C.NextI2CRead = 0;
               I2C_B1.I2CMasterDone = 1;
               //SSPEN = 1;
           }
#else
           I2C_B1.I2CMasterDone = 1;
#endif
           I2C_B1.I2CBusBusy = 0;
           //P = 0;
           I2C.I2CGettingPKG = 0;
           goto MAIN_EXIT;
       }
#ifdef I2C_INT_SUPPORT
       if (SSPOV) // overflow == done - done
       {
           SSPOV = 0;
           W = SSPBUF_RX;
           //work1 =2;
           goto ENFORCE_STOP;
       }
#endif


#ifdef __PIC24H__
MAIN_EXIT:;
#else
       //work1 =33;
       goto MAIN_EXIT;

#endif
    }
#ifndef __PIC24H__
 #ifdef I2C_INT_SUPPORT
   /////////////////////////////////////////////////////////////////////////////////////////////////////
   IF_BCLIF //if (BCLIF)
   {
       BCLIF = 0;
       //work1 =1;
       goto ENFORCE_STOP;
   }
 #endif
#endif

#ifdef __PIC24H__
   //////////////////////////////////////////////////////////////////////////////////////////////////////
   IF_SLAVEI2C
   {
       unsigned int work2;
       register unsigned int W;
       SSPIF = 0; // clean interrupt
       if (BF)     // receved salve mode byte - buffer is full
       {
           if (DA_) // 1 = this is byte of data  
           {
               //work2 = SSPBUF;
STORE_BYTES:
               if (AInI2CQu.iQueueSize < BUFFER_LEN)
               {
                   AInI2CQu.Queue[AInI2CQu.iEntry] = SSPBUF_RX;//work2;
                   if (++AInI2CQu.iEntry >= BUFFER_LEN)
                       AInI2CQu.iEntry = 0;
                   AInI2CQu.iQueueSize++;
               }
               else
                   W = SSPBUF_RX; // this byte is skipped to process = no space in a queue
               //DA_ = 0;
               //bitset(PORTA,2);
           }
           else    // this is a address
           {
               W = SSPBUF_RX;
               I2C.I2CGettingPKG =1;
           }
           //BF = 0;
           I2C1CONbits.SCLREL=1;
           //goto MAIN_EXIT;
       }
//CHECK_ANOTHER:
       if (S)  // this is a Start condition for I2C
           I2C_B1.I2CBusBusy = 1;
       if (P)  // I2C bus is free
       {
           I2C_B1.I2CMasterDone = 1;
           I2C_B1.I2CBusBusy = 0;
           I2C.I2CGettingPKG = 0;
       }
MAIN_EXIT:;

   }
#endif

#ifdef __PIC24H__
#ifdef USE_COM2
   ///////////////////////////////////////////////////////////////////////////////////////////////////////
   IF_RCIFCOM2 //if (RCIF)
   {

       unsigned int work1;
       unsigned int work2;
       RCIFCOM2 = 0;  // on pic24 needs to clean interrupt manualy
       if (U2STAbits.URXDA)
       {
           while(U2STAbits.URXDA)
           {
               work1 = RCSTACOM2;
               work2 = RCREGCOM2;
               if (AInQuCom2.iQueueSize < BUFFER_LEN)
               {
                   AInQuCom2.Queue[AInQuCom2.iEntry] = work2;
                   if (++AInQuCom2.iEntry >= BUFFER_LEN)
                       AInQuCom2.iEntry = 0;
                   AInQuCom2.iQueueSize++;
               }
           }
        }
   }
   /////////////////////////////////////////////////////////////////////////////////////////////////
// UART2 errors
   IF_RCERRORCOM2
   {
       unsigned int work1;
       unsigned int work2;

       work1 = RCSTACOM2;
       work2 = RCREGCOM2;
               
       if (bittest(work1,3)) //PERR)
       {
            bitclr(RCSTACOM2,3);
       }
       if (bittest(work1,2)) //FERR)
       {
            bitclr(RCSTACOM2,2);
       }
       if (bittest(work1,1)) //OERR)
       {
            bitclr(RCSTACOM2,1);
       }
   }
   ///////////////////////////////////////////////////////////////////////////////////////////////////
   IF_TXIECOM2 //if (TXIE) // expecting interrupts
   {
       unsigned int work1;
       TXIFCOM2 = 0; // on PIC24 needs to clean interrupt manualy
       if (!U2STAbits.UTXBF) // is any space in HW output queue ? (if bit == 0 then it is posible to push another byte)
       {
           if (AOutQuCom2.iQueueSize)
           {
               {
                   TXREGCOM2 = AOutQuCom2.Queue[AOutQuCom2.iExit];
                   if (++AOutQuCom2.iExit >= OUT_BUFFER_LEN)
                       AOutQuCom2.iExit = 0;
                   AOutQuCom2.iQueueSize--;
               }
           }
           else
           {
               if (TRMTCOM2)    // if nothing ina queue and transmit done - then disable interrupt for transmit
               {             // otherwise it will be endless
                    // for speed up output - first bytes already send + at the end needs to send UnitAddr
                   TXIECOM2 = 0;
               }
               else // transmit buffer has something in it (also for pic24 in a bufer there is a data)
               {
                      TXIECOM2 = 0;         // avoid reentry of interrupt
               }
           }
       }
   }
#endif // USE_COM2
#endif //__PIC24H__
   ///////////////////////////////////////////////////////////////////////////////////
   IF_RCIF //if (RCIF)
   {
#ifdef __PIC24H__
       unsigned int work1;
       unsigned int work2;

#endif

       //RCIF = 0;  // cleaned by reading RCREG
#ifdef __PIC24H__
        RCIF = 0;  // on pic24 needs to clean interrupt manualy
       if (U1STAbits.URXDA) // this bit indicat that bytes avalable in FIFO
       {
           while(U1STAbits.URXDA)
           {
#else
       if (RCIE)    // on another pics : ??????
       {
           while(RCIF) // interrupt indicate data avalable in FIFO
           {
#endif

               work1 = RCSTA;
               work2 = RCREG;
#ifdef __PIC24H__
               goto NO_RC_ERROR;
#else
               if (bittest(work1,2)) //FERR)
                   goto RC_ERROR;
               if (bittest(work1,1)) //OERR)
                   goto RC_ERROR;
               goto NO_RC_ERROR;
RC_ERROR:
               CREN = 0;
               CREN = 1; 

#endif
RC_24_ERROR:
               Main.getCMD = 0;
               I2C.RetransComI2C = 0;
               //Main.PrepI2C = 0;
               continue; // can be another bytes
NO_RC_ERROR:
               if (RetrUnit) // relay data over unit during processing commands streaming from I2C
               {
                   if (Main.prepStream)
                   {
                      
                       if (Main.prepSkip)     // was skip = clean and relay
                       {
                           Main.prepSkip = 0;
                           Main.prepZeroLen = 0;
                           goto RELAY_SYMB;
                       }
                       else if (work2 == ESC_SYMB) // do relay
                       {
                           Main.prepZeroLen = 0;
                           Main.prepSkip = 1;
RELAY_SYMB:
                           // exact copy from putchar == shit it is big!!! but it can be called recursivly!!
                           if (AOutQu.iQueueSize == 0)  // if this is a com and queue is empty then needs to directly send byte(s) 
                           {                            // on 16LH88,16F884,18F2321 = two bytes on pic24 = 4 bytes
                               // at that point Uart interrupt is disabled
                               if (_TRMT)            // indicator that tramsmit shift register is empty (on pic24 it is also mean that buffer is empty too)
                               {
                                   TXEN = 1;
                                   I2C.SendComOneByte = 0;
                                   TXREG = work2; // this will clean TXIF on 88,884 and 2321
                               }
                               else // case when something has allready send directly
                               {
#ifdef __PIC24H__
                                    TXIF = 0; // for pic24 needs to clean uart interrupt in software
                                    if (!U1STAbits.UTXBF) // on pic24 this bit is empy when at least there is one space in Tx buffer
                                        TXREG = work2;   // full up TX buffer to full capacity, also cleans TXIF
                                    else
                                        goto SEND_BYTE_TO_QU; // placing simbol into queue will also enable uart interrupt
#else
                                    if (!I2C.SendComOneByte)      // one byte was send already 
                                    {
                                        TXREG = work2;           // this will clean TXIF 
                                        I2C.SendComOneByte = 1;
                                    }
                                    else                     // two bytes was send on 88,884,2321 and up to 4 was send on pic24
                                    {
                                           goto SEND_BYTE_TO_QU; // placing simbol into queue will also enable uart interrupt
                                    }

#endif
                               }
                           }
                           else
                           {
                               if (AOutQu.iQueueSize < OUT_BUFFER_LEN)
                               {
SEND_BYTE_TO_QU:
                                   AOutQu.Queue[AOutQu.iEntry] = work2; // add bytes to a queue
                                   if (++AOutQu.iEntry >= OUT_BUFFER_LEN)
                                       AOutQu.iEntry = 0;
                                   AOutQu.iQueueSize++; // this is unar operation == it does not interfere with interrupt service decrement
                                   //if (!Main.PrepI2C)      // and allow transmit interrupt
                                   TXIE = 1;  // placed simbol will be pushed out of the queue by interrupt
                               } 
                           }
                           goto END_INPUT_COM;
                       }
                       else if (work2 == RetrUnit) // relay done
                       {
                           if (Main.prepZeroLen) // packets with 0 length does not exsists
                               goto RELAY_SYMB;
                           RetrUnit = 0;
                           Main.prepStream = 1;
#ifdef ALLOW_RELAY_TO_NEW
                           AllowMask = AllowOldMask;AllowOldMask= AllowOldMask1;AllowOldMask1= AllowOldMask2;AllowOldMask2= AllowOldMask3;AllowOldMask3= AllowOldMask4; // restore allow mask
                           AllowOldMask4 = 0xff;
#else
                           AllowMask = 0xff; // it is possible to send msg to any unit over COM
#endif
                           goto RELAY_SYMB;
                       }
#ifdef ALLOW_RELAY_TO_NEW
#endif
                       Main.prepZeroLen = 0;
                       goto RELAY_SYMB;
                   }
               }

               if (AInQu.iQueueSize < BUFFER_LEN)
               {
                   AInQu.Queue[AInQu.iEntry] = work2;
                   if (++AInQu.iEntry >= BUFFER_LEN)
                       AInQu.iEntry = 0;
#pragma updateBank 0
                   AInQu.iQueueSize++;
#pragma updateBank 1
               }
               //else
               //    W = RCREG; // this byte is skipped to process
           }
        }
END_INPUT_COM:;
               //bitclr(PORTA,2);
   }
#ifdef __PIC24H__
    /////////////////////////////////////////////////////////////////////////////////////////////
// UART errors
   IF_RCERRORCOM1
   {
       unsigned int work1;
       unsigned int work2;

       work1 = RCSTA;
       work2 = RCREG;
       //RCIF = 0;  // on pic24 needs to clean interrupt manualy
       if (bittest(work1,3)) //PERR)
       {
           bitclr(RCSTA,3);
       }
       if (bittest(work1,2)) //FERR)
       {
           bitclr(RCSTA,2);
       }
       if (bittest(work1,1)) //OERR)
       {
            bitclr(RCSTA,1);
       }
   }
#endif

   ///////////////////////////////////////////////////////////////////////////////////////////
   IF_TXIE //if (TXIE) // expecting interrupts
   {
#ifdef __PIC24H__
       unsigned int work1;
       TXIF = 0;             // for pic24 needs to clean interrupt flag inside software
       if (!U1STAbits.UTXBF) // is any space in HW output queue ? (if bit == 0 then it is posible to push another byte)
#else
       if (TXIF)        
#endif
       {
#ifdef SPEED_SEND_DATA
           if (Speed.SpeedSendLocked)
               goto SPEED_SEND;
#endif
           if (AOutQu.iQueueSize)
           {
               //if (!BlockComm) // com and I2C shared same output queue - in case of I2C nothing goes to com
               {
                   // load to TXREG will clean TXIF
                   TXREG = AOutQu.Queue[AOutQu.iExit];
                   if (++AOutQu.iExit >= OUT_BUFFER_LEN)
                       AOutQu.iExit = 0;
                   AOutQu.iQueueSize--;
               }
           }
           else
           {
SPEED_SEND:
#ifdef SPEED_SEND_DATA
               if (Speed.SpeedSend)
               {
                   Speed.SpeedSendLocked = 1;
                   if (Speed.SpeedSendUnit)
                   {
                       TXREG = UnitFrom;
DONE_WITH_SPEED:
                       Speed.SpeedSend =0;
                       Speed.SpeedSendLocked = 0;
                       goto CONTINUE_WITH_ISR;        
                   }
                   if (Speed.SpeedSendWithESC)
                   {
                       if (Speed.SpeedESCwas)
                       {
                           Speed.SpeedESCwas = 0;
                           goto SPEED_TX;
                       }
                       work1 = ptrSpeed[LenSpeed];
                       if (work1 == ESC_SYMB)
                       {
                           // escape it
ESCAPE_IT:                 TXREG = ESC_SYMB;
                           Speed.SpeedESCwas = 1;
                           goto CONTINUE_WITH_ISR;        
                       }
                       if (work1 < MIN_ADR)
                           goto SPEED_TX;
                       if (work1 > MAX_ADR)
                           goto SPEED_TX;
                       goto ESCAPE_IT;
                   }
SPEED_TX:
                   TXREG = ptrSpeed[LenSpeed];
                   if ((--LenSpeed) == 0)
                   {
                        if (UnitFrom)
                           Speed.SpeedSendUnit = 1;
						else                        
                           goto DONE_WITH_SPEED;
                   }
                   goto CONTINUE_WITH_ISR; 
              }
#endif

               if (_TRMT)    // if nothing ina queue and transmit done - then disable interrupt for transmit
               {             // otherwise it will be endless
                    // for speed up output - first bytes already send + at the end needs to send UnitAddr
                   TXIE = 0;
               }
               else // transmit buffer has something in it (also for pic24 in a bufer there is a data)
               {
#ifdef SPEED_SEND_DATA
                  if (!Speed.SpeedSend) // nothing in a output queue and speed send is done then needs to disable interrupt to
#endif
                      TXIE = 0;         // avoid reentry of interrupt
                  //I2C.SendComOneByte = 0;
               }
           }
       }
CONTINUE_WITH_ISR:;
   }
   
    /////////////////////////////////////////////////////////////////////////////////////////////
    IF_TIMER0_INT_FLG //if (TIMER0_INT_FLG) // can be transfer byte using TMR0
    {
        TIMER0_INT_FLG = 0; // clean timer0 interrupt
        //bitset(TIMER0_CONTROL_REG,5);
#ifndef __PIC24H__
        T0SE = 1;
   #ifdef _18F2321_18F25K20
       TMR0ON = 0;
   #endif
        I2C.Timer0Fired = 1;
        TIMER0_INT_ENBL = 0; // diasable timer0 interrupt
#else  // for __PIC24H__
   #ifdef TEMP_MEASURE
#ifdef _16F88
DELAY_1MKS:
#endif
       if (TEMP_Index > 0) // case when needs to send sequence with a time to TEMP sensor
       {
BEGIN_TERM_OP:           
           switch(TEMP_QUEUE[16 - TEMP_Index])
           {
           case 0xe0:  // init termomenter
               Check();
               if (TEMP_Status == 0) 
               {
                   TEMP_I_O = 0; TEMP_MEASURE = 0; TIMER0_BYTE = (TEMP_MASTER_RST-15); // 480 mks (mesaure==513)
               }
               else
               {
#define VISUAL_TEMP 1
#ifdef VISUAL_TEMP
                   TEMP_MEASURE = 1;
#endif
                   TIMER0_BYTE = (TEMP_MASTER_RST_WAIT-5); TEMP_I_O = 1;  TEMP_Index--;  // 30 mks (measure==64)
               } 
               TEMP_Status++; // 0->1 1 ->2
               break;
           case 0xf0:  // wait from termometer to respond
               Check();
               if (TEMP_MEASURE) // DS1822 responded by lowing bus
               {
                   // no respond from TEMP sensor == skip everything
TMR0_DONE_1:
                   TEMP_Status = 0;
                   TEMP_Index = 0;
                   goto TMR0_DONE;
               }
               else 
               {
                   // temperature ready wait another 480-30 mks
                   TEMP_Index--; TIMER0_BYTE = TEMP_DS1822_WAIT;
                   TEMP_Status =0;
               }
               break;
           case 0xc0: // wait for a temp sensor to finish temparature conversion
               Check();
               if (TEMP_TYPE&0x01) // was timeout
               {
                   if (TEMP_TYPE&0x02) // was request to read 1 bit yet
                   {
                        if (TEMP_TYPE&0x04) // was request
                        {
                             if (TEMP_TYPE&0x08)
                             {
                                 Check();
                                 if (TEMP_TYPE & 0x80) // temp finish temp converion == it is posible to read now
                                 {
                                     
                                     TEMP_Index--;
                                     TEMP_TYPE = 0x00;
                                     goto BEGIN_TERM_OP;
                                 }
                                 else // temp responded with 0 == need to continue pull sensor 
                                 {
                                     TEMP_TYPE = 0x01;
                                     TEMP_I_O = 1;
                                     TIMER0_BYTE = 0;
                                 }
                             }
                             else
                             {
                                 Check();
READ_SAMPLING_2:
                                 TEMP_TYPE |= 0x08;
                                 if (TEMP_MEASURE) // 1 
                                 {
                                     TEMP_TYPE |= 0x80;    
                                     TIMER0_BYTE =TEMP_MASTER_READ_WAIT_1;
                                 }
                                 else
                                     TIMER0_BYTE =TEMP_MASTER_READ_WAIT_0;
                             }
                        }
                        else // TEMP_MASTER_WRITE/READ_DELAY expired
                        {

                             TEMP_TYPE |= 0x04;
#ifdef VISUAL_TEMP
                             //TEMP_MEASURE = 1;
#endif
                             TEMP_I_O = 1;
DELAY_WRITE_DONE1MKS_2:
                             TIMER0_BYTE =(TEMP_MASTER_READ_SAMPLE_WAIT);
#ifdef _16F88
                              while(!TIMER0_INT_FLG)
                              {
                                  //Check();
                              }
                              TIMER0_INT_FLG = 0;
                              goto READ_SAMPLING_2;
#endif

                        }
                   }
                   else // was not send request to read yet
                   {
                       TEMP_TYPE |= 0x02;
                       TEMP_I_O = 0;
                       TEMP_MEASURE = 0;
#ifdef _16F88
                       
                       MASTER_MKS_DELAY;//nop();nop();nop();nop();//nop();nop();nop();nop();
#ifdef VISUAL_TEMP
                       TEMP_MEASURE = 1;
#endif
                       TEMP_I_O = 1;
                       //TEMP_I_O = 1;
                       TEMP_TYPE |= 0x04;
                       goto DELAY_WRITE_DONE1MKS_2;
#else
                       goto START_WRITE_BIT;
#endif 
                  }
               }
               else
               {
                   TEMP_TYPE = 0x01;
                   TIMER0_BYTE = 1;
                   TEMP_I_O = 1;
               }
               break;
           case 0:
               //Check();
               TEMP_I_O = 1;
               TEMP_Status = 0;
               TEMP_Index = 16;
               break;
               //goto TMR0_DONE_1;
           default:
               //Check();
               if (TEMP_Status ==0)
               {
                   
                   TEMP_byte = TEMP_QUEUE[16 - TEMP_Index];
                   if (TEMP_byte & 0x80) // operation write
                   {
                       
                       TEMP_Status = ((unsigned int)(TEMP_byte&0x0f))<<3;
START_WRITE_BYTE:       
                       TEMP_Index--;
                       TEMP_TYPE = 0x01;
START_BOTH:
                       TEMP_byte = TEMP_QUEUE[16 - TEMP_Index];
START_WRITE_BIT:
                       Check();
                       TEMP_I_O = 0;
                       TEMP_MEASURE = 0;
#ifdef _16F88
                       MASTER_MKS_DELAY;
                       if (TEMP_byte&0x01) // write 1
                       {
                           TEMP_TYPE |= 0x02;
#ifdef VISUAL_TEMP
                           TEMP_MEASURE = 1; // 1 mks
#endif
                           TEMP_I_O = 1;  // 1mks
                           goto DEALY_WRITE0_SEND;
                       }
                       else
                       {
                           TIMER0_BYTE =TEMP_MASTER_WRITE_0;
                           TEMP_TYPE |= 0x02;
                       }
                       //goto DELAY_WRITE_DONE1MKS_1;
#else
                       TIMER0_BYTE = TEMP_MASTER_WRITE_DELAY;
#endif
                   }
                   else // operation read
                   {
                       TEMP_ReadIndex = 0;
                       TEMP_Status = ((unsigned int)(TEMP_byte&0x0f))<<3;
START_READ_BYTE:
                       //TEMP_Index--;
                       TEMP_TYPE = 0x00;
#ifdef _16F88
                       TEMP_byte = TEMP_QUEUE[16 - TEMP_Index];
START_READ_BIT:
                       Check();
                       TEMP_I_O = 0;
                       TEMP_MEASURE = 0;

                       goto DELAY_READ_DONE1MKS_1;
#else
                       goto START_BOTH;
#endif
                   }
               }
               else // continue read/write
               {
                   if (TEMP_TYPE & 0x01) // write opearation
                   {
                       if (TEMP_TYPE & 0x02) // 0 or 1 was send
                       {

                           if (TEMP_TYPE & 0x04) // dealy after write slot was send 
                           {
DONE_WRITE_BYTE:
                               TEMP_byte >>=1;
                               if ((--TEMP_Status) & 0x07) // continue with bit
                               {
                                  Check();
                                  TEMP_TYPE = 0x01;
                                  goto START_WRITE_BIT;
                               }
                               else // byte done
                               {
                                   Check();
                                   if (TEMP_Status) // continye with another byte
                                       goto START_WRITE_BYTE;
                                   else // write done
                                   {
                                       TEMP_Index--;
                                       TEMP_TYPE = 0x00;
                                       goto BEGIN_TERM_OP;
                                   }
                               }
           
                           }
                           else // delay after write slot was not send yet (1 mks)
                           {
DEALY_WRITE0_SEND:
#ifdef VISUAL_TEMP
                               TEMP_MEASURE = 1;
#endif
                               TEMP_I_O = 1;
                               TEMP_TYPE |= 0x04;
                               if (TEMP_byte&0x01)
                                   TIMER0_BYTE = TEMP_MASTER_WRITE_1_AFTER; // 60 mks
                               else
#ifdef _16F88
                                   goto DONE_WRITE_BYTE;
#else
                                   TIMER0_BYTE = TEMP_MASTER_WRITE_0_AFTER; // 1 mks
#endif
                           }
                       }
                       else  // TEMP_MASTER_WRITE_DELAY expired : send 0 or 1
                       {
#ifdef _16F88                   
DELAY_WRITE_DONE1MKS_1:
#endif

                           TEMP_TYPE |= 0x02;
                           if (TEMP_byte&0x01) // write 1
                           {
#ifdef _16F88
                               goto DEALY_WRITE0_SEND;
#else
                               TIMER0_BYTE =TEMP_MASTER_WRITE_1; // 1 mks
#endif

                           }
                           else // write 0
                           {
                               //TEMP_I_O = 1;
                               TIMER0_BYTE =TEMP_MASTER_WRITE_0; // 60 mks (real 107)
                           }
                       } 
                   }
                   else // read opeartion
                   {
                       if (TEMP_TYPE & 0x02) // 0 or 1 will be read
                       {
                            if (TEMP_TYPE & 0x04) // delay after sampling was send
                            {
                                if ((--TEMP_Status) & 0x07) // this was a bit from byte
                                {
                                    Check();
                                    TEMP_TYPE = 0;
                                    goto START_READ_BIT;
                                }
                                else
                                {
                                    Check();
                                    TEMP_READ_QUEUE[TEMP_ReadIndex++]= TEMP_byte;
                                    if (TEMP_Status) // not all bytes was reseved yet
                                        goto START_READ_BYTE;
                                    else // done with read
                                    {
                                        Check();
                                        TEMP_Index--;
                                        TEMP_TYPE = 0x00;
                                        TEMP_WORD = (((unsigned long)TEMP_READ_QUEUE[1])<<8) + ((unsigned long)TEMP_READ_QUEUE[0]);
                                        if (TEMP_WORD != TEMP_SENSOR[0])
                                        {
                                            TEMP_SENSOR[0] = (((unsigned long )TEMP_READ_QUEUE[1])<<8) + ((unsigned long)TEMP_READ_QUEUE[0]);
                                            // TBD sprintf(&chTEMP_SENSOR[0],"%02d",TEMP_SENSOR[0]>>4);
                                        }
                                        Check();
#ifdef _16F88
                                        TIMER0_INT_FLG = 0;
                                        TIMER0_INT_ENBL = 0; // diasable timer0 interrupt
                                        
#else
                                        goto BEGIN_TERM_OP;
#endif
                                    }
                                }
                            }
                            else // sampling and delay after sampling
                            {
                                //Check();
READ_SAMPLING:
                                TEMP_byte>>=1;
                                TEMP_TYPE |= 0x04;
                                if (TEMP_MEASURE) // 1 
                                {
                                    TEMP_byte |= 0x80;    
                                    TIMER0_BYTE =TEMP_MASTER_READ_WAIT_1;
                                }
                                else
                                    TIMER0_BYTE =TEMP_MASTER_READ_WAIT_0;
                                
                            }
                        
                       }
                       else // TEMP_MASTER_WRITE/READ_DELAY expired
                       {
                           //Check();
DELAY_READ_DONE1MKS_1:
                          MASTER_MKS_DELAY;
#ifdef VISUAL_TEMP
                           TEMP_MEASURE = 1;
#endif
                           TEMP_I_O = 1;
                           TEMP_TYPE |= 0x02;
                           TIMER0_BYTE =TEMP_MASTER_READ_SAMPLE_WAIT;
#ifdef _16F88
                           while(!TIMER0_INT_FLG)
                           {
                               //Check();
                           }
                           TIMER0_INT_FLG = 0;
                           goto READ_SAMPLING;
#endif
                       }
                   }
               }
               break;
           }
       }
       else // case when it wait some event to signal
       {
TMR0_DONE:
            T0SE = 1;
            DataB0.Timer0Fired = 1;
            TIMER0_INT_ENBL = 0; // diasable timer0 interrupt
       }
       Check();
   #endif
       {
TMR0_DONE:
          TMR0ON = 0;
          I2C.Timer0Fired = 1;
          TIMER0_INT_ENBL = 0; // diasable timer0 interrupt
       }
#endif
    }
//    else 
//NextCheckBit:

    /////////////////////////////////////////////////////////////////////////////////////////////
    IF_TMR1IF //if (TMR1IF)  // update clock
    {
        TMR1IF = 0;
#ifdef BT_TIMER1
        if (DataB0.Timer1Meausre)
        {
            Tmr1High++; // timer count and interrupts continue
        }
        else if (DataB0.Timer1Count)
        {
            if ((++Tmr1TOHigh) == 0)
            {
                if (DataB0.Timer1DoTX) // was a request to TX data on that frquency
                {
                    PORT_AMPL.BT_TX = 1;
                    bitset(PORT_BT,Tx_CE);
                    //DataB0.Timer1Count = 0; // switch off req round robin
                    //DataB0.Timer1Meausre = 1; // and set timer measure
                    //TMR1 = 0;
                    //Tmr1High = 0;
                    //goto SWITCH_FQ;
                    DataB0.Timer1DoTX = 0;
                }
                //else
                //{
                    //TIMER1 = Tmr1LoadLow;  
                    TMR1H = (unsigned char)(Tmr1LoadLow>>8);
                    TMR1L = (unsigned char)(Tmr1LoadLow&0xff);
                    //DataB0.Timer1Inturrupt = 1; // and relaod timer
                    Tmr1TOHigh = Tmr1LoadHigh;
//#define SHOW_RX
                    if (++FqTXCount>=3)
                    {
                       FqTXCount = 0;
                       FqTX = Freq1;
#ifdef SHOW_RX_TX
   #ifdef SHOW_RX
   #else
                       bitset(PORTA,7);
   #endif
#endif

                    }
                    else
                    {
#ifdef SHOW_RX_TX
   #ifdef SHOW_RX
   #else
                       bitclr(PORTA,7);
   #endif
#endif

                        if (FqTXCount == 1)
                            FqTX = Freq2;
                        else
                            FqTX = Freq3;
                    }
                //}
            }
        }
        
#else // BT timer1
        if (++TMR130 == TIMER1_ADJ0)
        {
#ifdef __PIC24H__
            TMR2 += TIMER1_ADJUST;//46272; // needs to be adjusted !!!
            if (TMR2 < TIMER1_ADJUST)//46272)
                goto TMR2_COUNT_DONE;
#else

RE_READ_TMR1:
            work2 = TMR1H;
            work1 = TMR1L;
//          point of time
            if (work2 != TMR1H)    // 4 tick
                goto RE_READ_TMR1;// 
            if (work1 > 149)        // 4 tick
            {
                work1 += 0x6b;        
                work2 += 0x7b;        
                nop();
                nop();              // 8 tick
            }
            else
            {
                work1 += 0x6b;    
                work2 += 0x8f;    // 8 tick
                
            }
            TMR1L = 0;            // 1 tick
            TMR1H = work2;        // 2 tick
            TMR1L = work1;        // 2 tick
            // error will be in 1/30 TMR30 counter
            // needs to make sure from 0-29 is 1130 ticks plus on each TMR130 value
            //   on TMR130 == 30 it is adjust to next sec proper set
#endif
        }
        else if (TMR130 >TIMER1_ADJ0)
        {
TMR2_COUNT_DONE:
            TMR130 = 0;
            if (++TMR1SEC > 59)
            {
                TMR1SEC=0;
                if (++TMR1MIN > 59)
                {
                    TMR1MIN = 0;
                    if (++TMR1HOUR > 23)
                    {
                         TMR1HOUR = 0;
                         if (TMR1YEAR & 0x3) // regilar year
                         {
                             if (++TMR1DAY > 365)
                             {
                                 TMR1DAY = 0;
                                 TMR1YEAR++;
                             }
                         }
                         else // leap year
                         {
                             if (++TMR1DAY > 366)
                             {
                                 TMR1DAY = 0;
                                 TMR1YEAR++;
                             } 
                         }
#ifdef USE_LCD
                         sprintf(&LCDDD[0],"%02d",TMR1DAY);
#endif
                    }
#ifdef USE_LCD
                    sprintf(&LCDHH[0],"%02d",TMR1HOUR);
#endif
                }
#ifdef USE_LCD
                sprintf(&LCDMM[0],"%02d",TMR1MIN);
#endif
            }
#ifdef USE_LCD
            sprintf(&LCDSS[0],"%02d",TMR1SEC);
#endif
        }
#endif
    }
#ifdef BT_TIMER3
    IF_TMR3IF // RX timer
    {
        TMR3IF = 0;
        

        if (DataB0.Timer3Meausre)
        {
            Tmr3High++; // timer count and interrupts continue
        }
        else if (DataB0.Timer3Count)
        {
            if ((++Tmr3TOHigh) == 0)
            {
                //TIMER3 = Tmr3LoadLow;  
                TMR3H = (unsigned char)(Tmr3LoadLow>>8);
                TMR3L = (unsigned char)(Tmr3LoadLow&0xff);
                Tmr3TOHigh = Tmr3LoadHigh;
                Tmr3LoadLow = Tmr3LoadLowCopy;
                if (SkipPtr)
                {
                   SkipPtr--;
                }
                else
                {
                    DataB0.Timer3Inturrupt = 1;
                    if (++FqRXCount>=3)
                    {
                        FqRXCount = 0;
                        FqRX = Freq1;
                        if (OutSyncCounter)
                        {
                             // this will produce request to switch off Round-Robin to one FQ listening
                            if (DataB0.AlowSwitchFq1ToFq3)
                               if (--OutSyncCounter == 0)
                               {
                                  DataB0.Timer3OutSyncRQ = 1;
                               }
                        }
#ifdef DEBUG_LED
                        bitclr(PORTA,7);
   #ifdef DEBUG_LED_CALL_LUNA
                        if (ATCMD & MODE_CONNECT)
                        {
                            if (ESCCount == 0)
                            {
                                if (!BTFlags.BTNeedsTX)
                                {
                                    if (AInQu.iQueueSize == 0)
                                    {
                                        AInQu.Queue[AInQu.iEntry] = '?';
                                        if (++AInQu.iEntry >= BUFFER_LEN)
                                            AInQu.iEntry = 0;
                                        AInQu.iQueueSize++;
                                    }
                                }
                            }
                        }
   #endif
#endif

                    }
                    else
                    {
                        if (FqRXCount == 1)
                           FqRX = Freq2;
                        else
                           FqRX = Freq3;
                    }
                }
            }
        }
        
    }
#endif
#ifdef EXT_INT
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    IF_INT0_FLG //if (INT0_FLG)
    {
        INT0_FLG = 0;
        if (INT0_ENBL)
        {
            Main.ExtInterrupt = 1;
            //Main.ExtFirst =1;
            
#ifdef BT_TX
            // communication BT - on interrupt needs goto standby state
            // may be for RX it is owerkill but for TX it is definetly == in TX it should not stay longer
            // TBD: also may be need to switch off transmitter or receiver
            //BTCE_low();  // Chip Enable Activates RX or TX mode (now disable)
            
#ifdef BT_TIMER3
            if (BTType & 0x01) // it was RX operation
            {
                if (DataB0.Timer3SwitchRX)
                    bitclr(PORT_BT,Tx_CE);	// Chip Enable Activates RX or TX mode (now standby)

                if (DataB0.Timer3DoneMeasure) // receive set
                {
                    SkipPtr++; // set of next frquency will be in CallBackMain
                    AdjustTimer3 = TIMER3;
                    DataB0.Timer3Ready2Sync = 1;
                }
                else // needs to monitor FQ1 and FQ2 receive time
                {
                    if (RXreceiveFQ == 0) // it is receive over Fq1 == need to start timer to record time btw Fq1 and FQ2
                    {
                        DataB0.Timer3Meausre = 1;
                        TMR3H = 0;
                        TMR3L = 0;
                        TMR3IF = 0;
                        TMR3IE = 1;
                        Tmr3High  = 0;
                        T3CON = 0b10000001;
                    }
                    else if (RXreceiveFQ == 1) // it was receive over Fq2
                    {
                        if (DataB0.Timer3Meausre) // timer for a measure was started ??
                        {
                            TMR3ON = 0;
                            Tmr3LoadLowCopy =0xFFFF - TIMER3;      // timer1 interupt reload values 
                            Tmr3LoadLowCopy += 52;
                            Tmr3LoadLow = Tmr3LoadLowCopy - MEDIAN_TIME;
                            TMR3H = (Tmr3LoadLow>>8);
                            TMR3L = (unsigned char)(Tmr3LoadLow&0xFF);
                            //TMR3L = 0;//xff;
                            TMR3ON = 1; // continue run
                            Tmr3TOHigh = Tmr3LoadHigh = 0xffff - Tmr3High;
                            DataB0.Timer3Meausre = 0;
                            DataB0.Timer3Count = 1;
                            DataB0.Timer3Inturrupt = 0;
                            //SkipPtr =1;
                            DataB0.Timer3OutSync = 0;
                        }
                    }
                }
                if (!DataB0.Timer3OutSync)
                    OutSyncCounter = 125; // 2.5 sec no packets == switch for out of sync
            }
            else
                bitclr(PORT_BT,Tx_CE);	// Chip Enable Activates RX or TX mode (now standby)
#endif

#endif
        }
    }
 #ifdef _18F2321_18F25K20 
 //#define USE_INT1 1
 #endif
 #ifdef __PIC24H__
 #define USE_INT1 1
 #endif
 #ifdef USE_INT1
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    IF_INT1IF //if (INT1IF)
    {
        INT1IF = 0;
        if (INT1IE)
        {
            Main.ExtInterrupt1 = 1;
            //Main.ExtFirst =0;
        }
    }
  #ifdef __PIC24H__
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    IF_INT2IF //if (INT1IF)
    {
        INT2IF = 0;
        if (INT2IE)
        {
            Main.ExtInterrupt2 = 1;
            //Main.ExtFirst =0;
        }
    }
  #endif // only for PIC24
 #endif // USE_INT1
#endif  // EXT_INT
//    else 
//Next2CheckBit:
//    if (RBIF)
//    {
////#pragma updateBank 0
//    }
#ifndef __PIC24H__
ExitIntr:
/*
    if (TimerB1.SetSleep)
    {
        TimerB1.SetSleep = 0;
        GIE = 0;
    }
*/
MAIN_EXIT:

 #ifdef __18CXX
 #else
    
    FSR_REGISTER = sv_FSR;       // restore FSR if saved
    int_restore_registers // W, STATUS (and PCLATH)
 #endif
}
#endif // not __PIC24H__
//#define ONEBIT_TMR0_LEN 0x10
// temp vars will be on bank 1 together with I2C queue
#pragma rambank RAM_BANK_1
////////////////////////////////////BANK 1////////////////////////////////////
unsigned char Monitor(unsigned char bWork, unsigned char CheckUnit)
{
    if (Main.prepSkip)
    {
        Main.prepZeroLen = 0;
        Main.prepSkip = 0;
    }
    else if (bWork == ESC_SYMB) // it is ESC char ??
    {
        Main.prepSkip = 1;
        Main.prepZeroLen = 0;
    }
    else if (bWork == CheckUnit) // 
    {
        if (Main.prepZeroLen) // packets with zero length does not exsists
            return 0;   
        Main.prepCmd = 0;
        Main.prepStream = 1;
#ifdef ALLOW_RELAY_TO_NEW
        AllowMask = AllowOldMask;AllowOldMask= AllowOldMask1;AllowOldMask1= AllowOldMask2;AllowOldMask2= AllowOldMask3;AllowOldMask3= AllowOldMask4; // restore allow mask
        AllowOldMask4 = 0xff;
#else
        AllowMask = 0xff;
#endif
        return 1;
    }
    else
        Main.prepZeroLen = 0;
    return 0;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//      END COPY1
/////////////////////////////////////////////////////////////////

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
//#include "commc6.h"
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// begin COPY 6
///////////////////////////////////////////////////////////////////////   

        AInQu.iEntry = 0;
        AInQu.iExit = 0;
		AInQu.iQueueSize = 0;

		AOutQu.iEntry = 0;
        AOutQu.iExit = 0;
		AOutQu.iQueueSize = 0;

        AInI2CQu.iEntry = 0;
        AInI2CQu.iExit = 0;
		AInI2CQu.iQueueSize = 0;

		AOutI2CQu.iEntry = 0;
        AOutI2CQu.iExit = 0;
		AOutI2CQu.iQueueSize = 0;
        //TimerB1=0;
#ifdef BT_TIMER1
#else
		TimerB1.SetSleep = 0;
        TimerB1.SetSyncTime = 0;
#endif

        //Main= 0;
        Main.getCMD = 0;
        Main.ESCNextByte = 0;
        Main.PrepI2C = 0;
        Main.DoneWithCMD = 1;
        RetransmitLen = 0;
        //Main.SendWithEsc = 0;
        //Main.CommLoopOK = 0;

        SSPADD = UnitADR<<1;
        I2C.LockToI2C = 0;
        I2C.WaitQuToEmp = 0;
        I2C.SetI2CYesNo = 0;
        I2C.EchoWhenI2C = 1;

        I2C_B1.I2CBusBusy = 0;
        //BlockComm = 0;

        I2C.Timer0Fired = 0;
        I2C.LastWasUnitAddr = 0;
#ifdef SPEED_SEND_DATA
        Speed.SpeedSend = 0;
        Speed.SpeedSendLocked = 0;
        Speed.SpeedSendUnit = 0;
        Speed.SpeedSendWithESC = 0;
        Speed.SpeedESCwas = 0;

#endif
#ifdef I2C_INT_SUPPORT
		I2C_B1.NeedMaster = 0;
        I2C_B1.NeedRestart = 0;
        I2C_B1.NeedStop = 0;
        I2C_B1.NeedReceive = 0;
        I2C_B1.NeedSend = 0;
        //NeedReciveACK = 0;


#endif
        I2C_B1.I2CMasterDone = 1;
        RetrUnit = 0;
        AllowMask = 0xff;
#ifdef ALLOW_RELAY_TO_NEW
        AllowOldMask1 = 0xff;AllowOldMask2=0xff;AllowOldMask3=0xff;AllowOldMask4=0xff; AllowOldMask =  0xff;
#endif
        UnitMask1 = 0xff;
        UnitMask2 = 0;
        Main.prepStream = 1;
        Main.prepCmd = 0;
        Main.prepSkip = 0;
        Main.prepZeroLen = 0;
        UnitFrom = 0;
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// end COPY 6


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
    //bitset(PORTA,3);
    //bitset(SSPCON,4);  // set clock high;
//#include "commc7.h"
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// begin COPY 7
///////////////////////////////////////////////////////////////////////   
// STREAM two types:
//    STREAM  ...=> processed inside a ProcessCMD in a area after "if (!getCMD)"
//    COMMANDS...=> processied inside a ProcessCMD in area "if (getCMD)" after #include "commc3.h"
//    STREAM or COMMANDS can be monitored in an area befor "if (!getCMD)"
//  COMMANDS...:= <OneCMD> || <LongCMD> on each command unit does action
//                                 action on ManyByte command depends
//                                 escape char is #:=> ##==#;#N==N(N=0-9)
// on <LongCMD> needs to clean DoneWithCMD = 0 and at the end of the command
// needs to set DoneWithCMD = 1 
// 
// stream from com:
// STREAM<Unit><COMMANDS><Unit>SSTREAM   or <unit><unit><COMMANDS><unit>
// STREAM                                 -> processed as !getCMD
//       <Unit>                           -> getCMD = 1;
//              COMMANDS                  -> processed as getCMD
//                       <Unit>           -> getCMD = 0;
//                             SSTREAM    -> processed as !getCMD
// stream from I2C: 
//    received data as slave device or receve desponce from different I2C device from as a master read request 
//    processed as a COMMANDS 
//    I2C_STREAM... == COMMANDS...
// 
//   
//  
//  receved data as responce on or just some data from inside unit
//    
// 
// b) retransmit to I2C device was set: UnitFrom=ToI2CAddr SendCMD=CMD RetransI2CCom = 1 RetransI2CComSet = 0
//    this can be done by command: =xcI where x=ToI2CAddr c=CMD  I=set retransmit
//    and 
// STREAM<Unit>=xcIstream<Unit>STREAM<unit>=xcIstram<unit>
//                 stream -> to I2C        stram-> to I2C
// 
// c)
// c) unit internaly reads something from I2C and 
//     I2Cstream-> process internaly

// stream from I2C:
// a) 
// <COMMAND>
//  CMD:= <OneByte> || <ManyBytes> on each command unit does action
//                                 action on ManyByte command is actualy on last byte

    while(1)
    {
        if (CallBkMain() == 0) // 0 = do continue; 1 = process queues
            continue;
        if (AInI2CQu.iQueueSize) // if something comes from I2C (slave received or some I2c device responded on read command)
        {
            if (RetransmitLen) // from I2C comes command =<LEN> - needs to retransmit everything to previously set device
            {                  //                          (set done by =5CC)
                if (!I2C.RetransComI2CSet)
                {
                    I2C.RetransComI2CSet = 1;
                    if (UnitFrom)
                    {
                        putch(UnitFrom);
                        if (SendCMD)
                            putch(SendCMD);
                    }
                }
REPEAT_OP1:                
                putchWithESC(getchI2C()); // if out queue does not has empty space putchWithESC will wait
                if (--RetransmitLen ==0)
                {
                    if (UnitFrom)
                        putch(UnitFrom);
                    Main.DoneWithCMD = 1; // long command done // this will unlock switching process from I2C to com
                    continue;
                }
                if (AInI2CQu.iQueueSize)
                    goto REPEAT_OP1;
                continue;
            }
            if (CallBkI2C())// 0 = do not process byte; 1 = process;
            {
                bitclr(bWork,0);
                if (Main.getCMD)
                    bitset(bWork,0);
                Main.getCMD = 1;
                ProcessCMD(getchI2C());
                Main.getCMD = 0;
                if (bittest(bWork,0))
                    Main.getCMD = 1;

            }
        }
        if (AInQu.iQueueSize)      // in comm queue bytes
        {
            //if (RetransmitLen)
            //{
            //   // TBD
            //    continue;
            //}
            if (CallBkComm()) // 0 = do not process byte; 1 = process;
            {
                 // place where has to be checked realy message mode : if CallBkComm desided to process data then needs to monitor
                 // input message for not related to unit device
                 bWork = AInQu.Queue[AInQu.iExit]; // next char
                 if (Main.prepStream)
                 {
                     if (Main.prepSkip)
                         Main.prepSkip = 0;
                     else if (bWork == ESC_SYMB) // it is ESC char ??
                         Main.prepSkip = 1;
                     else if (bWork <= MAX_ADR)
                     {
                          if (bWork >= MIN_ADR) // msg to relay
                          {
#ifdef ALLOW_RELAY_TO_NEW
                              AllowOldMask4=AllowOldMask3;AllowOldMask3=AllowOldMask2;AllowOldMask2=AllowOldMask1;AllowOldMask1=AllowOldMask;AllowOldMask=AllowMask; // put mask into stack
                              AllowMask = 0;
                              if (bWork > MY_UNIT)
                              {
                                  iWork = bWork - (MY_UNIT+1);
                                  while(iWork)
                                  { 
                                      iWork--;
                                      AllowMask <<=1;AllowMask|=UNIT_MASK;        
                                  }
                              }
                              else if (bWork < MY_UNIT)
                              {
                                  iWork = (MY_UNIT-1);// - bWork;
                                  while(iWork)
                                  { 
                                      iWork--;
                                      AllowMask <<=1;AllowMask|=0x01;        
                                  }
                                  AllowMask |= UNIT_MASK_H;
                              }
                              else // unit matches
                              {
                                  Main.prepCmd = 1;
                                  Main.prepStream = 0;
                                  Main.prepZeroLen = 1;
                                  if (UnitFrom)              // if reply's unit was set
                                      AllowMask = UnitMask1; // then allow to send in CMD mode to exact unit
                                                             // otherwise unit can not initiate any transfer before endin processing CMD
                                  goto PROCESS_IN_CMD;
                              }
                              AllowMask &= AllowOldMask;
#else
                              if (bWork == MY_UNIT) // unit matches
                              {
                                  Main.prepCmd = 1;
                                  Main.prepStream = 0;
                                  Main.prepZeroLen = 1;
                                  AllowMask = 0xff;
                                  goto PROCESS_IN_CMD;
                              }
                              AllowMask =  0;
#endif
                              // now needs to stream everything exsisting from input comm queue to output comm queue
                              putch(bWork);
                              getch();
                              Main.prepZeroLen = 1;
                              RCIE = 0; // disable com1 interrupts
                              RetrUnit = bWork;
                              while(AInQu.iQueueSize)
                              {
                                  bWork = getch();
                                  putch(bWork);
                                  if (Monitor(bWork,RetrUnit)) // search for end of packet
                                  {
                                      RetrUnit = 0;
                                      break;
                                  }
                              }
                              RCIE = 1; // enable com1 interrupts
                              goto NO_PROCESS_IN_CMD; // give a chance to process char on a next loop
                          }
                     }
                 }
                 else // Main.prepCmd == monitor message processed inside ProcessCMD == search for end of packet
                 {
                     Monitor(bWork,MY_UNIT);
                 }
PROCESS_IN_CMD:

                 ProcessCMD(getch());
NO_PROCESS_IN_CMD:;
            }
        }
        else  // nothing in both queue can sleep till interrupt
        {
        }
    }
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// end COPY 7
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////

} // at the end will be Sleep which then continue to main


#define SPBRG_9600 51
#define SPBRG_19200 25

#define SPBRG_19200_8MHZ 25
#define SPBRG_19200_16MHZ 51
#define SPBRG_19200_32MHZ 103
#define SPBRG_19200_64MHZ 207

#define SPBRG_38400_8MHZ 13
#define SPBRG_38400_16MHZ 25
#define SPBRG_38400_32MHZ 51
#define SPBRG_38400_64MHZ 103

#define SPBRG_38400 12

#define SPBRG_57600 8
#define SPBRG_57600_16MHZ 16
#define SPBRG_57600_32MHZ 34
#define SPBRG_57600_64MHZ 68
#define SPBRG_57600_40MIPS 172

#define SPBRG_115200_16MHZ 8
#define SPBRG_115200_32MHZ 16
#define SPBRG_115200_64MHZ 34
#define SPBRG_115200_40MIPS 85
#define SPBRG_SPEED     SPBRG_57600_40MIPS
#define SPBRG_SPEEDCOM2 SPBRG_57600_40MIPS


//#include "commc2.h"
/////////////////////////////////////////////////////////////////
//      Begin COPY 2
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
#ifdef SSPORT
void SendSSByte(unsigned char bByte);
unsigned char GetSSByte(void);
void SendSSByteFAST(unsigned char bByte); // for a values <= 3
#endif


void enable_uart(void);
void enable_I2C(void);
void EnableTMR1(void);
void putch(unsigned char simbol)
{
    if (AOutQu.iQueueSize == 0)  // if this is a com and queue is empty then needs to directly send byte(s) 
    {                            // on 16LH88,16F884,18F2321 = two bytes on pic24 = 4 bytes
        // at that point Uart interrupt is disabled
#ifdef __PIC24H__
        if (!U1STAbits.UTXBF) // on pic24 this bit is empy when at least there is one space in Tx buffer
        {
            TXEN = 1; // just in case ENABLE transmission
            //TXIF = 0; // for pic24 needs to clean uart interrupt in software
            TXREG = simbol;   // full up TX buffer to full capacity, also cleans TXIF
        }
        else
        {
            goto SEND_BYTE_TO_QU; // placing simbol into queue will also enable uart interrupt
        }
#else
        if (_TRMT)            // indicator that tramsmit shift register is empty (on pic24 it is also mean that buffer is empty too)
        {
            TXEN = 1;
            I2C.SendComOneByte = 0;
            TXREG = simbol; // this will clean TXIF on 88,884 and 2321
        }
        else // case when something has allready send directly
        {

            if (!I2C.SendComOneByte)      // one byte was send already 
            {
                TXREG = simbol;           // this will clean TXIF 
                I2C.SendComOneByte = 1;
            }
            else                     // two bytes was send on 88,884,2321 and up to 4 was send on pic24
                goto SEND_BYTE_TO_QU; // placing simbol into queue will also enable uart interrupt
        }
#endif
    }
    else
    {
        if (AOutQu.iQueueSize < OUT_BUFFER_LEN)
        {
SEND_BYTE_TO_QU:
            AOutQu.Queue[AOutQu.iEntry] = simbol; // add bytes to a queue
            if (++AOutQu.iEntry >= OUT_BUFFER_LEN)
                AOutQu.iEntry = 0;
            AOutQu.iQueueSize++; // this is unar operation == it does not interfere with interrupt service decrement
            //if (!Main.PrepI2C)      // and allow transmit interrupt
            TXIE = 1;  // placed simol will be pushed out of the queue by interrupt
        } 
    }
}

#ifdef USE_COM2
#ifdef __PIC24H__
void putchCom2(unsigned char simbol)
{
    if (AOutQuCom2.iQueueSize == 0)  // if this is a com and queue is empty then needs to directly send byte(s) 
    {                            // on 16LH88,16F884,18F2321 = two bytes on pic24 = 4 bytes
        // at that point Uart interrupt is disabled
        if (!U2STAbits.UTXBF) // on pic24 this bit is empy when at least there is one space in Tx buffer
        {
            TXENCOM2 = 1; // just in case ENABLE transmission
            //TXIF = 0; // for pic24 needs to clean uart interrupt in software
            TXREGCOM2 = simbol;   // full up TX buffer to full capacity, also cleans TXIF
        }
        else
        {
            goto SEND_BYTE_TO_QU; // placing simbol into queue will also enable uart interrupt
        }
    }
    else
    {
        if (AOutQuCom2.iQueueSize < OUT_BUFFER_LEN)
        {
SEND_BYTE_TO_QU:
            AOutQuCom2.Queue[AOutQuCom2.iEntry] = simbol; // add bytes to a queue
            if (++AOutQuCom2.iEntry >= OUT_BUFFER_LEN)
                AOutQuCom2.iEntry = 0;
            AOutQuCom2.iQueueSize++; // this is unar operation == it does not interfere with interrupt service decrement
            //if (!Main.PrepI2C)      // and allow transmit interrupt
            TXIECOM2 = 1;  // placed simbol will be pushed out of the queue by interrupt
        } 
    }
}

void putchCom2WithESC(unsigned char simbol)
{
    if (Main.SendCom2WithEsc)
    {
WAIT_SPACE_Q:
        if (AOutQuCom2.iQueueSize >= (OUT_BUFFER_LEN-3)) // is enought space to output ??
            goto WAIT_SPACE_Q;
        if (simbol == ESC_SYMB)
           goto PUT_ESC;

        if (simbol >= MIN_ADR)
        {
            if (simbol <= MAX_ADR)
            {
PUT_ESC:
                putchCom2(ESC_SYMB);
            }
        }
    }
    putchCom2(simbol);
}
void PutsCom2(const char * s)
{
	while(*s)
	{
        if (Main.SendCom2WithEsc)
        {
            if (*s >= MIN_ADR)
                if (*s <= MAX_ADR)
                    putchCom2(ESC_SYMB);
        } 
		putchCom2(*s);
		s++;
	}
}


#endif // __PIC24H_
#endif // USE_COM2

void putchI2C(unsigned char simbol)
{
    AOutI2CQu.Queue[AOutI2CQu.iEntry] = simbol; // add bytes to a queue
    if (++AOutI2CQu.iEntry >= OUT_BUFFER_LEN)
        AOutI2CQu.iEntry = 0;
    AOutI2CQu.iQueueSize++;
}

void putchWithESC(unsigned char simbol)
{
    if (Main.SendWithEsc)
    {
WAIT_SPACE_Q:
        if (AOutQu.iQueueSize >= (OUT_BUFFER_LEN-3)) // is enought space to output ??
            goto WAIT_SPACE_Q;
        if (simbol == ESC_SYMB)
           goto PUT_ESC;

        if (simbol >= MIN_ADR)
        {
            if (simbol <= MAX_ADR)
            {
PUT_ESC:
                putch(ESC_SYMB);
            }
        }
    }
    putch(simbol);
}
void putmsg(const char *s, unsigned char len)
{
    while(len)
    {
        putch(*s);
		s++;
        len--;
    }
}
void Puts(const char * s)
{
	while(*s)
	{
        if (Main.SendWithEsc)
        {
            if (*s >= MIN_ADR)
                if (*s <= MAX_ADR)
                    putch(ESC_SYMB);
        } 
		putch(*s);
		s++;
	}
}
#pragma rambank RAM_BANK_0
////////////////////////////////////////////BANK 0//////////////////////////

unsigned char getch(void)
{
    unsigned char bRet = AInQu.Queue[AInQu.iExit];
    if (++AInQu.iExit >= BUFFER_LEN)
        AInQu.iExit = 0;
#pragma updateBank 0
    AInQu.iQueueSize --;
    return bRet;
}
#pragma updateBank 1

/*
unsigned char getch(void)
{
    unsigned char bRet = InQu[iPtr2InQu];
    if (++iPtr2InQu >= BUFFER_LEN)
        iPtr2InQu = 0;
    iInQuSize --;
    return bRet;
}*/
unsigned char getchI2C(void)
{
    unsigned char bRet = AInI2CQu.Queue[AInI2CQu.iExit];
    if (++AInI2CQu.iExit >= BUFFER_LEN)
        AInI2CQu.iExit = 0;
    AInI2CQu.iQueueSize --;
    return bRet;
}
void InsertI2C(unsigned char bWork)
{
    if (AInI2CQu.iQueueSize < BUFFER_LEN)
    {
        AInI2CQu.Queue[AInI2CQu.iEntry] = bWork;
        if (++AInI2CQu.iEntry >= BUFFER_LEN)
            AInI2CQu.iEntry = 0;
        AInI2CQu.iQueueSize++;
    }
//    else
//    {
//bWork = 0;
//    } 
}
#pragma rambank RAM_BANK_1
//////////////////////////////////////////////BANK 1///////////////////////////


unsigned char eeprom_read(unsigned char addr);
void eeprom_write(unsigned char addr, unsigned char value);
/////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////
unsigned char InitI2cMaster(void)
{
#ifdef I2C_INT_SUPPORT
WAIT_STOP:
//      if (!P)
//      {
//         if (!S)
//             goto FIRST_I2C;
//         goto WAIT_STOP;
//      }
FIRST_I2C:
      //bitset(I2CTRIS,I2C_SDA);  // SDA=1
      //bitset(I2CTRIS,I2C_SCL);  // SCL=1
      I2C_B1.I2CMasterDone = 0;
      
#ifndef I2C_ONLY_MASTER
   SSPCON2 =0b00000000;
   SSPCON1 &=0b11111000; // for PIC18F2321 only set master 1000
#endif
      //SSPCON1 =0b00011000;
      // I2C_BRG == SSPADD on 88, 884, 2321
      I2C_BRG = SLOW_DELAY;//0x9;//12; // 400 kHz formula:
                     // 400000 = 32000000/(4*(SSPADD +1))
                     // 4*(SSPADD + 1) = 32000000/400000 = 80
                     // SSPADD + 1 = 80/4 = 20
                     // SSPADD = 20-1 = 19 = 0x13
                     // or for 100kHz
                     // SSPADD + 1 = 32000000/100000/4 = 80
                     // SSPADD = 79 = 0x4F
      //SMP = 0; // 1 for < 400kHz 0 == 400kHz
#ifndef __PIC24H__
      CKE = 0;
#endif
      WCOL = 0;
      SSPOV = 0;
      BCLIF = 0;
      I2C_B1.NeedMaster = 1;

      //SSPEN = 1; // enable I2C
      //BCLIE = 1; // enable collision interrupt
      SEN = 1;   // Start condition Enable      
      return 1;
#else

I2C_IWAIT_READ:
     if (I2C.NextI2CRead) // this will be restart condition
         goto FROM_RESTART;
     if (I2C_B1.I2CBusBusy) // needs to wait when I2C will be not busy
         goto I2C_IWAIT_READ;
      I2C_B1.I2CMasterDone = 0;
FROM_RESTART:
//#pragma updateBank 0
     bitclr(I2CTRIS,I2C_SDA);  // SDA=0 SCL=1
     DELAY_START_I2C;
     bitclr(I2CTRIS,I2C_SCL);  // SDA=0 SCL=0
     DELAY_1_I2C;      // TBD - different delay for different units for arbitration
     bitset(I2CTRIS,I2C_SDA);  // SDA up for a little bit to check that bus not busy
//#pragma updateBank 1
     DELAY_1_I2C;
     if (bittest(I2CPORT,I2C_SDA))  // that we succesfully garbed I2C bus
     {
         bitclr(I2CTRIS,I2C_SDA);  // bus is ours
         return 0;
     }
     bitset(I2CTRIS,I2C_SCL);  // release I2C bus immiduatly : SDA=1 SCL=1
     return 1;
#endif
}
/////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////
#ifdef I2C_INT_SUPPORT
#else
unsigned char sendI2C()
{
    unsigned char bWork;
    unsigned char bCount;
    bWork = I2Caddr<<1;   // first will be address it is shifted by 1 bit left
    if (I2C_B1.I2Cread)
        bitset(bWork,0);  // thi can be wriye
    FSR_REGISTER = &AOutI2CQu.Queue[AOutI2CQu.iExit]; // the goes bytes of data
    goto SEND_I_ADDR;
    
    while(AOutI2CQu.iQueueSize)
    {
        AOutI2CQu.iQueueSize--;
        bWork = PTR_FSR;
        FSR_REGISTER++;

        //bWork = AOutI2CQu.Queue[AOutI2CQu.iExit]; // the goes bytes of data
            
        //if (++AOutI2CQu.iExit >= OUT_BUFFER_LEN)
        //    AOutI2CQu.iExit = 0;
//#pragma updateBank 0
        //AOutI2CQu.iQueueSize--;
//#pragma updateBank 1

SEND_I_ADDR:
///////////////////////////////////////////////////////////////////////////
//                                            optimozation part of a send
//////////////////////////////////////////////////////////////////////////
#ifdef _OPTIMIZED_
//#pragma updateBank 0
        bCount = 9;
        Carry = 0;
        goto SEND_I_FIRST_BIT;

        // on entry SDA = 0 SCL = 0
        while (--bCount)
        {
            //DELAY_1_I2C; // this dealy is not nessesary - loop already does delay
            bitset(I2CTRIS,I2C_SCL);     // SDA = X SCL = 1 set SCL high
            DELAY_1_I2C;
            bitclr(I2CTRIS,I2C_SCL);     // SDA = X SCL = 0  set SCL low
SEND_I_FIRST_BIT:
#ifdef      _18F2321_18F25K20
            #asm
              RLCF bWork,1,1
            #endasm
#else
            RLF(bWork,1);
#endif
            if (Carry)
                bitset(I2CTRIS,I2C_SDA); //SDA = 1 SCL = 0 set SDA High
            else
                bitclr(I2CTRIS,I2C_SDA); //SDA = 0 SCL = 0 set SDA Low
        }
        // carry rotate full circle and returned back == 0
        // on exit SDA = 0 SCL = 0

        // bWork now is the same as at the begining
        // wait for ACK
        //
        //bitclr(I2CTRIS,I2C_SDA);         //SDA = 0 SCL = 0 set SDA low - this was done on last loop
        //DELAY_1_I2C;
        // check:
        //bitset(I2CTRIS,I2C_SDA);        //  now SDA = 1 (looks like high) with SCL = 0 prepear to read input value from receiver

        bitset(I2CTRIS,I2C_SCL);        // SDA = 0 SCL = 1 set SCL high to read ACK (or NACK)
        bitset(I2CTRIS,I2C_SDA);        //  now SDA = 1 (looks like high) with SCL = 1 prepear to read input value from receiver
        
#pragma updateBank 1
        
        if (bittest(I2CPORT,I2C_SDA))   //     if bit set then it is NAK - TBD then transfered byte was not acsepted
            Carry = 1;
        bitclr(I2CTRIS,I2C_SDA);        //SDA = 0 SCL = 1 set SDA low for next step 
        DELAY_1_I2C;
        bitclr(I2CTRIS,I2C_SCL);     // SDA = 0 SCL = 0 and ready to go with next byte
#ifdef _NOT_SIMULATOR
        if (Carry)           // if NAK was then done with communication
        {
            bWork = 1;
            //break;
            goto CLEAN_I2C_OUT_QUEUE;//return;  
        }
#endif            

#else
        ///////////////////////////////////////////////////////////////////////////
        //                              non optimization part of a send
        ///////////////////////////////////////////////////////////////////////////
        bCount = 9;  // stupid but will be smaller code
#pragma updateBank 0
        // on entry SDA = 0 SCL = 0
        while (--bCount)
        {
            // on each itereation SDA = X SCL = 0
            if (bittest(bWork,7))
                bitset(I2CTRIS,I2C_SDA); //SDA = 1 SCL = 0 set SDA High
            else
                bitclr(I2CTRIS,I2C_SDA); //SDA = 0 SCL = 0 set SDA Low
            //DELAY_1_I2C; // this dealy is not nessesary - loop already does delay
            bitset(I2CTRIS,I2C_SCL);     // SDA = X SCL = 1 set SCL high
            DELAY_1_I2C;
            bitclr(I2CTRIS,I2C_SCL);     // SDA = X SCL = 0  set SCL low
            bWork <<=1;
            
        }

        // on exit SDA = X SCL = 0

        // bWork now is 0
        // wait for ACK
        //
        bitclr(I2CTRIS,I2C_SDA);         //SDA = 0 SCL = 0 set SDA low
        DELAY_1_I2C;
        // check:
        bitset(I2CTRIS,I2C_SCL);        // SDA = 0 SCL = 1 set SCL high to read ACK (or NACK)
CLOCK_STRETCH:
        if (!bittest(I2CPORT,I2C_SCL))   //  SCL must be release by recepient
        goto CLOCK_STRETCH;

        bitset(I2CTRIS,I2C_SDA);        //  now SDA = 1 (looks like high) with SCL = 0 prepear to read input value from receiver

#pragma updateBank 1
        if (bittest(I2CPORT,I2C_SDA))   //     if bit set then it is NAK - TBD then needs to repeat byte
            ++bWork;//bWork = 1;
        bitclr(I2CTRIS,I2C_SDA);        //SDA = 0 SCL = 1 set SDA low for next step 
        //DELAY_1_I2C;
        bitclr(I2CTRIS,I2C_SCL);     // SDA = 0 SCL = 0 and ready to go with next byte
#ifdef _NOT_SIMULATOR
        if (bittest(bWork,0))           // if NAK was then done with communication
        {
            break;
            //if (!I2C_B1.I2Cread)
                //bitset(PORTA,4);
            //goto CLEAN_I2C_OUT_QUEUE;//return;  
        }
        //else
        //{
        //    //if (!I2C_B1.I2Cread)
        //        //bitclr(PORTA,4);
        //}
#endif            

#endif  // NOT OPTIMIZED VERSION
    }
CLEAN_I2C_OUT_QUEUE:
    AOutI2CQu.iQueueSize = 0;
    AOutI2CQu.iEntry = 0;
    AOutI2CQu.iExit = 0;
    return bWork;
}
/////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////
void receiveI2C()
{
    unsigned char bWork;
    unsigned char bCount;
    // on entry SDA = 0 SCL = 0

///////////////////////////////////////////////////////////////////////////////
//                                          optimization part of receive
//////////////////////////////////////////////////////////////////////////////
#ifdef _OPTIMIZED_
    //GIE =0;
    bitset(I2CTRIS,I2C_SDA);     // SDA = 1 (input) SCL = 0
    DELAY_1_I2C;
    while(LenI2CRead)
    {
        bitset(I2CTRIS,I2C_SCL);     // SDA = 1(input) SCL = 1 
        bWork = 0;
RELESE_CHECK:
        if (!bittest(I2CPORT,I2C_SCL))  // check when SCL will be relesed TBD - can be endless loop
            goto RELESE_CHECK;
                                 //SDA = 1(INPUT) SCL = 1 now ready to read
        bCount = 9;  // stupid but will be smaller code
//#pragma updateBank 0
        //DELAY_1_I2C;            
        //goto READ_I_BEGIN;
//#pragma updateBank 0
        
        while (--bCount)
        {
            bitset(I2CTRIS,I2C_SCL);     // SDA = ? SCL = 1 
//#pragma updateBank 1
READ_I_BEGIN:
            //DELAY_1_I2C;            
            Carry = 0;
            if (bittest(I2CPORT,I2C_SDA))
                Carry = 1;
#ifdef      _18F2321_18F25K20
            #asm
              RLCF bWork,1,1
            #endasm
#else
            RLF(bWork,1);
#endif
            bitclr(I2CTRIS,I2C_SCL);     // SDA = ? SCL = 0
            //DELAY_1_I2C;
        }
        // send ACK (or NAK on last read byte
                                 // SDA = ? SCL = 0
        if (LenI2CRead == 1)
            bitset(I2CTRIS,I2C_SDA);     // SDA = 1 SCL = 0 NAK
        else
            bitclr(I2CTRIS,I2C_SDA);     // SDA = 0 SCL = 0  ACK
//DELAY_1_I2C;
        bitset(I2CTRIS,I2C_SCL);        //SDA = ACK or NAK SCL = 1 -> send
        DELAY_1_I2C;
        bitclr(I2CTRIS,I2C_SCL);        // SDA = ACK or NAK SCL = 0
                                        // delay will be all following code:
        LenI2CRead--;
#ifdef _NOT_SIMULATOR
#else
        bWork = '*';
#endif
        InsertI2C(bWork);
        bitset(I2CTRIS,I2C_SDA);     // SDA = 1 (input) SCL = 0
    } // to next loop it is SDA = 0 SCL = 0
    bitclr(I2CTRIS,I2C_SDA);     // SDA = 0 SCL = 0
#else         // START NON OPTIMIZED VERSION
    //////////////////////////////////////////////////////////////////////////////////
    //                                   non optimization part of receive
    //////////////////////////////////////////////////////////////////////////////////
    //GIE =0;
    bitset(I2CTRIS,I2C_SDA);     // SDA = 1 (input) SCL = 0
    DELAY_1_I2C;
    while(LenI2CRead)
    {
        bitset(I2CTRIS,I2C_SCL);     // SDA = 1(input) SCL = 1 
        bWork = 0;
RELESE_CHECK:
        if (!bittest(I2CPORT,I2C_SCL))  // check when SCL will be relesed TBD - can be endless loop
            goto RELESE_CHECK;
        
        //bitset(I2CTRIS,I2C_SDA); //SDA = 1(INPUT) SCL = 1 now ready to read
        

        bCount = 9;  // stupid but will be smaller code
        while (--bCount)
        {
            DELAY_1_I2C;            
            bWork <<=1;
            if (bittest(I2CPORT,I2C_SDA))
                bitset(bWork,0);
            
            bitclr(I2CTRIS,I2C_SCL);     // SDA = ? SCL = 0
            DELAY_1_I2C;
            if (bCount != 1)
                bitset(I2CTRIS,I2C_SCL);     // SDA = ? SCL = 1 
        }
        // send ACK (or NAK on last read byte
                                 // SDA = ? SCL = 0
     
        if (LenI2CRead == 1)
            bitset(I2CTRIS,I2C_SDA);     // SDA = 1 SCL = 0 NAK
        else
            bitclr(I2CTRIS,I2C_SDA);     // SDA = 0 SCL = 0  ACK
        DELAY_1_I2C;
        bitset(I2CTRIS,I2C_SCL);        //SDA = ACK or NAK SCL = 1 -> send
        DELAY_1_I2C;
        bitclr(I2CTRIS,I2C_SCL);        // SDA = ACK or NAK SCL = 0
                                // delay will be all following code:
        LenI2CRead--;
#ifdef _NOT_SIMULATOR
#else
        bWork = '*';
#endif
        InsertI2C(bWork);
        bitset(I2CTRIS,I2C_SDA);     // SDA = 1 (input) SCL = 0
    } // to next loop it is SDA = 0 SCL = 0
    bitclr(I2CTRIS,I2C_SDA);     // SDA = 0 SCL = 0

#endif   // END NOT OPTIMIZED VERSION

    //GIE =1;
}
void ReleseI2cMaster(void)
{
     if (I2C.NextI2CRead) // this will be restart condition
     {
         // at this call   SDA = 0 and SCL = 0
         DELAY_1_I2C;
         bitset(I2CTRIS,I2C_SDA); // SDA = 1 SCL = 0
         DELAY_1_I2C;
         bitset(I2CTRIS,I2C_SCL); // SDA = 1 SCL = 1;
         //DELAY_1_I2C;  // delay propagated
         return;
     }
     //SSPCON = 0b00011110;
     // at this call   SDA = 0 and SCL = 0
     DELAY_1_I2C;
     bitset(I2CTRIS,I2C_SCL); // SDA = 0 SCL = 1;
     DELAY_1_I2C;
     bitset(I2CTRIS,I2C_SDA); // SDA = 1 SCL = 1
     //SSPEN = 1;
}
#endif //I2C_INT_SUPPORT
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// end COPY 2
//////////////////////////////////////////////////////////////////////

// additional code:

void ProcessCMD(unsigned char bByte)
{
    unsigned char bWork;
    long wWork;
    long *FileLen;
    if (!Main.getCMD) // CMD not receved et.
    {

//#include "commc3.h"
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// begin COPY 3
/////////////////////////////////////////////////////////////////////
    //if (!Main.getCMD) // outside of the include was if == unit in "stream" relay mode
    //{
        // getCMD == 0
        // in stream was ESC char and now needs to echo that char to loop
        if (Main.ESCNextByte)
            Main.ESCNextByte = 0;
        else
        {  
            // if this is addressed to this unit then process it and switch "stream" -> "command" mode
            if (bByte == UnitADR)
            {
                Main.getCMD = 1; //next will be: <CMD>
                Main.SetFromAddr = 0;
                Main.SetSendCMD = 0;
                I2C.ESCI2CChar = 0;
                I2C.LastWasUnitAddr = 1;
                return;
            }
            else if (bByte == ESC_SYMB)   // ESC char - needs to echo next simbol to loop
                Main.ESCNextByte = 1;
        }
        // relay char to the loop, bcs now it is "stream" mode      
        putch(bByte); //ok
SKIP_ECHO_BYTE: ;
    }
    else    // now unit in command mode == processing all data
    {
        // getCMD == 1 
        // stream addressing this unit
        if (Main.ESCNextByte)
            Main.ESCNextByte = 0;
        else
        {
            if (bByte == ESC_SYMB)
            {
                I2C.LastWasUnitAddr = 0;
                if (!Main.PrepI2C)
                    Main.ESCNextByte = 1;
                else
                    I2C.ESCI2CChar = 1;
             
                return;
            }
            else if (bByte == UnitADR)
            {
                if (I2C.LastWasUnitAddr)  // pakets can not travel with 0 length - it is definetly was a lost packet and
                    return;           // needs to continue CMD mode  
                
                Main.getCMD = 0; // CMD stream done 
                if (Main.PrepI2C) // execute I2C if CMD stream done 
                {
                    bByte = '@';
                    goto END_I2C_MSG_WAIT;
                }
            }
            I2C.LastWasUnitAddr = 0;
        }
//////////////////////////////////////////////////////////////////////////////////
//  I2C command processing:
//     "<"<I2CAddr><DATA>@ or "<"<I2C addr><data><unit> 
//     "<"<I2Caddr><data>">"L@   or "<"<I2Caddr><data>">"L<unit> 
//         where L is a length data to read
//     ">"<I2C addr>L@  or ">"<I2C addr>L<unit> 
//         where L is a length bytes to read
//////////////////////////////////////////////////////////////////////////////////
        if (Main.PrepI2C) // stream addressing I2C 
        {
I2C_PROCESS:
            if (I2C.WaitQuToEmp)      // out queue was to be emptied before any next operation with another I2C
            {
                if (I2Caddr == 0xff)  
                {
                    I2Caddr = bByte;  // first after '<' is address 
                    return;
                }
WAIT_QU_EMP:
                if (AOutI2CQu.iQueueSize) // wait untill ouput I2C queue will be empty to start communication to I2C
                    goto WAIT_QU_EMP;
                //if (TXIE)
                //    goto WAIT_QU_EMP;
                I2C.WaitQuToEmp = 0;
                //BlockComm = 1;
            }
            if (I2C.NextI2CRead)
            {
                LenI2CRead = bByte;
                Main.PrepI2C = 0;
                goto END_I2C_MSG_WAIT;
            }
            if (I2C.ESCI2CChar)
            {
                I2C.ESCI2CChar = 0;
                goto PUT_CHAR;
            }
            if (bByte == '@') // this is end of the message
            {
                Main.PrepI2C = 0;
                goto END_I2C_MSG_WAIT;
            }
            if (bByte == '>') // this is end of the message and start read from same I2C device
            {
                I2C.NextI2CRead = 1;
                return;
            }
PUT_CHAR:

            putchI2C(bByte);
            if (AOutI2CQu.iQueueSize < 14) // packet can be long
                return;
END_I2C_MSG_WAIT:              // TBD this loop has to have limitation - bus can be dead
            if (I2C.NextI2CRead) // something expected ?
            {
                if (I2C.RetransI2CCom)
                {
                    InsertI2C('=');
                    InsertI2C(LenI2CRead | 0x80);
                }
            }
#ifdef I2C_INT_SUPPORT ////////////////////////////////////////////////////////
            InitI2cMaster();
WAIT_I2C_DONE:
            if (!I2C_B1.I2CMasterDone)  // needs to wait
                goto WAIT_I2C_DONE;
            if (bByte == '@')
                Main.PrepI2C = 0;
#else // not I2C_INT_SUPPORT /////////////////////////////////////////////////
            if (AOutI2CQu.iQueueSize) // this is a case when something in a queue and needs to send it
            {
                //if (I2C_B1.I2CBusBusy) // needs to wait when I2C will be not busy
                //    goto END_I2C_MSG_WAIT;
WAIT_I2C_START:
                if (InitI2cMaster()) // TBD I2C line busy by somebody else what to do?
                    goto WAIT_I2C_START;
                sendI2C();
                //if (sendI2C())  // if return not Zero == error in send everything else has to be skipped
                //    goto DONE_DONE_I2C;
                // out qu must be cleaned (in sendI2C)
                //AOutI2CQu.iQueueSize = 0;
                //AOutI2CQu.iEntry = 0;
                //AOutI2CQu.iExit = 0;
         
                if (bByte == '@')
                {
DONE_I2C:                
                    //I2Caddr = 0xff;
                    Main.PrepI2C = 0;
                    //BlockComm = 0;
                }
                ReleseI2cMaster();
            }
            if (I2C.NextI2CRead)
            {
                I2C_B1.I2Cread = 1;
I2C_WAIT_READ:              // TBD this loop has to have limitation - bus can be dead
                if (InitI2cMaster()) // if in restart somebody uses line then collision
                    goto DONE_DONE_I2C;
                sendI2C();
                //if (sendI2C()) // send address only - TBD needs to check how was ACK on address
                //    goto DONE_DONE_I2C;
                receiveI2C();
DONE_DONE_I2C:
                I2C.NextI2CRead = 0;
                goto DONE_I2C;
            }

#endif // not I2C_INT_SUPPORT/////////////////////////////////////////////////////////////
            Main.DoneWithCMD = 1; // long command ends
            return;
        }  // end if a adressing I2C stream
//////////////////////////////////////////////////////////////////////////////
// FLASH command processing
// set by external comman like F
//        F<length-of-packet><CMD><data>
//            send and receive responce from FLASH
//        F<length-of-packet><CMD><data>@<length-to-read>
//            in last case <length-of-packet> must include simbol '@'
//////////////////////////////////////////////////////////////////////////////
#ifdef SSPORT
        if (DataB3.FlashCmd)
        {
            if (DataB3.FlashCmdLen) // store length of a flash command
            {
                DataB3.FlashCmdLen = 0;
                CountWrite = bByte;
                DataB3.FlashRead = 0;
                CS_LOW;
            }
            else
            {
                if (DataB3.FlashRead)
                {
                    DataB3.FlashRead = 0;
                    if (!Main.ComNotI2C)
                    {
                        //do 
                        //{
                        //    InsertI2C(GetSSByte()); // read byte from FLASh will goes to I2C < 10 bytes
                        //} while(--bByte);
                        //InsertI2C('@');
                    }
                    else
                    {
                        Main.SendWithEsc = 1;
                        do 
                        {
                            putchWithESC(GetSSByte()); // read byte from FLASh will goes to Com
                                                       // if size bigger then 13 bytes it can be delay (putchWithESC waits out queue avalable space)
                        } while(--bByte);
                        Main.SendWithEsc = 0;
                        if (UnitFrom)
                            putch(UnitFrom);
                    }
                    goto DONE_WITH_FLASH;
                }
                else if (CountWrite == 1) // this will be last byte to write or it can be symb=@ request to read
                {
                    if (bByte == '@') // without CS_HIGH will be next read
                    {
                        DataB3.FlashRead = 1;
                        if (!Main.ComNotI2C) // CMD comes from I2C - reply from read should goes back to I2C
                        {
                            //InsertI2C('<');
                            //InsertI2C(UnitFrom);
                            //if (SendCMD)
                            //    InsertI2C(SendCMD);
                        }
                        else     // CMD comes from Com == relay (read) must go back to comm
                        {
                            if (UnitFrom)
                            {
                                putch(UnitFrom);
                                if (SendCMD)
                                    putch(SendCMD);
                            }
                        }
                        return;
                    }
                }
                SendSSByte(bByte);
                //SendSSByteFAST(bByte); //for testing only
                if (--CountWrite)
                    return;
DONE_WITH_FLASH:
                DataB3.FlashCmd = 0;
                CS_HIGH;
                if (!Main.ComNotI2C) // CMD comes from I2C - reply from read should goes back to I2C
                {
                     // initiate send using I2C
                     //InitI2cMaster();
                }
                else
                {
                    //if (UnitFrom)
                    //    putch(UnitFrom);
                }
                Main.DoneWithCMD = 1; // long command flash manipulation done 
            }
            return;
        }
#endif // SSPORT
/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// end COPY 3
////////////////////////////////////////////////////////////////////////

       
// additional code:

//#include "commc4.h"
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// begin COPY 4
///////////////////////////////////////////////////////////////////////   

        if (Main.SetFromAddr) //<unit>=Xci<unit> 
        {                     //       |         if ' ' than responce unit is not set
            if (bByte == ' ')
                bByte = 0;
            if (bByte == '*') // this will switch stream permanently
            {
                return;
            }
            if (bittest(bByte,7))
            {
                 RetransmitLen = bByte&0x7f;
                 Main.SetFromAddr = 0;
                 Main.DoneWithCMD = 0;
                 return;
            }
            UnitFrom = bByte;
            Main.SetFromAddr = 0;
            Main.SetSendCMD = 1;
            return;
        }
        else if (Main.SetSendCMD) //<unit>=xCi<unit> 
        {                         //        |        if ' ' than SendCMD is not set
            if (bByte == ' ')
                bByte = 0;
            SendCMD = bByte;
            Main.SetSendCMD = 0;
            I2C.SetI2CYesNo = 1;
            return;
        }
        else if (I2C.SetI2CYesNo) //<unit>=xcI<unit> I= com->I2C C = I2C->com ' '=nothing 
        {                         //         |
            I2C.SetI2CYesNo = 0;
            if (bByte == 'i') // it is just for convinience I2CReplyExpected can be set in any CMD
            {
                I2C.I2CReplyExpected = 1;
            }
            else if (bByte == 'I')
            {
                I2C.RetransComI2C = 1;
                I2C.RetransComI2CSet = 0;
                I2C.RetransI2CCom = 0;
            }
            else if (bByte == 'C') //<unit>=xcC = I2C->com ' '=nothing
            {
                I2C.RetransI2CCom = 1;
                I2C.RetransI2CComSet = 0;
                I2C.RetransComI2C = 0;
            }
            else // clean all set for retransmit
            {
                I2C.RetransI2CCom = 0;
                I2C.RetransI2CComSet = 0;
                I2C.RetransComI2C = 0;
            }
            Main.DoneWithCMD = 1; // long command =XCI done
        }
        else if (bByte == '=') // <unit>=XCI<unit> from unit = X, CMD to send =C (space = no CMD) I = expect retransmit over I2C
        {                      //  '=5CC' == to unit=5 with CMD=C over Type=C (Com) (operation SET)
                               //  '=5CI' == to unit=5 with CMD=C over Type=I (I2C) (opeartion SET) equivalent of <5C<DATA>@ 
                               //  '=*'   == to unit=5 with CMD=C over I2C == starting next byte all stream goes from com to I2C (retransmit)
                               //  '=*'   == to unit=5 with CMD=C over Com == starting next byte all stream goes from I2C to com (retransmit)
                               //  '=<NBIT+LEN>' (LEN < 128) next LEN bytes will goes to previously set device
                               //  high bit has to be set
            Main.DoneWithCMD = 0; // long command
            Main.SetFromAddr = 1;
            I2C.RetransComI2C = 0;
            I2C.RetransComI2CSet = 0;
            I2C.RetransI2CCom = 0;
        }
        // processing CMD
        else if (bByte == '~') // reseved test message from itself
        {
            Main.CommLoopOK = 1;
        }
        else if (bByte == '<') // "<"<I2CAddr><DATA>@ or "<"<I2C addr><data><unit> 
        {                      // "<"<I2Caddr><data>">"L@   or "<"<I2Caddr><data>">"L<unit> 
                               // where L is a length data to read
            Main.DoneWithCMD = 0; // long command
            I2Caddr = 0xff;
            Main.PrepI2C = 1;
            I2C_B1.I2Cread = 0;
            I2C.WaitQuToEmp = 1;
            I2C.NextI2CRead = 0;
       }
        else if (bByte == '>') // ><I2C addr>L@  or ><I2C addr>L<unit> where L is a length bytes to read
        {                      
            Main.DoneWithCMD = 0; // long command
            I2Caddr = 0xff;
            Main.PrepI2C = 1;
            I2C.WaitQuToEmp =  1;
            I2C.NextI2CRead = 1;
        }
#ifdef SSPORT
        else if (bByte == 'F') // manipulation with FLASH memory: read/write/erase/any flash command
        {
            Main.DoneWithCMD = 0; // long command
            DataB3.FlashCmd = 1;
            DataB3.FlashCmdLen = 1;
            // send something to FLASH
            // F<length-of-packet><CMD><data>
            // send and receive responce from FLASH
            // F<length-of-packet><CMD><data>@<length-to-read>
            // in last case <length-of-packet> must include simbol '@'
            // F\x01\x06              == write enable (flash command 06) -> send 0x06
            // F\x01\0xc7             == erase all flash                 -> send =0xc7
            // F\x05\x03\x00\x12\x34@\x04 == read 4 bytes from a address 0x001234  -> send 0x03 0x00 0x12 0x34 <- read 4 bytes (must not to cross boundary)
            // F\x01\x06F\x0c\x02\x00\x11\x22\x00\x00\x00\x00\x00\x00\x00\x00 == write 8 bytes to address 0x001122
            // F\x01\x06F\x04\x20\x00\x04\x00 == erase sector (4K) starting from address 0x000400
        }
#endif
/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// end COPY 4
////////////////////////////////////////////////////////////////////////

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




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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

    // CLKDIV:
    //     bit 15 ROI: Recover on Interrupt bit
    //         1 = Interrupts clears the DOZEN bit and the processor clock/peripheral clock ratio is set to 1:1
    //         0 = Interrupts have no effect on the DOZEN bit
    //     bit 14-12 DOZE<2:0>: Processor Clock Reduction Select bits
    //       111 = FCY/128
    //       110 = FCY/64
    //       101 = FCY/32
    //       100 = FCY/16
    //       011 = FCY/8 (default)
    //       010 = FCY/4
    //       001 = FCY/2
    //       000 = FCY/1
    //    bit 11 DOZEN: DOZE Mode Enable bit(1)
    //         1 = The DOZE<2:0> bits specify the ratio between the peripheral clocks and the processor clocks
    //         0 = Processor clock/peripheral clock ratio forced to 1:1
    CLKDIVbits.DOZE = 0b101 ; // speed of a processor is 40MOP => DOZE = 1.25 MOP (like 88)
    CLKDIVbits.ROI = 1;

//                                                       PIC24HJ128GP504
// I2C SDA            SDA1/RP9(1)/CN21/PMD3/RB9 |pin1              pin44| SCL1/RP8(1)/CN22/PMD4/RB8           I2C SCL
//                        RP22(1)/CN18/PMA1/RC6 |pin2              pin43| INT0/RP7(1)/CN23/PMD5/RB7           COM2 =>
//                        RP23(1)/CN17/PMA0/RC7 |pin3              pin42| PGEC3/ASCL1/RP6(1)/CN24/PMD6/RB6    COM1 TX =>
//                        RP23(1)/CN17/PMA0/RC7 |pin4              pin41| PGED3/ASDA1/RP5(1)/CN27/PMD7/RB5    COM1 RX <=
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


    // ANALOG configuration (for IR detector)
    // disable analog
	AD1CON1bits.ADON = 0; 
    // and switch analog pins to digital
    AD1PCFGL = 0xffff;
    //AD1PCFGH = 0xffff;

     // PORTA settings
    // porta is not re-mappable and on 504 device it is RA0-RA4 RA7=RA10
    // SPI output in FLASH mem terminoligy:
    // SSCLOCK RA0(pin19), SSDATA_IN RA1(pin20), SSDATA_OUT RA2(pin30), SSCS RA3(pin31)
    //          0            0                        IN                  1
    // RA7(pin13) - not used output
    // RA8(pin32) HD Camera ON/OFF button (0) not pressed (1) pressed
    // RA9(pin35) HD Camera click1 button to click (0) not pressed (1) pressed
    // RA10(pin12) - output - not used
    TRISA = 0b0000000000000100;  //0 = Output, 1 = Input 
    PORTA = 0b0000000000001000;

    // PORTB
    // pin24| RB3 pin23| RB2 RB1 |pin22 RB0 |pin21 
    //TRISB = 0b0000000000000100;  //0 = Output, 1 = Input 
    //PORTB = 0b0000000000001000;
    TRISBbits.TRISB12=0;    PORTBbits.RB12 =0; // <=SW1 COM2  
    TRISBbits.TRISB13=0;    PORTBbits.RB13 =0; // <=SW2 COM2  
    TRISBbits.TRISB11=0;    PORTBbits.RB11 =0; // <=SW3 COM2  
    
    TRISCbits.TRISC3=0;     PORTCbits.RC3 =0;  // HD Camera VIDEO record
    TRISCbits.TRISC4=0;     PORTCbits.RC4 =0;  // Switch MicroSD -> HDcam / MEM
    TRISCbits.TRISC5=0;     PORTCbits.RC5 =0;  // ON/OFF power GPS
    

    // this kaind funny, and for PIC24 and up = PRX pins can be reassigned to differrent preferias
    // additionaly needs to remember on which pin sits which PRX : // VSIAK SVERCHOK ZNAI SVOI SHESTOK
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock port remapping
    // INT0 == pin 16 
    // INT1 == pin 21 == PR10 
    //IN_FN_PPS_INT1 = IN_PIN_PPS_RP10; // RPINR0 
    // INT2 == pin 22 == PR11
    //IN_FN_PPS_INT2 = IN_PIN_PPS_RP11;

    // com1 == serial loop btw devices
    // PR5 - Serial RX  Pin 41
    IN_FN_PPS_U1RX = IN_PIN_PPS_RP5;
	// RR6 - Serial TX  Pin 42
    OUT_PIN_PPS_RP6 = OUT_FN_PPS_U1TX;

    // com2 == serial loop from (a) backup (b) GPS (c) camera (d) memory
    // PR7 - Serial2 RX  Pin 43
    IN_FN_PPS_U2RX = IN_PIN_PPS_RP7;
	// RR10 - Serial2 TX  Pin 8
    OUT_PIN_PPS_RP10 = OUT_FN_PPS_U2TX;

    // I2C:
    // SCL1 = I2C clock Pin 17 (this is NOT alernative I2c set as FPOR = 1 in configuration) 
    // SDA1 = I2C data Pin 18  this two pins permamet
    __builtin_write_OSCCONL(OSCCON | 0x40); //lock back port remapping


  // enable secondary oscilator

    OSCCONbits.LPOSCEN = 1;

    __builtin_write_OSCCONL(OSCCON | 0x40); //lock back port remapping

    TRISBbits.TRISB11=1;
     RtccInitClock();       //turn on clock source
     RtccWrOn();            //enable RTCC peripheral
     //RtccWriteTimeDate(&RtccTimeDate,TRUE);
     //RtccReadTimeDate(&RtccTimeDateVal);
     mRtccOn();
     RtccReadTimeDate(&RtccTimeDateVal);
     if ((RtccTimeDateVal.f.mday > 0x31) ||
         ((RtccTimeDateVal.f.mday & 0x0f) > 0x9) || 
         (RtccTimeDateVal.f.mon > 0x12) ||
         ((RtccTimeDateVal.f.mon & 0x0f) > 0x9) || 
         (RtccTimeDateVal.f.year < 0x12) ||
         (RtccTimeDateVal.f.hour>0x24) ||
         ((RtccTimeDateVal.f.hour & 0x0f)>0x9) )
     {
         RtccTimeDateVal.f.mday = 0x09;RtccTimeDateVal.f.mon=0x11;
         RtccTimeDateVal.f.year=0x12;
         RtccTimeDateVal.f.hour = 0x10;RtccTimeDateVal.f.min = 0x01;
         RtccTimeDateVal.f.sec = 0x01;RtccTimeDateVal.f.wday = 0x05;
         //mRtccOff();
         RtccWriteTimeDate(&RtccTimeDateVal,TRUE);
         mRtccOn();
     }

     

    //INT0_ENBL = 0; // disable external interrupt for GYRO 1
    //INT1IE = 0;    // disable external interrupt for GYRO2
    enable_uart(); //Setup the hardware UART for 20MHz at 9600bps
    // next two bits has to be set after all intialization done
    //PEIE = 1;    // bit 6 PEIE: Peripheral Interrupt Enable bit
                 // 1 = Enables all unmasked peripheral interrupts
                 // 0 = Disables all peripheral interrupts
    //GIE = 1;     // bit 7 GIE: Global Interrupt Enable bit
                 // 1 = Enables all unmasked interrupts
                 // 0 = Disables all interrupts
    //enable_I2C();
    //TIMER0_INT_FLG = 0; // clean timer0 interrupt
    //TIMER0_INT_ENBL = 0; // diasable timer0 interrupt
    //TMR1IF = 0; // clean timer0 interrupt
    //TMR1IE = 0; // diasable timer0 interrupt
    //INT0_EDG = 1; // 1 = Interrupt on negative edge
    //INT0_FLG = 0; // clean extrnal interrupt RB0 pin 6

    //INT1IF = 0;
    //INTEDG1 = 1;    

    //INT2IF = 0;
    //INTEDG2 = 1;    

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
//#include "commc8.h"
/////////////////////////////////////////////////////////////////
//      Begin COPY 8
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#ifndef __PIC24H__
// EECON1
//        bit 7 EEPGD: Flash Program or Data EEPROM Memory Select bit
//            1 = Access Flash program memory
//            0 = Access data EEPROM memory
//        bit 6 CFGS: Flash Program/Data EEPROM or Configuration Select bit
//            1 = Access Configuration registers
//            0 = Access Flash program or data EEPROM memory
//        bit 5 Unimplemented: Read as '0'
//        bit 4 FREE: Flash Row Erase Enable bit
//            1 = Erase the program memory row addressed by TBLPTR on the next WR command (cleared
//               by completion of erase operation)
//           0 = Perform write only
//        bit 3 WRERR: Flash Program/Data EEPROM Error Flag bit
//            1 = A write operation is prematurely terminated (any Reset during self-timed programming in
//                normal operation, or an improper write attempt)
//            0 = The write operation completed
//          Note: When a WRERR occurs, the EEPGD and CFGS bits are not cleared.
//          This allows tracing of the error condition.
//        bit 2 WREN: Flash Program/Data EEPROM Write Enable bit
//            1 = Allows write cycles to Flash program/data EEPROM
//            0 = Inhibits write cycles to Flash program/data EEPROM
//        bit 1 WR: Write Control bit
//            1 = Initiates a data EEPROM erase/write cycle or a program memory erase cycle or write cycle
//                (The operation is self-timed and the bit is cleared by hardware once write is complete.
//                The WR bit can only be set (not cleared) in software.)
//            0 = Write cycle to the EEPROM is complete
//        bit 0 RD: Read Control bit
//            1 = Initiates an EEPROM read
//                (Read takes one cycle. RD is cleared in hardware. The RD bit can only be set (not cleared)
//                 in software. RD bit cannot be set when EEPGD = 1 or CFGS = 1.)
//            0 = Does not initiate an EEPROM read
unsigned char eeprom_read(unsigned char addr)
{
    EEADR = addr;
    EECON1 &= 0x3F;
    RD = 1;
    return EEDATA;
}
void eeprom_write(unsigned char addr, unsigned char value)
{
    EEADR = addr;
    EEDATA = value;
    EECON1 &= 0x3F;
    Carry = 0;
    if(GIE)
        Carry = 1;
    GIE = 0;
    WREN = 1;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    WR = 1;
    WREN = 0;
    if(Carry)
        GIE = 1;
    while(WR)
    {
    }
}
#endif

void enable_uart(void)//bit want_ints)
{
 
#ifdef __PIC24H__
    TXIE = 0;  // this is macro redifinitions to be compatible with 16LF88 16LF884 18F2321
    RCIE = 0; // disable interrupt on recieve byte
    U1MODE = 0b000100010001000;
             //0                // bit 15 UARTEN: UARTx Enable bit(1)
                                //         1 = UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN<1:0>
                                //         0 = UARTx is disabled; all UARTx pins are controlled by port latches; UARTx power consumption minimal
                                // bit 14 Unimplemented: Read as ‘0’
             // 0               // bit 13 USIDL: Stop in Idle Mode bit
                                //         1 = Discontinue module operation when device enters Idle mode
                                //         0 = Continue module operation in Idle mode
             //  0              // bit 12 IREN: IrDA® Encoder and Decoder Enable bit(2)
                                //         1 = IrDA encoder and decoder enabled
                                //         0 = IrDA encoder and decoder disabled
             //   1             // bit 11 RTSMD: Mode Selection for UxRTS Pin bit
                                //         1 = UxRTS pin in Simplex mode
                                //         0 = UxRTS pin in Flow Control mode
             //    0            // bit 10 Unimplemented: Read as ‘0’
                                // bit 9-8 UEN<1:0>: UARTx Enable bits
             //     00          //        11 = UxTX, UxRX and BCLK pins are enabled and used; UxCTS pin controlled by port latches
                                //        10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
                                //        01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by port latches
                                //        00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLK pins controlled by port latches
             //       1         // bit 7 WAKE: Wake-up on Start bit Detect During Sleep Mode Enable bit
                                //         1 = UARTx continues to sample the UxRX pin; interrupt generated on falling edge; bit cleared in hardware on following rising edge
                                //         0 = No wake-up enabled
             //        0        // bit 6 LPBACK: UARTx Loopback Mode Select bit
                                //         1 = Enable Loopback mode
                                //         0 = Loopback mode is disabled
             //         0       // bit 5 ABAUD: Auto-Baud Enable bit
                                //         1 = Enable baud rate measurement on the next character – requires reception of a Sync field (55h)
                                //             before other data; cleared in hardware upon completion
                                //         0 = Baud rate measurement disabled or completed
             //          0      // bit 4 URXINV: Receive Polarity Inversion bit
                                //        1 = UxRX Idle state is ‘0’
                                //        0 = UxRX Idle state is ‘1’
             //           1     // bit 3 BRGH: High Baud Rate Enable bit
                                //        1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
                                //        0 = BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
             //            00   // bit 2-1 PDSEL<1:0>: Parity and Data Selection bits
                                //       11 = 9-bit data, no parity
                                //       10 = 8-bit data, odd parity
                                //       01 = 8-bit data, even parity
                                //       00 = 8-bit data, no parity
             //              0  // bit 0 STSEL: Stop Bit Selection bit
                                //        1 = Two Stop bits
                                //        0 = One Stop bit
    U1BRG = SPBRG_SPEED;
    U1STAbits.UTXBRK = 0;
    U1STAbits.UTXISEL1 = 0;     // 11 = Reserved
    U1STAbits.UTXISEL0 = 0;     // 10 = Interrupt generated when a character is transferred to the Transmit Shift register and the transmit buffer becomes empty
                                // 01 = Interrupt generated when the last transmission is over (last character shifted out of Transmit Shift register) and all the transmit operations are completed
                                // 00 = Interrupt generated when any character is transferred to the Transmit Shift Register (this implies at least one location is empty in the transmit buffer)

    U1STAbits.URXISEL1 = 0;
    U1STAbits.URXISEL0 = 0;     // 11 = Interrupt is set on UxRSR transfer making the receive buffer full (i.e., has 4 data characters)
                                // 10 = Interrupt is set on UxRSR transfer making the receive buffer 3/4 full (i.e., has 3 data characters)
                                // 0x = Interrupt is set when any character is received and transferred from the UxRSR to the receive buffer. Receive buffer has one or more characters
    TXEN = 0;  // this is macro redifinitions to be compatible with 16LF88 16LF884 18F2321 // disable transmit
    RCIF = 0;  // clean bit of receive interrupt
    RCIE = 1;  // enable interrupt on recieve byte
    U1MODEbits.UARTEN = 1; // enable uart
#ifdef USE_COM2
    // second UART
    TXIECOM2 = 0;  // this is macro redifinitions to be compatible with 16LF88 16LF884 18F2321
    RCIECOM2 = 0; // disable interrupt on recieve byte
#ifdef INVERS_COM2
    U2MODE = 0b000100010011000;
#else
    U2MODE = 0b000100010001000;
#endif
             //0                // bit 15 UARTEN: UARTx Enable bit(1)
                                //         1 = UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN<1:0>
                                //         0 = UARTx is disabled; all UARTx pins are controlled by port latches; UARTx power consumption minimal
                                // bit 14 Unimplemented: Read as '0'
             // 0               // bit 13 USIDL: Stop in Idle Mode bit
                                //         1 = Discontinue module operation when device enters Idle mode
                                //         0 = Continue module operation in Idle mode
             //  0              // bit 12 IREN: IrDA Encoder and Decoder Enable bit(2)
                                //         1 = IrDA encoder and decoder enabled
                                //         0 = IrDA encoder and decoder disabled
             //   1             // bit 11 RTSMD: Mode Selection for UxRTS Pin bit
                                //         1 = UxRTS pin in Simplex mode
                                //         0 = UxRTS pin in Flow Control mode
             //    0            // bit 10 Unimplemented: Read as '0'
                                // bit 9-8 UEN<1:0>: UARTx Enable bits
             //     00          //        11 = UxTX, UxRX and BCLK pins are enabled and used; UxCTS pin controlled by port latches
                                //        10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
                                //        01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by port latches
                                //        00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLK pins controlled by port latches
             //       1         // bit 7 WAKE: Wake-up on Start bit Detect During Sleep Mode Enable bit
                                //         1 = UARTx continues to sample the UxRX pin; interrupt generated on falling edge; bit cleared in hardware on following rising edge
                                //         0 = No wake-up enabled
             //        0        // bit 6 LPBACK: UARTx Loopback Mode Select bit
                                //         1 = Enable Loopback mode
                                //         0 = Loopback mode is disabled
             //         0       // bit 5 ABAUD: Auto-Baud Enable bit
                                //         1 = Enable baud rate measurement on the next character requires reception of a Sync field (55h)
                                //             before other data; cleared in hardware upon completion
                                //         0 = Baud rate measurement disabled or completed
             //          1      // bit 4 URXINV: Receive Polarity Inversion bit
                                //        1 = UxRX Idle state is '0'
                                //        0 = UxRX Idle state is '1'
             //           1     // bit 3 BRGH: High Baud Rate Enable bit
                                //        1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
                                //        0 = BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
             //            00   // bit 2-1 PDSEL<1:0>: Parity and Data Selection bits
                                //       11 = 9-bit data, no parity
                                //       10 = 8-bit data, odd parity
                                //       01 = 8-bit data, even parity
                                //       00 = 8-bit data, no parity
             //              0  // bit 0 STSEL: Stop Bit Selection bit
                                //        1 = Two Stop bits
                                //        0 = One Stop bit
    U2BRG = SPBRG_SPEEDCOM2;
    //   boud rate formula: BRGH = 0
    // BoudRate = Fcy/ (16 * (UxBRG + 1))
    //                      BRGH = 1
    // BoudRate = Fcy/ (4 * (UxBRG + 1)
    // for example on Fcy = 10.13375 MHz = 10133750 and U2BRG = 263 BoudRate = 9596
    // #define SPBRG_9600_10MIPS 262
    // #define SPBRG_57600_10MIPS 43

    U2STAbits.UTXBRK = 0;
    U2STAbits.UTXISEL1 = 0;     // 11 = Reserved
    U2STAbits.UTXISEL0 = 0;     // 10 = Interrupt generated when a character is transferred to the Transmit Shift register and the transmit buffer becomes empty
                                // 01 = Interrupt generated when the last transmission is over (last character shifted out of Transmit Shift register) and all the transmit operations are completed
                                // 00 = Interrupt generated when any character is transferred to the Transmit Shift Register (this implies at least one location is empty in the transmit buffer)

    U2STAbits.URXISEL1 = 0;
    U2STAbits.URXISEL0 = 0;     // 11 = Interrupt is set on UxRSR transfer making the receive buffer full (i.e., has 4 data characters)
                                // 10 = Interrupt is set on UxRSR transfer making the receive buffer 3/4 full (i.e., has 3 data characters)
                                // 0x = Interrupt is set when any character is received and transferred from the UxRSR to the receive buffer. Receive buffer has one or more characters

                                // bit 15,13 UTXISEL<1:0>: Transmission Interrupt Mode Selection bits
                                //    11 = Reserved; do not use
                                //    10 = Interrupt when a character is transferred to the Transmit Shift Register, and as a result, the
                                //         transmit buffer becomes empty
                                //    01 = Interrupt when the last character is shifted out of the Transmit Shift Register; all transmit
                                //         operations are completed
                                //    00 = Interrupt when a character is transferred to the Transmit Shift Register (this implies there is
                                //         at least one character open in the transmit buffer)
                                // bit 14 UTXINV: Transmit Polarity Inversion bit
                                //         If IREN = 0:
                                //     1 = UxTX Idle state is '0'
                                //     0 = UxTX Idle state is '1'
                                //         If IREN = 1:
                                //     1 = IrDA encoded UxTX Idle state is '1'
                                //     0 = IrDA encoded UxTX Idle state is '0'
                                // bit 12 Unimplemented: Read as '0'
                                // bit 11 UTXBRK: Transmit Break bit
                                //     1 = Send Sync Break on next transmission Start bit, followed by twelve '0' bits, followed by Stop bit;
                                //         cleared by hardware upon completion
                                //     0 = Sync Break transmission disabled or completed
                                // bit 10 UTXEN: Transmit Enable bit(1)
                                //     1 = Transmit enabled, UxTX pin controlled by UARTx
                                //     0 = Transmit disabled, any pending transmission is aborted and buffer is reset. UxTX pin controlled
                                //          by port
                                // bit 9 UTXBF: Transmit Buffer Full Status bit (read-only)
                                //     1 = Transmit buffer is full
                                //     0 = Transmit buffer is not full, at least one more character can be written
                                // bit 8 TRMT: Transmit Shift Register Empty bit (read-only)
                                //     1 = Transmit Shift Register is empty and transmit buffer is empty (the last transmission has completed)
                                //     0 = Transmit Shift Register is not empty, a transmission is in progress or queued
                                // bit 7-6 URXISEL<1:0>: Receive Interrupt Mode Selection bits
                                //    11 = Interrupt is set on UxRSR transfer making the receive buffer full (i.e., has 4 data characters)
                                //    10 = Interrupt is set on UxRSR transfer making the receive buffer 3/4 full (i.e., has 3 data characters)
                                //    0x = Interrupt is set when any character is received and transferred from the UxRSR to the receive
                                //         buffer. Receive buffer has one or more characters
                                // bit 5 ADDEN: Address Character Detect bit (bit 8 of received data = 1)
                                //     1 = Address Detect mode enabled. If 9-bit mode is not selected, this does not take effect
                                //     0 = Address Detect mode disabled
                                // bit 4 RIDLE: Receiver Idle bit (read-only)
                                //     1 = Receiver is Idle
                                //     0 = Receiver is active
                                // bit 3 PERR: Parity Error Status bit (read-only)
                                //     1 = Parity error has been detected for the current character (character at the top of the receive FIFO)
                                //     0 = Parity error has not been detected
                                // bit 2 FERR: Framing Error Status bit (read-only)
                                //     1 = Framing error has been detected for the current character (character at the top of the receive
                                //         FIFO)
                                //     0 = Framing error has not been detected
                                // bit 1 OERR: Receive Buffer Overrun Error Status bit (read/clear only)
                                //     1 = Receive buffer has overflowed
                                //     0 = Receive buffer has not overflowed. Clearing a previously set OERR bit (1 ? 0 transition) resets
                                //         the receiver buffer and the UxRSR to the empty state
                                // bit 0 URXDA: Receive Buffer Data Available bit (read-only)
                                //     1 = Receive buffer has data, at least one more character can be read
                                //     0 = Receive buffer is empty


#ifdef INVERS_COM2
    U2STAbits.UTXINV = 1;       // inversion of TX data == idle = 0
#else
    U2STAbits.UTXINV = 0;       // TX data == idle = 1
#endif
    TXENCOM2 = 0;  // this is macro redifinitions to be compatible with 16LF88 16LF884 18F2321 // disable transmit
    RCIFCOM2 = 0;  // clean bit of receive interrupt
    RCIECOM2 = 1;  // enable interrupt on recieve byte
    U2MODEbits.UARTEN = 1; // enable uart
#endif // second uart

#else // 88,884,2321

    TX9 = 0;
    RX9 = 0;
//#ifdef _18F2321_18F25K20
//    SPBRGH = 0;
//    BRG16 = 0;
//#endif

    BRGH = 1; //Normal speed UART port 0x98 
              // 00000x00 BRGH: High Baud Rate Select bit
              //Asynchronous mode:
              // 1 = High speed 0 = Low speed
    SPBRG = SPBRG_SPEED;//51;// 9600
    //SPBRG = 25;// 19200
    //SPBRG = 12;// 38400
    //SPBRG = 8; // 57600

    SYNC = 0; // port 0x98 = 000x0000 x=0 asynch mode 1-synchr
    SPEN = 1; // port 0x18 x0000000 SPEN: Serial Port Enable bit
              // 1 = Serial port enabled (configures RB2/SDO/RX/DT 
              // and RB5/SS/TX/CK pins as serial port pins)

    //TXIF = 0;
    TXIE = 0;
    TXEN = 0;
    //TXEN = 1; //Enable transmission port 0x8c
              // AUSART Transmit Interrupt Enable bit
              // 1 = Enabled 0 = Disabled
    ADDEN = 0;
    //RCIF = 0;  // clean bit of receive interrupt
    RCIE = 1; // enable interrupt on recieve byte
    CREN = 1; // port 0x18 000x0000 CREN: Continuous Receive Enable bit
              // Asynchronous mode: 1 = Enables continuous receive
    
    //WREN = 1;
#endif
}    
void enable_I2C(void)
{
#ifdef __PIC24H__
   I2C1CON = 0b1000000000000000; 
                                 // bit 15 I2CEN: I2Cx Enable bit
           //  1                 //      1 = Enables the I2Cx module and configures the SDAx and SCLx pins as serial port pins
                                 //      0 = Disables the I2Cx module; all I2C pins are controlled by port functions
           //   0                // bit 14 Unimplemented: Read as ‘0’
           //    0               // bit 13 I2CSIDL: Stop in Idle Mode bit
                                 //      1 = Discontinue module operation when device enters Idle mode
                                 //      0 = Continue module operation in Idle mode
           //     0              // bit 12 SCLREL: SCLx Release Control bit (when operating as I2C slave)
                                 //      1 = Release SCLx clock 
                                 //      0 = Hold SCLx clock low (clock stretch)
                                 //          If STREN = 1:
                                 //          Bit is R/W (i.e., software may write ‘0’ to initiate stretch and write ‘1’ to release clock). Hardware clear
                                 //          at beginning of slave transmission and at end of slave reception.
                                 //          If STREN = 0:
                                 //          Bit is R/S (i.e., software may only write ‘1’ to release clock). Hardware clear at beginning of slave transmission.
           //      0             // bit 11 IPMIEN: Intelligent Platform Management Interface (IPMI) Enable bit
                                 //      1 = IPMI Support mode is enabled; all addresses Acknowledged
                                 //      0 = IPMI Support mode disabled
           //       0            // bit 10 A10M: 10-Bit Slave Address bit
                                 // 1 = I2CxADD register is a 10-bit slave address 0 = I2CxADD register is a 7-bit slave address
           //        0           // bit 9 DISSLW: Disable Slew Rate Control bit
                                 //      1 = Slew rate control disabled 0 = Slew rate control enabled
           //         0          // bit 8 SMEN: SMBus Input Levels bit
                                 // 1 = Enable I/O pin thresholds compliant with SMBus specification 0 = Disable SMBus input thresholds
           //          0         // bit 7 GCEN: General Call Enable bit (when operating as I2C slave)
                                 //      1 = Enable interrupt when a general call address is received in the I2CxRSR register (module is enabled for reception)
                                 //      0 = General call address disabled
           //           0        // bit 6 STREN: SCLx Clock Stretch Enable bit (I2C Slave mode only; used in conjunction with SCLREL bit)
                                 //      1 = Enable software or receive clock stretching 0 = Disable software or receive clock stretching
           //            0       // bit 5 ACKDT: Acknowledge Data bit (I2C Master mode; receive operation only) 
                                 // Value that will be transmitted when the software initiates an Acknowledge sequence
                                 //      1 = Send NACK during Acknowledge 0 = Send ACK during Acknowledge
           //             0      // bit 4 ACKEN: Acknowledge Sequence Enable bit (I2C Master mode receive operation)
                                 //      1 = Initiate Acknowledge sequence on SDAx and SCLx pins and transmit ACKDT data bit (hardware clear at end of master Acknowledge sequence)
                                 //      0 = Acknowledge sequence not in progress
           //              0     // bit 3 RCEN: Receive Enable bit (I2C Master mode)
                                 //      1 = Enables Receive mode for I2C (hardware clear at end of eighth bit of master receive data byte)
                                 //      0 = Receive sequence not in progress
           //               0    // bit 2 PEN: Stop Condition Enable bit (I2C Master mode)
                                 //      1 = Initiate Stop condition on SDAx and SCLx pins (hardware clear at end of master Stop sequence)
                                 //      0 = Stop condition not in progress
           //                0   // bit 1 RSEN: Repeated Start Condition Enable bit (I2C Master mode)
                                 //      1 = Initiate Repeated Start condition on SDAx and SCLx pins (hardware clear at end of master Repeated Start sequence)
                                 //      0 = Repeated Start condition not in progress
           //                 0  // bit 0 SEN: Start Condition Enable bit (I2C Master mode)
                                 //      1 = Initiate Start condition on SDAx and SCLx pins (hardware clear at end of master Start sequence)
                                 //      0 = Start condition not in progress
      IFS1bits.MI2C1IF = 0; // clean interrupt
      IEC1bits.MI2C1IE = 1; // enable interupt on master
#else // end of PIC24

#ifdef _18F2321_18F25K20
 #ifndef I2C_ONLY_MASTER
    SSPCON1 =0b00111110; // TBD in _18F2321_18F25K20 master I2C implemented 
 #else
    SSPCON1 =0b00011000;
    SSPCON2 =0b00000000;
 #endif
#else // 88, 884
    #warning "pic18f884 does have I2C master support in firmaware"
    SSPCON = 0b00111110;
#endif
             //0        -WCOL: Write Collision Detect bit
             //          1 = An attempt to write the SSPBUF register failed because the SSP module is busy
             //              (must be cleared in software)
             //          0 = No collision
             // 0       -SSPOV: Receive Overflow Indicator bit
             //          1 = A byte is received while the SSPBUF register is still holding the previous byte. SSPOV is
             //              a “don’t care” in Transmit mode. SSPOV must be cleared in software in either mode.
             //          0 = No overflow
             //  1      -SSPEN: Synchronous Serial Port Enable bit
             //          1 = Enables the serial port and configures the SDA and SCL pins as serial port pins
             //          0 = Disables serial port and configures these pins as I/O port pins
             //   1     -CKP: Clock Polarity Select bit.SCK release control
             //          1 = Enable clock
             //          0 = Holds clock low (clock stretch). (Used to ensure data setup time.)
             //    1110-SSPM<3:0>: Synchronous Serial Port Mode Select bits
             //          0110 = I2C Slave mode, 7-bit address
             //          0111 = I2C Slave mode, 10-bit address
             //          1011 = I2C Firmware Controlled Master mode (Slave Idle)
             //          1110 = I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
             //          1111 = I2C Slave mode, 10-bit address with Start and Stop bit interrupts enabled
             //          1000 = I2C MASTER mode
    SMP = 0;
    CKE = 0;

    SSPEN = 1; // enable I2C
    //P = 0;     // no stop
    //S = 0;     // no start
    //BF = 0;    // nothing in a buffer (may be better read SSPBUFF?)
    SSPIF = 0;  // clean inderrupt
    SSPIE = 1;  // enable interrupt
    //SSPOV = 0;  // clean owerflow
#endif
}
void EnableTMR1(void)
{
#ifdef __PIC24H__
#else
    TMR1L =0;
    TMR1H = 0;
    T1CON = 0b1001001;
    //                 T1CON: TIMER1 CONTROL REGISTER (ADDRESS 10h)
    //        1        TMR1ON: Timer1 On bit 1 = Enables Timer1 0 = Stops Timer1
    //                         1 = External clock from pin RB6/AN5(1)/PGC/T1OSO/T1CKI (on the rising edge)
    //                         0 = Internal clock (FOSC/4)
    //         00      T1CKPS<1:0>: Timer1 Input Clock Prescale Select bits
    //                        11 =1:8 Prescale value with FOSC/4 == 16cmd per count
    //                        10 =1:4 Prescale value with FOSC/4 == 8 cmd per count
    //                        01 =1:2 Prescale value with FOSC/4 == 4 cmd per count
    //                        00 =1:1 Prescale value with FOSC/4 == 2 cmd per count
    //           1     T1OSCEN - 1 ocsilator enabled 0 disabled
    //            0    T1SYNC  - ignored with TMR1CS == 0
    //             0   TMR1CS: Timer1 Clock Source Select bit internal FOSC/4 
    //              1  TMR1ON - timer enabled
    TMR1IF = 0;
    TMR1IE = 0;
    //TMR1ON = 1;  // TMR1ON: Timer1 On bit
                 // 1 = Enables Timer1
                 // 0 = Stops Timer1
#endif
}
//#define SSPORT PORTA
//#define SSCLOCK 7
//#define SSDATA_IN 6
//#define SSDATA_OUT 5
//#define SSCS       4
#ifdef SSPORT
// it is working with port in bank 0
#pragma rambank RAM_BANK_0
void SendSSByte(unsigned char bByte)
{
    WORD bWork;
    //bitclr(SSPORT,SSCS); // set low Chip Select
    bWork = 8;
#pragma updateBank 0

    do
    {
#ifdef __PIC24H__
        // nobody uses portA ??? make sure!!!
        // otherwise it must be 3 commands instead of one:
        // bclr(SSPORT,SSCLOCK);
        // nop();
        // bclr(SSPORT,SSDATA_IN);

        // SSCLOCK RA0(pin2), SSDATA_IN RA1(pin3), SSDATA_OUT RA2(pin9), SSCS RA3(pin10)
        PORTA = 0b00000000;
#else
        bclr(SSPORT,SSCLOCK);
        bclr(SSPORT,SSDATA_IN);
#endif
		if (bittest(bByte,7))
            bset(SSPORT,SSDATA_IN);
#ifdef _OPTIMIZED_
  #ifdef __PIC24H__
         bByte<<=1;
  #else
   #ifdef      _18F2321_18F25K20
            #asm
              RLCF bByte,1,1
            #endasm
   #else
            RLF(bByte,1);
   #endif
  #endif
#else // not optimized version
        bByte<<=1;
#endif
        bset(SSPORT,SSCLOCK);
    }
    while (--bWork); // 7*8 = 56 or 8*8 = 64 commands
    bclr(SSPORT,SSCLOCK);
    //nop();
    //bclr(SSPORT,SSDATA_IN);
    //bset(SSPORT,SSCS); // set high Chip Select
}
#pragma updateBank 1
void SendSSByteFAST(unsigned char bByte)
{
#ifdef __PIC24H__
    // nobody uses portA ??? make sure!!!
    // otherwise it must be changed on bit 1:
    // SSCLOCK RA0(pin2), SSDATA_IN RA1(pin3), SSDATA_OUT RA2(pin9), SSCS RA3(pin10)
    bclr(SSPORT,SSDATA_IN);
    nop();
    bset(SSPORT,SSCLOCK);  // bit 7
    nop();
    bclr(SSPORT,SSCLOCK);
    nop();
    bset(SSPORT,SSCLOCK);  // bit 6
    nop();
    bclr(SSPORT,SSCLOCK);
    nop();
    bset(SSPORT,SSCLOCK);  // bit 5
    nop();
    bclr(SSPORT,SSCLOCK);
    nop();
    bset(SSPORT,SSCLOCK);  // bit 4
    nop();
    bclr(SSPORT,SSCLOCK);
    nop();
    bset(SSPORT,SSCLOCK);  // bit 3
    nop();
    bclr(SSPORT,SSCLOCK);
    nop();
    bset(SSPORT,SSCLOCK);  // bit 2
    nop();
    bclr(SSPORT,SSCLOCK);
    //if (bittest(bByte,1))
    if (bByte&2)
    {
        bset(SSPORT,SSDATA_IN);
        nop();
    }
    bset(SSPORT,SSCLOCK);  // bit 1
    nop();
    PORTA = 0b00000000;
    //bclr(SSPORT,SSCLOCK);
    //nop();
    //bclr(SSPORT,SSDATA_IN); // essential

    //if (bittest(bByte,0))
    if (bByte&1)
    {
        bset(SSPORT,SSDATA_IN);
        nop();
    }
    bset(SSPORT,SSCLOCK);  // bit 0
    nop();
    bclr(SSPORT,SSCLOCK);  // 42 commands
#else
    bclr(SSPORT,SSDATA_IN);
    bset(SSPORT,SSCLOCK);  // bit 7
    bclr(SSPORT,SSCLOCK);
    bset(SSPORT,SSCLOCK);  // bit 6
    bclr(SSPORT,SSCLOCK);
    bset(SSPORT,SSCLOCK);  // bit 5
    bclr(SSPORT,SSCLOCK);
    bset(SSPORT,SSCLOCK);  // bit 4
    bclr(SSPORT,SSCLOCK);
    bset(SSPORT,SSCLOCK);  // bit 3
    bclr(SSPORT,SSCLOCK);
    bset(SSPORT,SSCLOCK);  // bit 2
    bclr(SSPORT,SSCLOCK);
    //if (bittest(bByte,1))
    if (bByte&2)
        bset(SSPORT,SSDATA_IN);
    bset(SSPORT,SSCLOCK);  // bit 1
    bclr(SSPORT,SSCLOCK);
    bclr(SSPORT,SSDATA_IN);
    //if (bittest(bByte,0))
    if (bByte&1)
        bset(SSPORT,SSDATA_IN);
    bset(SSPORT,SSCLOCK);  // bit 0
    bclr(SSPORT,SSCLOCK);  // 23 commands
#endif
}

unsigned char GetSSByte(void)
{
    int bWork;
    unsigned int bWork2;
    //bitclr(SSPORT,SSCS); // set low Chip Select
    bWork = 8;
#pragma updateBank 0
    bWork2 = 0;
    do
    {
        bWork2 <<=1;
        bset(SSPORT,SSCLOCK);
        //nop();
        //bitclr(bWork2,0); // bWork2 is unsigned == zero in low bit garanteed check assembler code to confirm
        if (btest(SSPORT,SSDATA_OUT))
            bitset(bWork2,0);
        bclr(SSPORT,SSCLOCK);
    }
    while (--bWork);
    return bWork2;
    //bset(SSPORT,SSCS); // set high Chip Select
}
#pragma updateBank 1
#endif
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// end COPY 8
///////////////////////////////////////////////////////////////////////

