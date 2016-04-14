#include <stdio.h>
#include <wiringPi.h>

/* Written by Gina Hortenbach */

/*
   This code is intended to run on the Raspberry Pi with the WiriPi Library.
   It has the advantage over the Arduino version in that less shift registers
   are required and dumping is significantly faster.

   WiriPi can cause troubles. If you run into trouble check you have version:

     wiringPi-78b5c32

   The code is currently configured for a dump setup that uses ShiftOut
   registers for address lines and Rapsberry Pi GPIO pins for input data.
   See ParallelFLASHDumper_rpi.jpg

   Compile:

   cc -o ParallelFLASHDumper_rpi ParallelFLASHDumper_rpi.c -lwiringPi -lpthread

*/

/*
 * BEGIN USER DEFINITIONS
 */
#define ADDR_STROBE      25  // latch orange
#define ADDR_DATAOUT     29  // green
#define ADDR_CLOCK       28  // yellow
#define ADDR_A24         26

// define according to the size of your target chip
#define MAX_ADDR    0xFFFFFFFF


// INPUT DATA pins
#define IN_DQ00        9 //
#define IN_DQ01        7 //
#define IN_DQ02        1 //
#define IN_DQ03        2 //
#define IN_DQ04        3 //
#define IN_DQ05        5 //
#define IN_DQ06        6 //
#define IN_DQ07       14 //
//#define IN_DQ08       15 //
#define IN_DQ08       31 //
#define IN_DQ09       16 //
#define IN_DQ10        0 //
#define IN_DQ11        4 //
#define IN_DQ12       12 //
#define IN_DQ13       13 //
#define IN_DQ14       10 //
#define IN_DQ15       11 //

//#define CE              // gnd
#define OE             8 //
#define BYTE          21
#define WP            22
#define RST           23
#define WE            27
#define RYBY          24


/*
 * END USER DEFINITIONS
 */

 unsigned char input1 = 0;
 unsigned char input2 = 0;
 unsigned char input[2];


// assuming LSB shifted out first
void shiftOut(int myDataPin, int myClockPin, char byte) {
        int i; // cant use unsigned or byte otherwise loop never ends

        pinMode(myClockPin, OUTPUT);
        pinMode(myDataPin, OUTPUT);

        char currentbit;
          for (i=0; i<=7; i++) {
            // byte 0b11001011
            currentbit = (byte       & (1 << i           ))            >>   i;
            // i=0       (0b11001011 & (1 << 0=0b00000001))=0b00000001 >>   0  =  1
            // i=1       (0b11001011 & (1 << 1=0b00000010))=0b00000010 >>   1  =  1
            // i=2       (0b11001011 & (1 << 2=0b00000100))=0b00000000 >>   2  =  0
            // i=3       (0b11001011 & (1 << 3=0b00001000))=0b00001000 >>   3  =  1
            digitalWrite(myClockPin, 0);
            delayMicroseconds(2);

            digitalWrite(myDataPin, currentbit);

            digitalWrite(myClockPin, 1);
            delayMicroseconds(2);
          }
}


void sendDataOut(unsigned long value) {
  digitalWrite(ADDR_STROBE, LOW);

  unsigned char byte1 =  value &   0x0000FF;
  unsigned char byte2 = (value &   0x00FF00) >> 8;
  unsigned char byte3 = (value &   0xFF0000) >> 16;
  unsigned char a24   = (value & 0x01000000) >> 24;

  // shift out assumes LSB first
  shiftOut(ADDR_DATAOUT, ADDR_CLOCK, byte1 ); //first byte
  shiftOut(ADDR_DATAOUT, ADDR_CLOCK, byte2  ); //second byte
  shiftOut(ADDR_DATAOUT, ADDR_CLOCK, byte3 ); //third byte
  digitalWrite(ADDR_A24, a24);

  // LATCH/STROBE:
  digitalWrite(ADDR_STROBE, HIGH);
  delayMicroseconds(2);
  digitalWrite(ADDR_STROBE, LOW);
}


int main(int argc, char const *argv[]) {
  FILE* file;
  if (argc != 2) {
    printf("provide dump file as argument\n");
    return 1;
  }

  file = fopen(argv[1], "wb");

  //chech if wiringPi build in successfully
  if (wiringPiSetup () == -1)
    return 1 ;

    pinMode(OE, OUTPUT);
    pinMode(WE, OUTPUT);
    pinMode(WP, OUTPUT);
    pinMode(BYTE, OUTPUT);
    pinMode(RST, OUTPUT);
    pinMode(RYBY, INPUT);
    //digitalWrite(CE, LOW); // grounded
    digitalWrite(OE, LOW);
    digitalWrite(WE, HIGH); // Not care?
    digitalWrite(WP, LOW); // Not care?
    digitalWrite(BYTE, HIGH); // LOW 8bit mode HIGH 16 bit mode
    digitalWrite(RST, HIGH); //

    pinMode(ADDR_STROBE, OUTPUT);
    pinMode(ADDR_DATAOUT, OUTPUT);
    pinMode(ADDR_CLOCK, OUTPUT);
    pinMode(ADDR_A24, OUTPUT);
    pinMode(IN_DQ00, INPUT);
    pinMode(IN_DQ01, INPUT);
    pinMode(IN_DQ02, INPUT);
    pinMode(IN_DQ03, INPUT);
    pinMode(IN_DQ04, INPUT);
    pinMode(IN_DQ05, INPUT);
    pinMode(IN_DQ06, INPUT);
    pinMode(IN_DQ07, INPUT);
    pinMode(IN_DQ08, INPUT);
    pinMode(IN_DQ09, INPUT);
    pinMode(IN_DQ10, INPUT);
    pinMode(IN_DQ11, INPUT);
    pinMode(IN_DQ12, INPUT);
    pinMode(IN_DQ13, INPUT);
    pinMode(IN_DQ14, INPUT);
    pinMode(IN_DQ15, INPUT);

  //
  // OUTPUT (address)
  //
  pinMode(ADDR_STROBE, OUTPUT);
  pinMode(ADDR_CLOCK,  OUTPUT);
  pinMode(ADDR_DATAOUT, OUTPUT);

  //unsigned long addr = 0xFFFFF0; // debug
  unsigned long addr = 0x000000;

  while (addr < MAX_ADDR) {
      delayMicroseconds(4);
      //
      // SEND OUTPUT (address)
      //
      sendDataOut(addr);
      //addr += 0x1000; // debug
      addr += 1;
      //
      // READ INPUT (data)
      //

      input1 = 0;
      input2 = 0;
      input1 = digitalRead(IN_DQ00);
      input1 = input1 | (digitalRead(IN_DQ01) << 1);
      input1 = input1 | (digitalRead(IN_DQ02) << 2);
      input1 = input1 | (digitalRead(IN_DQ03) << 3);
      input1 = input1 | (digitalRead(IN_DQ04) << 4);
      input1 = input1 | (digitalRead(IN_DQ05) << 5);
      input1 = input1 | (digitalRead(IN_DQ06) << 6);
      input1 = input1 | (digitalRead(IN_DQ07) << 7);
      input2 = digitalRead(IN_DQ08);
      input2 = input2 | (digitalRead(IN_DQ09) << 1);
      input2 = input2 | (digitalRead(IN_DQ10) << 2);
      input2 = input2 | (digitalRead(IN_DQ11) << 3);
      input2 = input2 | (digitalRead(IN_DQ12) << 4);
      input2 = input2 | (digitalRead(IN_DQ13) << 5);
      input2 = input2 | (digitalRead(IN_DQ14) << 6);
      input2 = input2 | (digitalRead(IN_DQ15) << 7);


      //
      // SAVE/PRINT DATA OUT
      //
      // Reverse the order if needbe:
      //printf("%c", input1);
      //printf("%c", input2);
      if (fputc(input1, file) == EOF || fputc(input2, file) == EOF) {
         printf("error at addr 0x%x\n", addr);
	 return 1;
      }


  }
  return 0;

}
