/*
Based on ParallelFLASHDumper but this version doesn't use any
shift registers. Meaning all Data, Address and state pins 
connect directly to microcontroller (Teensy++ in our case).

On host:
stty -F /dev/ttyACM* 230400 -icrnl -imaxbel -opost -onlcr -isig -icanon -echo && \
cat /dev/ttyACM* > dump
tail -f dump | xxd -c 44 


About Pins on ST M36W0R604 (with a M58WR064HT FLASH chip inside):

Lf = Latch Enable = RISING EDGE
Latch Enable latches the address bits on its rising edge. The address latch is transparent
when Latch Enable is at V IL and it is inhibited when Latch Enable is at V IH .

RP = RESET = LOW
When Reset is at V IH , the device is in normal operation. Upon exiting reset
mode the device enters asynchronous read mode, but a negative transition of Chip Enable
or Latch Enable is required to ensure valid data outputs.

WA = WAIT = WAIT READY
Wait is an output signal used during synchronous read to indicate whether the data on the
output bus is valid.

Gf = Output Enable = LOW
The Output Enable input controls data outputs during the bus read operation of the memory.

VddQ supplies voltage to IO. Can be tied to VDDF (main Vcc). In our case we attached our
1.8v supply voltage to both Vddq and Vddf



Bus read
Bus read operations output the contents of the memory array, the electronic signature, the
Status Register and the common Flash interface. Both Chip Enable and Output Enable must
be at V IL to perform a read operation. The Chip Enable input should be used to enable the
device. Output Enable should be used to gate data onto the output. The data read depends
on the previous command written to the memory (see command interface section). See
Figures 9, 10, 11 and 12 Read AC Waveforms, and Tables 22 and 23 Read AC
Characteristics for details of when the output becomes valid.
*/




/*
 * BEGIN USER DEFINITIONS
 */

byte addr_pins[]  = {
/* bit pos. */
/*  0 - 7   */    PIN_A3, PIN_A1, PIN_A2, PIN_C3, PIN_C7, PIN_C4, PIN_A0, PIN_C1,
/*  8 - 15  */    PIN_A7, PIN_E0, PIN_E1, PIN_D4, PIN_D6, PIN_D7, PIN_A6, PIN_A5,
/* 16 - 21  */    PIN_A4, PIN_C2, PIN_C6, PIN_C5, PIN_C0, PIN_D5
};
byte data_pins[]  = {
/* bit pos. */
/*  0 - 7   */    PIN_F7, PIN_F6, PIN_F2, PIN_F1, PIN_B1, PIN_E7, PIN_B5, PIN_B4,
/*  8 - 15  */    PIN_F3, PIN_F5, PIN_F0, PIN_F4, PIN_B2, PIN_B0, PIN_B3, PIN_B6,
};

#define LF        PIN_D1 // ADDR_STROBE - address latched on rising edge
#define RP        PIN_D0 // RESET - HIGH normal operation
#define WA        PIN_B7 // WAIT READY indicator
#define GF        PIN_D2 // OE Output Enabled
//#define CE             // externally tied to GND

// define according to the size of your target chip M58WR064HT
// 64 megabits = 8MB/8000000B = 0x7a1200 / words bus width = 0x3d0900
//uint32_t MAX_ADDR = (64000000/8/2); 

uint32_t MAX_ADDR = 0x3FFFFF;

// DEBUG:
//uint32_t MAX_ADDR = 0xFF; 
//uint32_t MAX_ADDR = (0x1fb*3); // BUG Hunting


/*
 * END USER DEFINITIONS
 */





const byte addr_pins_len = sizeof(addr_pins)/sizeof(addr_pins[0]);
const byte data_pins_len = sizeof(data_pins)/sizeof(data_pins[0]);

void setup (void) {
        Serial.begin(230400);

        for (int i = 0; i < addr_pins_len; i++)
            pinMode(addr_pins[i], OUTPUT);
        for (int i = 0; i < data_pins_len; i++) {
            pinMode(data_pins[i], INPUT ); // INPUT_PULLUP wasn't a good idea
        }

        pinMode(LF, OUTPUT); // ADDR_STROBE - address latched on rising edge
        pinMode(GF, OUTPUT); // OE Output Enabled
        pinMode(RP, OUTPUT); // RESET
        pinMode(WA, INPUT);  // WAIT READY indicator

        /* RESET: Upon exiting reset mode the device enters asynchronous
        read mode, but a negative transition of Chip Enable or Latch Enable
        is required to ensure valid data outputs. */
        digitalWrite(LF, HIGH);
        digitalWrite(GF, HIGH);
        digitalWrite(RP, HIGH); // HIGH for normal operation. Device now in async read mode
        delayMicroseconds(1); // not certain needed or not
        digitalWrite(LF, LOW);
        digitalWrite(GF, LOW);

        // give user time to connect up serial monitor before spitting data out
        delay(4000);
}

void printbin (int out) {
        int j;
        for (j=7; j>=0; j--) {
                if (out & (1 << j)) {
                        Serial.print(49, BYTE); // "1"
                }
                else {
                        Serial.print(48, BYTE); // "0"
                }
        }
}
//unsigned long addr = 0xFFFFF0; // debug
unsigned long addr = 0x000000;
unsigned int input = 0;
byte input1;
byte input2;

void loop (void) {
//        delayMicroseconds(4);
        //
        // SEND OUTPUT (address)
        //
        if (addr > MAX_ADDR) {
                delay(1000000);
                // Debug:
                //delay(1000);
                //addr=0;
        }
        else {
                sendDataOut(addr);
                addr += 1;
                // Debug:
                //addr += 0x800;

        //
        // READ INPUT (data)
        //
                input  = readDataIn();
                input1 =  input & 0xFF;
                input2 = (input & 0xFF00) >> 8;
        //
        // PRINT TO SERIAL
        //
                // Reverse the order if needbe:
                 Serial.print(input1, BYTE);
                 Serial.print(input2, BYTE);

                // Debug:
//                Serial.print("addr:");
//                printbin((addr-1&0xff0000)>>16);
//                printbin((addr-1&0xff00)>>8);
//                printbin( addr-1&0xff);
//                Serial.print(":");
//                Serial.print((addr-1&0xff0000)>>16, HEX);
//                Serial.print((addr-1&0xff00)>>8, HEX);
//                Serial.print( addr-1&0xff, HEX);
//                Serial.print(":\t");
//                Serial.print(input1,HEX);
//                Serial.print("\t");
//                printbin(input1);
//                Serial.print("\t");
//                if (input1 > 32 && input1 < 126) {  Serial.print(input1,BYTE); } else { Serial.print(" "); }
//                Serial.print("\t");
//                Serial.print(input2,HEX);
//                Serial.print("\t");
//                printbin(input2);
//                Serial.print("\t");
//                if (input2 > 32 && input2 < 126) {  Serial.print(input2,BYTE); } else { Serial.print(" "); }
//                Serial.println();
        }


}


void sendDataOut(unsigned long value) {
        digitalWrite(LF, LOW);

        // LSB
//        Serial.print("ADDR:");
//        Serial.println(value,HEX);
        for (int i = 0; i < addr_pins_len; i++) {
// Useful to debug if software is sending out address:
//          Serial.print("addr:");
//          Serial.print(i);
//          Serial.print(":");
          // Critical BUG: need 1UL otherwise expression assumes using ints
          if ((value & (1UL << i))>>i) {
             digitalWrite(addr_pins[i], 1);
//             Serial.println("1");
          }
          else {
             digitalWrite(addr_pins[i], 0);
//             Serial.println("0");
          }
        }

        // LATCH/STROBE:
        digitalWrite(LF, HIGH);
}

unsigned int readDataIn() {
//    delayMicroseconds(4);

    byte temp = 0;
    byte pinState;
    unsigned int myDataIn = 0;

    // LSB
    for (int i = 0; i < data_pins_len; i++) {
      
        temp = digitalRead(data_pins[i]);
        if (temp) { //temp == 1
                myDataIn = myDataIn | (1 << i);
        }
        //DEBUG:
//        if (i==15 || i==14) {
//        Serial.print("pin: ");
//        Serial.print(i,DEC);
//        Serial.print("  value:");
//        Serial.print(temp);
//        Serial.print("  buffer:");
//        Serial.print((myDataIn&0x0000ff00)>>8, HEX);
//        Serial.print(myDataIn&0x000000ff, HEX);
//        Serial.print("  ");
//        printbin((myDataIn&0x00ff0000)>>16);
//                Serial.print(" ");
//        printbin((myDataIn&0x0000ff00)>>8);
//                Serial.print(" ");
//        printbin(myDataIn&0x000000ff);
//        Serial.print("  ");
//        Serial.print((1<<data_pins_len)-1);
//                Serial.print("  ");
//        Serial.print(myDataIn & (1<<data_pins_len)-1, HEX);
//        Serial.println();
//        }

    }
    //DEBUG, print statements whitespace
//    Serial.println();
//    Serial.println(myDataIn, BIN);
    return myDataIn;

}
