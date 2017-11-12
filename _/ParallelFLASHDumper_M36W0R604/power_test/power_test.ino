// COPY OF SKETCH BUT WE USE THIS TO CHECK POWER

#define OUTPIN PIN_F0
#define INPIN  PIN_B0
#define NULLPIN PIN_B1 // not in use, use for state pins which are ignored

/*
About Pins on ST M36W0R604 (with a M58WR064HT FLASH chip)

Lf = Latch Enable = RISING EDGE
Latch Enable latches the address bits on its rising edge. The address latch is transparent
when Latch Enable is at V IL and it is inhibited when Latch Enable is at V IH .

RP = RESET = LOW
When Reset is at V IH , the device is in normal operation. Upon exiting reset
mode the device enters asynchronous read mode, but a negative transition of Chip Enable
or Latch Enable is required to ensure valid data outputs.

WA = WAITf = WAIT READY
Wait is an output signal used during synchronous read to indicate whether the data on the
output bus is valid.

Gf = Output Enable =
The Output Enable input controls data outputs during the bus read operation of the memory.

VddQ supplies voltage to IO. Can be tied to VDDF (main Vcc)



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
//
byte addr_pins[]  = {
/* bit pos. */
/*  0 - 7   */    OUTPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN,
/*  8 - 15  */    NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN,
/* 16 - 21  */    NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN
};
byte data_pins[]  = {
/* bit pos. */
/*  0 - 7   */    INPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN,
/*  8 - 15  */    NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN, NULLPIN,
};

#define LF        NULLPIN // ADDR_STROBE - address latched on rising edge
#define RP        NULLPIN // RESET
#define WA        NULLPIN // WAIT READY indicator
#define GF        NULLPIN // OE Output Enabled
//#define CE              // gnd

// define according to the size of your target chip M58WR064HT
// 64 megabits = 8MB/8000000B = 0x7a1200 / words bus width = 0x3d0900
uint32_t MAX_ADDR = (64000000/8/2); 

// DEBUG:
//uint32_t MAX_ADDR = 0xFF; 

/*
 * END USER DEFINITIONS
 */
const byte addr_pins_len = sizeof(addr_pins)/sizeof(addr_pins[0]);
const byte data_pins_len = sizeof(data_pins)/sizeof(data_pins[0]);




// Both buffers must be powers of 2, up to 128 bytes
// Keep in mind the memory limits of the MCU!!
// #define USART_RX_BUFFER_SIZE   8
// #define USART_TX_BUFFER_SIZE   32
//#define USART_RX_BUFFER_SIZE   16
//#define USART_TX_BUFFER_SIZE   32
// The above are both AVR specific.  Comment out if you are using
// a different microcontroller
void setup (void) {
        for (int i = 0; i < addr_pins_len; i++)
            pinMode(addr_pins[i], OUTPUT);
        for (int i = 0; i < data_pins_len; i++) {
            pinMode(data_pins[i], INPUT);
            digitalWrite(data_pins[i], LOW);
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
        digitalWrite(RP, LOW); // LOW for nornmal operation. Device now in async read mode
        delayMicroseconds(1); // not certain needed or not
        digitalWrite(LF, LOW);
        digitalWrite(GF, LOW);

        //Serial.begin(9600);
        // 230400 is the MAX i've been able to get with 16Mhz at 3.3v, and only when doubled from 115200
        //Serial.begin(115200);
        //UCSR0A |= 2; //double the baud to 230400 - doesnt seem to work
        Serial.begin(230400);
        //UCSR0A |= 2; //double the baud to 460800
        //Serial.begin(460800);
        //UCSR0A |= 2; //double the baud to 921600
        //Serial.begin(921600);
        //UCSR0A |= 2; //double the baud to 1843200

                delayMicroseconds(4);
                delay(2000);

}

// Currently AVR dependent
void tx( byte data) {
   // src: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=83501&start=0
   // while ( !( UCSR0A & (1<<UDRE0)) ) ;      // Wait for empty transmit buffer
   // UDR0 = data;                             // Put data in buffer and send it
   // Comment out above and uncomment below for non-AVR microcontrollers:
   Serial.print(data,BYTE);
}

void printbin (int out) {
        int j;
        for (j=7; j>=0; j--) {
                if (out & (1UL << j)) {
                        tx(49); // "1"
                }
                else {
                        tx(48); // "0"
                }
        }
}
//unsigned long addr = 0xFFFFF0; // debug
unsigned long addr = 0x000000;
unsigned int input = 0;
byte input1;
byte input2;

void loop (void) {
        delayMicroseconds(4);
        //
        // SEND OUTPUT (address)
        //
        if (addr > MAX_ADDR) {
                delay(1000000);
//                delay(4000);
//                addr=0;
        }
        else {
                sendDataOut(addr);
                //addr += 0x800; // debug
                addr += 1;

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
                // Serial.print(input1, BYTE);
                // Serial.print(input2, BYTE);
//                tx( input & 0xFF);
//                tx((input & 0xFF00) >> 8 );

                // Debug
                Serial.print("addr:");
                printbin((addr-1&0xff0000UL)>>16);
                printbin((addr-1&0xff00)>>8);
                printbin( addr-1&0xff);
                Serial.print(":");
                Serial.print((addr-1&0xff0000UL)>>16, HEX);
                Serial.print((addr-1&0xff00)>>8, HEX);
                Serial.print( addr-1&0xff, HEX);
                Serial.print(":\t");
                Serial.print(input1,HEX);
                Serial.print(" ");
                printbin(input1);
                Serial.print(" ");
                if (input1 > 32 && input1 < 126) {  Serial.print(input1,BYTE); }
                Serial.print("\t");
                Serial.print(input2,HEX);
                Serial.print(" ");
                printbin(input2);
                Serial.print(" ");
                if (input2 > 32 && input2 < 126) {  Serial.print(input2,BYTE); }
                Serial.println();
        }


}


void sendDataOut(unsigned long value) {
        digitalWrite(LF, LOW);

        /*
        byte byte1 =  value & 0x0000FF;
        byte byte2 = (value & 0x00FF00) >> 8;
        byte byte3 = (value & 0xFF0000) >> 16;
        // For LSB order (depends on how you connected the shift in regs):
        //shiftOut(ADDR_DATAOUT, ADDR_CLOCK, LSBFIRST, byte1 ); //first byte
        //shiftOut(ADDR_DATAOUT, ADDR_CLOCK, LSBFIRST, byte2  ); //second byte
        //shiftOut(ADDR_DATAOUT, ADDR_CLOCK, LSBFIRST, byte3 ); //third byte
        shiftOut(ADDR_DATAOUT, ADDR_CLOCK, MSBFIRST, byte1 ); //first byte
        shiftOut(ADDR_DATAOUT, ADDR_CLOCK, MSBFIRST, byte2  ); //second byte
        shiftOut(ADDR_DATAOUT, ADDR_CLOCK, MSBFIRST, byte3 ); //third byte
        */

        // LSB
        for (int i = 0; i < addr_pins_len; i++)
            digitalWrite(addr_pins[i], value & (1UL << i) );

        // LATCH/STROBE:
        digitalWrite(LF, HIGH);
}

unsigned int readDataIn() {
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

    }
    //DEBUG, print statements whitespace
//    Serial.println();
//    Serial.println(myDataIn, BIN);
    return myDataIn;

}
