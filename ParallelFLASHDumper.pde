/* 
 Parallel FLASH Dumper (v0.2 20101015)
 There are many commercial FLASH memory programmers but their out of box 
 support for different FLASH chips can be a problem, or I should say 
 some FLASH chips just are not supported.  The meaning of "Parallel" in
 this context means we will be connecting directly to the full address 
 and databus of the target using shift in and out registers.  At the 
 moment only read (hence dump) functionality is supported.   

 Further documentation: http://deadhacker.com/tools/

 SETUP:
 First, the timing of when to throw Chip Select, Write Select or other
 lines might differ from your chip.  You can try the default here but
 you will likely need to modify.  For this you need to consult the 
 timing diagrams of your target.  Assuming that is correct...
 
 1. Connect "parallel out" shift registers to address pins of target 
 2. Connect "parallel out" pins to arduino:
      ADDR_STROBE   = 8;  // latch orange (color shown in fz)
      ADDR_DATAOUT  = 7;  // green
      ADDR_CLOCK    = 6;  // yellow
 3. Connect "parallel in" shift registers to data out pins of target
 4. Connect "parallel in" pins to arduino:
      IN_STROBE     = 4; // latch orange
      IN_DATAIN     = 3; // green
      IN_CLOCK      = 2; // yellow  
 5. Define MAX_ADDR. Code with dump address 0 to MAX_ADDR

 USAGE: 
 1. Load code to microcontroller, code will start dumping raw
    data within a second or two.
 2. Open serial terminal with baud rate 230400. Data will be 
    dumped in raw binary form over serial and might take a 
    couple hours. 
    Example using `screen`:
      screen -l /dev/ttysUSB0 230400 > flashdump.raw
    To monitor progres open in tail through a hex editor:
      tail -f flashdump.raw | xxd
    You can also put some LED's on the address lines for visual
    status.
  
 AUTHORS & CODE BRANCHES:
 cyphunk  http://github.com/cyphunk/ParallelFLASHDumper
 
 DEVELOPER NOTES:
 In the future with write support SFI could be used.  U-boot can 
 be used as a reference to help. I would advocate using SFI when 
 it is supported but in some cases there is no serial interface 
 available for this and will require setting up a parallel 
 conenction anyway.  The advantage of using this software is that
 only the read logic is required when all you care to do is dump
 the flash. That and, its cheap.
 I tried to speed things up by using a SD card but this turned out
 to be slower than serial actually.  Perhaps there are ways to
 improve such a setup though.

 TROUBLESHOOTING:
 Various issues and solutions I've run into
  
 * check data lines to determine proper byte order
 * check address lines to determine proper bit order (MSB or LSB).
   leds can help
 * check solder connection to pins.  Sometimes a multimeter check 
   is enough to press the pin to its pad while checking continuity.
   Hence, you might think it is connected with a continuity test 
   when it isnt.  In such cases you will only get a hint of this 
   when scrutinizing the address in the dump, to see where data 
   out might duplicate. (where address bits aren't registering)

 TROUBLESHOTTING EXAMPLES:
 a log of human error from various projects
 
 * Vss, CE#, OE# of chip were shorted at pin N of dual (word) 
   shift reg. I was able to find this because target dump had
   a section of memory with all FFFF's that were being read 
   as 7FFF due to the grounded pin N.  This happened because 
   I got my orientation of the shift regs upside down.
 * The target chip itself was plugged up wrong due to mixing up
   the orientation with address being connected to data and 
   data to address.  The result was some odd behavior and output
   that looked too constant.
 * Address pin 1 of target was not properly soldered to the shift
   reg input.  This resulted in the board seeing address 00001101
   when I/microcontroller was sending 00001111. I was able to 
   notice this because every 4 bytes repeated once in the dump.
 * The bit order of the ShiftOut for the address shiftregs was 
   LSB when it should have been MSB. I only noticed this after 
   plugging up LED's to each addr lineon the hunch that this 
   might be an issue.
   
   
 This code is public domain, abuse as you wish and at your own risk
*/ 


/*
 * BEGIN USER DEFINITIONS
 */
int ADDR_STROBE   = 8;  // latch orange
int ADDR_DATAOUT    = 7;  // green
int ADDR_CLOCK      = 6;  // yellow

// Data in pins
// connect to corisponding pins on data shift registers
int IN_STROBE       = 4; // latch orange
int IN_DATAIN       = 3; // green
int IN_CLOCK        = 2; // yellow

// define according to the size of your target chip
uint32_t MAX_ADDR   = 0xFFFFFF;

/*
 * END USER DEFINITIONS
 */




byte input1 = 0;
byte input2 = 0;
byte input[2];

// Both buffers must be powers of 2, up to 128 bytes
// Keep in mind the memory limits of the MCU!!
#define USART_RX_BUFFER_SIZE   8
#define USART_TX_BUFFER_SIZE   32
//#define USART_RX_BUFFER_SIZE   16
//#define USART_TX_BUFFER_SIZE   32
// The above are both AVR specific.  Comment out if you are using
// a different microcontroller
void setup (void) {
        //
        // INPUT (data)
        //
        pinMode(IN_STROBE, OUTPUT);
        pinMode(IN_CLOCK,  OUTPUT);
        pinMode(IN_DATAIN, INPUT);

        //
        // OUTPUT (address)
        //
        pinMode(ADDR_STROBE, OUTPUT);
        pinMode(ADDR_CLOCK,  OUTPUT);
        pinMode(ADDR_DATAOUT, OUTPUT);

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
}

// Currently AVR dependent
void tx( byte data) { 
   // src: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=83501&start=0
   while ( !( UCSR0A & (1<<UDRE0)) ) ;      // Wait for empty transmit buffer 
   UDR0 = data;                             // Put data in buffer and send it 
   // Comment out above and uncomment below for non-AVR microcontrollers: 
   //Serial.print(data,BYTE);
} 

void printbin (int out) {
        int j;
        for (j=7; j>=0; j--) {
                if (out & (1 << j)) {
                        tx(49); // "1"
                }
                else {
                        tx(48); // "0"
                }
        }
}
//unsigned long addr = 0xFFFFF0; // debug
unsigned long addr = 0x000000;

void loop (void) {
        delayMicroseconds(4);
        //
        // SEND OUTPUT (address)
        //
        if (addr > MAX_ADDR) {
                delay(1000000);
        }
        else {
                sendDataOut(addr);
                //addr += 0x1000; // debug
                addr += 1;

        //
        // READ INPUT (data)
        //
                digitalWrite(IN_CLOCK,0);
                digitalWrite(IN_STROBE,1);
                digitalWrite(IN_CLOCK,1); // important
                digitalWrite(IN_STROBE,0);
                                  
                input1 = shiftIn(IN_DATAIN, IN_CLOCK);
                input2 = shiftIn(IN_DATAIN, IN_CLOCK);
        // 
        // PRINT TO SERIAL
        //
                // Reverse the order if needbe:
                //Serial.print(input1, BYTE);
                //Serial.print(input2, BYTE);
                tx(input1); 
                tx(input2);
                
                // Debug
                /*
                printbin(addr-1);
                Serial.print(":\t");
                Serial.print(input1,HEX);
                Serial.print("\t");
                printbin(input1);
                Serial.print(" ");
                if (input1 > 32 && input1 < 126) {  Serial.print(input1,BYTE); }
                Serial.print("\t");
                Serial.print(input2,HEX);
                Serial.print("\t");
                printbin(input2);
                Serial.print(" ");
                if (input2 > 32 && input2 < 126) {  Serial.print(input2,BYTE); }
                Serial.println();
                */
        }


}

///// ----------------------------------------shiftIn function
///// just needs the location of the data pin and the clock pin
///// it returns a byte with each bit in the byte corresponding
///// to a pin on the shift register. leftBit 7 = Pin 7 / Bit 0= Pin 0

byte shiftIn(int myDataPin, int myClockPin) { 
        int i; // cant use unsigned or byte otherwise loop never ends
        byte temp = 0;
        byte pinState;
        byte myDataIn = 0;

        pinMode(myClockPin, OUTPUT);
        pinMode(myDataPin, INPUT);

        //we will be holding the clock pin high 8 times (0,..,7) at the
        //end of each time through the for loop

        //at the begining of each loop when we set the clock low, it will
        //be doing the necessary low to high drop to cause the shift
        //register's DataPin to change state based on the value
        //of the next bit in its serial information flow.
        //The register transmits the information about the register pins 
        //from pin 7 to pin 0 so that is why our function counts down
        for (i=7; i>=0; i--)
        {
                digitalWrite(myClockPin, 0);
                delayMicroseconds(2);

                 temp = digitalRead(myDataPin);
                if (temp) { //temp == 1
                        pinState = 1;
                        myDataIn = myDataIn | (1 << i);
                }
                else {  //temp == 0 ,no need to do anything. myDataIn is already 0
                        //turn it off -- only necessary for debuging
                        pinState = 0;
                }
                digitalWrite(myClockPin, 1);
                delayMicroseconds(2);

                //DEBUG:
                //Serial.print(pinState);
                //Serial.print("     i: ");
                //Serial.print(i,DEC);
                //Serial.print("  ");
                //Serial.println (myDataIn, BIN);


        }
        //DEBUG, print statements whitespace
        //Serial.println();
        //Serial.println(myDataIn, BIN);
        return myDataIn;
}

void sendDataOut(unsigned long value) {
        digitalWrite(ADDR_STROBE, LOW);

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

        // LATCH/STROBE:
        digitalWrite(ADDR_STROBE, HIGH);
        delayMicroseconds(2);
        digitalWrite(ADDR_STROBE, LOW);
}
