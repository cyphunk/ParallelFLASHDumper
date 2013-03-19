 Parallel FLASH Dumper (v0.2.1 20130319)
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
    Example dump from console (unix/linux):
      stty -f /dev/tty.usb* 230400; cat /dev/cu.usb* > dump
    This will dump raw data to dump
    To monitor progres open in tail through a hex editor:
      tail -f dump | xxd
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
 About serial throughput issues on Arduino:
 http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1281611592/0

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
   
   
 This code is public domain, [ab]use as you wish and at your own risk
