/* PINOUT used:
 *
 * Databus
 *  PORTpin A0 - A7 - Cerberus XD0 - XD7
 *
 * Addressbus
 *  PORTpin C0 - C7 - Cerberus XA0 - XA7
 *  PORTpin L0 - L7 - Cerberus XA8 - XA15
 *
 * Control pins
 *  PORTpin B0 - Cerberus XRD
 *  PORTpin B1 - Cerberus XWR
 *  PORTpin B2 - Cerberus XBUSACK
 *  PORTpin B3 - Cerberus XBUSREQ
 *  PORTpin G0 - Cerberus XE
 *  PORTpin E4 - Cerberus XSLC - IRQ capable pin on ATMEGA - not used in this code
 *  PORTpin E5 - Cerberus XIRQ - IRQ capable pin on ATMEGA - not used in this code
 *  PORTpin D2 - Cerberus XIN - IRQ capable pin on ATMEGA - not used in this code
 *  PORTpin D3 - Cerberus XCLK - IRQ capable pin on ATMEGA - not used in this code
 */

#define XRDBIT      0
#define XWRBIT      1
#define XBUSACKBIT  2
#define XBUSREQBIT  3
#define XEBIT       0
#define XSLCBIT     4
#define XIRQBIT     5
#define XINBIT      2
#define XCLKBIT     3

#define SCREENSIZE    30*40
#define SCREENSTART   0xf800
#define CHARDEFSIZE   2048
#define CHARDEFSTART  0xf000

void setup_controlpins(void) {
  DDRB  |= (1 << XRDBIT) | (1 << XWRBIT) | (1 << XBUSREQBIT); // output pins
  PORTB |= (1 << XRDBIT) | (1 << XWRBIT) | (1 << XBUSREQBIT); // output high

  DDRB  &=~(1 << XBUSACKBIT); // XBUSACK in input mode
  PORTB |= (1 << XBUSACKBIT); // XBUSACK pull-up
  
  DDRG  |= (1 << XEBIT);      // XE in output mode
  PORTG |= (1 << XEBIT);      // XE output high

  DDRE  |= (1 << XIRQBIT);    // XIRQ in output mode
  PORTE |= (1 << XIRQBIT);    // XIRQ output high

  DDRE  &= ~(1 << XSLCBIT);   // XSLC in input mode
  PORTE |= (1 << XSLCBIT);    // XSLC pull-up

  DDRD  |= (1 << XINBIT);     // XIN in output mode
  PORTD |= (1 << XINBIT);     // XIN output high

  DDRD  &= ~(1 << XCLKBIT);   // XCLK in input mode
  PORTD |= (1 << XCLKBIT);    // XCLK pull-up
}

void databus_mode(uint8_t mode) {
  if(mode == OUTPUT) DDRA = 0b11111111;
  else DDRA = 0;
}

void addressbus_mode(uint8_t mode) {
  if(mode == OUTPUT) {
    DDRC = 0b11111111;
    DDRL = 0b11111111;
  }
  else {
    DDRC = 0;
    DDRL = 0;
    PORTC = 0xff; // internal pull-up for all pins
    PORTL = 0xff; 
  }
}

void bus_acquire(void) {
  PORTB &= ~(1 << XBUSREQBIT); // XBUSREQ to 0, active low
  while(PINB & (1 << XBUSACKBIT)); // Wait for XBUSACK to go low
  PORTG &= ~(1 << XEBIT); // XE to 0, active low, take over bus
  delayMicroseconds(1); // wait for bus stabilization
}

void bus_release(void) {
  PORTB |= (1 << XRDBIT); // XRD to 1, inactive high - should have been set by bus_read already
  PORTB |= (1 << XWRBIT); // XWR to 1, inactive high - should have been set by bus_write already
  PORTG |= (1 << XEBIT); // XE to 1, inactive high
  PORTB |= (1 << XBUSREQBIT); // XBUSREQ to 1, inactive high
}

void bus_write(uint16_t address, uint8_t data) {
  databus_mode(OUTPUT);
  PORTA = data;
  PORTC = address & 0xff; // LSB
  PORTL = (address >> 8) & 0xff; // MSB
  PORTB &= ~(1 << XWRBIT); // XWR to 0, active low
  //delayMicroseconds(1); // wait for memory write cycle to complete
  delayMicroseconds(10);
  PORTB |= (1 << XWRBIT);
}

uint8_t bus_read(uint16_t address) {
  uint8_t retval;
  databus_mode(INPUT);
  PORTC = address & 0xff; // LSB
  PORTL = (address >> 8) & 0xff; // MSB
  //delayMicroseconds(100);
  PORTB &= ~(1 << XRDBIT); // XRD to 0, active low
  delayMicroseconds(50); // wait for bus stabilization and memory cycle
  //delayMicroseconds(200);
  retval = PINA;
  PORTB |= (1 << XRDBIT);
  return retval;
}

// Send a byte as Intel Hex data (ascii hex value with two digits)
void sendHxByte(uint8_t data) {
  char buffer[3];

  sprintf(buffer, "%02X", data);
  Serial.print(buffer);
}

// Receive a single Nibble from the incoming Intel Hex data
uint8_t getHxNibble(void) {
  uint8_t c ,val;

  c = 0;
  while(c == 0)
  {
    if(Serial.available() > 0) c = toupper(Serial.read());
  }
  
  if((c >= '0') && c <='9') val = c - '0';
  else val = c - 'A' + 10;
  // illegal characters will be dealt with by checksum later
  return val;
}

// Receive a byte from the incoming Intel Hex data
// as two combined nibbles
uint8_t getHxByte(void) {
  uint8_t val = 0;

  val = getHxNibble() << 4;
  val |= getHxNibble();

  return val;  
}

void echo_checksum(uint8_t hxchecksum)
{
  // local echo status to the user
  if(hxchecksum) Serial.print("X");
  else Serial.print(".");
}

void hexdump(uint8_t *buffer, uint16_t address, uint16_t count) {
  uint8_t h,l;
  uint8_t hxchecksum;
  uint8_t recordbytecount;

  while(count) {
    if(count > 16) recordbytecount = 16;
    else recordbytecount = count;
    count -= recordbytecount;
    //Serial.println(recordbytecount);
    h = address >> 8;
    l = address & 0xff;
    address += recordbytecount;
    hxchecksum = recordbytecount + h + l;
    Serial.print(":");
    sendHxByte(recordbytecount);
    sendHxByte(h);
    sendHxByte(l);
    sendHxByte(0); // Data record
    while(recordbytecount--) {
      sendHxByte(*buffer);
      hxchecksum += *buffer;
      buffer++;
    }
    hxchecksum = ~(hxchecksum) + 1;
    sendHxByte(hxchecksum);
    Serial.println();
  }
}
void hexdump_sendEOF(void) {
  // send termination record
  Serial.print(":");
  sendHxByte(0);
  sendHxByte(0);
  sendHxByte(0);
  sendHxByte(1); // record type 1 - end of file
  sendHxByte(0xff);
  Serial.println();
}

// Hexload engine
//
void hexload(void)
{
  uint16_t address;
  uint8_t h,l;
  uint8_t count;
  uint8_t record;
  uint8_t data;
  uint8_t hxchecksum;

  bool done,firstrecord;
  uint16_t errors;
  
  errors = 0;
  done = false;
  firstrecord = true;
  while(!done) {
    data = 0;
    // hunt for start of record
    while(data != ':') if(Serial.available() > 0) data = Serial.read();
    if(firstrecord) Serial.println("Receiving Intel HEX records");
    firstrecord = false;
    bus_acquire();

    count = getHxByte();  // number of bytes in this record
    h = getHxByte();      // middle byte of address
    l = getHxByte();      // lower byte of address 
    address = (h << 8) | l;
    record = getHxByte(); // record type

    hxchecksum = count + h + l + record;  // init control checksum

    switch(record)
    {
      case 0: // data record
        while(count--)
        {
          data = getHxByte();
          bus_write(address++, data);
          hxchecksum += data;   // update hxchecksum
        }
        hxchecksum += getHxByte();  // finalize checksum with actual checksum byte in record, total 0 if no errors
        if(hxchecksum) errors++;
        //echo_checksum(hxchecksum);
        break;
      case 1: // end of file record
        getHxByte();
        done = true;
        break;
      default:// ignore other (non I32Hex) records
        break;
    }
    bus_release();
  }
  Serial.print(errors);
  Serial.println(" errors(s)");
}

void setup() {
  addressbus_mode(OUTPUT);
  setup_controlpins();

  // pushbutton screencapture
  DDRK  &= ~(1 << 0); // K0 input
  PORTK |= (1 << 0);  // K0 pull-up

  Serial.begin(115200);
}

void captureScreen() {
  char screenbuffer[SCREENSIZE];
  char charbuffer[CHARDEFSIZE];
  uint16_t count;
  uint16_t address;
  char *ptr;

  bus_acquire();
  address = CHARDEFSTART;
  count = CHARDEFSIZE;
  ptr = charbuffer;
  while(count--) *(ptr++) = bus_read(address++);
  address = SCREENSTART;
  count = SCREENSIZE;
  ptr = screenbuffer;
  while(count--) *(ptr++) = bus_read(address++);
  bus_release();

  hexdump(charbuffer, CHARDEFSTART, CHARDEFSIZE);
  hexdump(screenbuffer, SCREENSTART, SCREENSIZE);
  hexdump_sendEOF();

  delay(250);
}

void waitMessage(void) {
  Serial.println("Waiting for Intel Hex file / Button push");
}

void loop() {
  uint8_t val;
  waitMessage();

  while(true) {
    val = PINK & (1 << 0);
    if(val == 0) {
      captureScreen();
      waitMessage();
    }
    if((Serial.available() > 0) && (Serial.peek() != ':')) Serial.read(); // discard non-iHex crap
    if((Serial.available() > 0) && (Serial.peek() == ':')) {
      hexload();
      waitMessage();
    }
  }
}

/*
void loop() {
  uint16_t errors = 0;
  uint8_t temp;
  uint16_t address;
  uint8_t data;

  Serial.println("Starting 32KB run");
  data = 0;
  bus_acquire();
  for(address = 0x205; address < 33285; address++) bus_write(address, data++);

  Serial.println("Start verificaton");
  data = 0;
  for(address = 0x205; address < 33285; address++) {
    temp = bus_read(address);
    if(temp != data) {
      Serial.print("Error at address ");
      Serial.print(address);
      Serial.print(" - ");
      Serial.println(temp);
      errors++;
    }
    data++;
  }  
  if(errors) {
    Serial.print(errors);
    Serial.println(" ERRORS");
  }
  else Serial.println("Verify complete");
  
  bus_release();
  Serial.println("Done");
  while(1);
}
*/