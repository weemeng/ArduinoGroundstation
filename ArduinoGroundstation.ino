// RFM98W - FSK modulation
// Arduino SPI interface 
// Receiver side
// by Ng Wee Meng
// adapted from Dave Akerman's LoRaHandheld AVR

#include <string.h>
#include <ctype.h>
#include <SPI.h>
#include <SD.h>
//#include <interrupt.h>

// Pin Configuration
/*---------------------------------------------------*\
|                                                     |
|               Arduino  2(15) - RFM DIO0             |
|               Arduino 18     - RFM DIO3             |
|               Arduino 19 	   - RFM DIO4             |
|               Arduino  3(14) - RFM DIO5             |
|               Arduino 10(53) - RFM NSS              |
|               Arduino 11(51) - RFM MOSI             |
|               Arduino 12(50) - RFM MISO             |
|               Arduino 13(52) - RFM CLK              |
|                                                     |
\*---------------------------------------------------*/

// RFM98W FSK
int _slaveSelectPin = 39; 
int dio0 = 15;
int dio3 = 18;
int dio4 = 19;
int dio5 = 14;
boolean reading = 0;
int CurrentCount = 0, Bytes, state;  
byte currentMode = 0x09;
byte testmode;
//int validpreamble;
unsigned char Message[256] = {0};
int Payload_Count = 0, Payload_Fail_Count = 0;
int usethis = 0, used = 0;
int Image_Packet_Count_new = 0, Image_Packet_Count_curr = 0;
int CorrectPackets = 0, pseudo_CorrectPackets = 0;
//File Saving
File  myFile;
char *cstr=new char[8];
String filename=String();
double first=0;
boolean firstcount=0;
int filecount = 0;
int imagedone = 0;
int Endflag; //signals the Endflag of an image so as to prevent premature FFD9 counters
/*
String content = "";
char character;
unsigned long LastPacketAt=0;
unsigned long UpdateTimeAt=0;
unsigned long UpdateRSSIAt=0;
*/

#define REG_FIFO                    0x00 	//okay
#define REG_OPMODE                  0x01	//ever changing
#define REG_BITRATEMSB				0x02	//0x00 fixed
#define REG_BITRATELSB				0x03	//0x6B fixed
#define REG_FDEVMSB					0x04	//0x04 
#define REG_FDEVLSB					0x05	//0x00 
#define REG_FRFMSB					0x06	//0x6C
#define REG_FRFMID					0x07	//0x40
#define REG_FRFLSB					0x08	//0x00
//#define REG_PACONFIG				0x09 	//09, 0A, 0B are for transmitter

// for Receiver
#define REG_LNA						0x0C	//0x60 01100000 //0xC0 11000000
#define REG_RXCONFIG				0x0D	//0x00 00000000 00011111
#define REG_RSSICONFIG				0x0E	//0x03 00000011
#define REG_RSSICOLLISION			0x0F	//0x0A	dont need
#define REG_RSSITHRESH				0x10	//0xC8  dont need
#define REG_RSSIVALUE				0x11	//read
#define REG_RXBW					0x12	//0x11 00010001
#define REG_AFCBW					0x13	//read	dont need
#define REG_AFCFEI					0x1A	//0x00  dont need
#define REG_AFCMSB					0x1B	//auto
#define REG_AFCLSB					0x1C	//auto
#define REG_FEIMSB					0x1D	//read
#define REG_FEILSB					0x1E	//read
#define REG_PREAMBLEDETECT			0x1F	//0xC4 1-10-01010
#define REG_RXTIMEOUT1				0x20	//0x00
#define REG_RXTIMEOUT2				0x21	//0x00
#define REG_RXTIMEOUT3				0x22	//0x00
#define REG_RXDELAY					0x23	//0x00

// Oscillator
#define REG_OSC						0x24	//0x07 clockout off

// Packet Handling
#define REG_PREAMBLEMSB				0x25 	//0x00
#define REG_PREAMBLELSB				0x26 	//0x64 send 100 preamble
#define REG_SYNCCONFIG				0x27	//0x18 00011000
#define REG_SYNCVALUE1				0x28	//0x48
#define REG_PACKETCONFIG1			0x30	//0x10 00000000 00010000
#define REG_PACKETCONFIG2			0x31	//0x40 01000-000//111
#define REG_PAYLOADLENGTH			0x32	//change to 7FF - 2047 bytes
#define REG_FIFOTHRESH				0x35	//dont need

//	Sequencer
#define REG_SEQCONFIG1				0x36
#define REG_SEQCONFIG2				0x37

#define REG_IMAGECAL				0x3B	//0xC2 11000010 10deg
#define REG_IRQFLAGS1				0x3E	//trigger
#define REG_IRQFLAGS2				0x3F	//trigger
#define REG_DIOMAPPING1				0x40	//00-00-00-00
#define REG_DIOMAPPING2				0x41	//0xF1 11-11-000-1
#define REG_REGVERSION				0x42	//read
#define REG_PLLHOP					0x44	
//#define REG_BITRATEFRAC				0x5D	//super minute accuracy which can be accounted for by the doppler shift
#define REG_AGCREFLF				0x61
#define REG_AGCTHRESHLF1			0x62
#define REG_AGCTHRESHLF2			0x63
#define REG_AGCTHRESHLF3			0x64	

//MODES
#define RFM98_MODE_SLEEP			0x08	
#define RFM98_MODE_STANDBY			0x09	//00001001
#define RFM98_MODE_FSTX				0x0A	//00001010
#define RFM98_MODE_TX				0x0B	//00001011
#define RFM98_MODE_FSRX				0x0C	//00001100
#define RFM98_MODE_RX				0x0D	//00001101

unsigned long compMessage[32] = {	0xAB,0xCD,0xEF,0x12, //4 bytes = 1 word
									0x34,0x56,0x78,0x9A,
									0xBC,0xDE,0xF1,0x23,
									0x45,0x67,0x89,0xAB,
									0xCD,0xEF,0x12,0x34,
									0x56,0x78,0x9A,0xBC,
									0xDE,0xF1,0x23,0x45,
									0x67,89,0xAB,0xCD};

void setup()
{
  Serial.begin(57600);
  initialiseSD();
  Serial.println();
  Serial.println(F("Groundstation Initializing.."));
  Serial.println();
  setRFM98W();
  
  
  Serial.println(F("Setup Complete"));
}

void loop()
{  
	CheckRx();
	//delay(100);
}
/*
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}*/

byte readRegister(byte addr)
{
  select();
  Serial.print(F("I am reading this "));
  Serial.print(addr);
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0x00);
  Serial.print(F(" and i am receiving this "));
  Serial.println(regval);
  unselect();
  return regval;
}

void writeRegister(byte addr, byte value)
{
  select();
  Serial.print(F("WritingRegister "));
  Serial.print(addr);
  Serial.print(F(" and setting it to "));
  Serial.println(value);
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

void select() 
{
  digitalWrite(_slaveSelectPin, LOW);
}

void unselect() 
{
  digitalWrite(_slaveSelectPin, HIGH);
}

void setRFM98W(void)
{
  // initialize the pins
  pinMode( _slaveSelectPin, OUTPUT);
  pinMode(dio0, INPUT);
  pinMode(dio3, INPUT);
  pinMode(dio4, INPUT);
  pinMode(dio5, INPUT);
  //setInterrupts();
  SPI.begin();
  SetFSKMod();
  //testCommunication();
  Receiver_Startup();
  return;
}
void initialiseSD(void) {
	while (!Serial) delay(25);
	Serial.print(F("Initializing SD card..."));

	//pinMode(53, OUTPUT);
	//pinMode(8, OUTPUT);
	if (!SD.begin(8)) {//8
		Serial.println(F("initialization failed!"));
		return;
	}
	Serial.println(F("initialization done."));
	Serial.println(F("please wait...."));
	
	for(int i=1; 1;i++)
    {
      filename=String(i)+".jpg";
      strcpy(cstr, filename.c_str());
      if (SD.exists(cstr))
      {
        filecount += 1;
      }
      else
      {	
        Serial.print(filecount);
        Serial.println(F(" existing files present.")); //debug
        filecount += 1;
        break;
      }
    }
}
void SetFSKMod()
{
  Serial.println(F("Setting FSK Mode"));
  setMode(RFM98_MODE_SLEEP);
  /*
  writeRegister(0x06, 0x6C);
  writeRegister(0x07, 0x9C);
  writeRegister(0x08, 0xCC);
  */
  writeRegister(REG_BITRATEMSB, 0x68); //0x00
  writeRegister(REG_BITRATELSB, 0x2B); //0x6B
  writeRegister(REG_FDEVMSB, 0x00); //2547*61.5*2 ~ 300khz
  writeRegister(REG_FDEVLSB, 0x31);// 1B
  writeRegister(REG_FRFMSB, 0x6C); //exact at 433Mhz
  writeRegister(REG_FRFMID, 0x9C);
  writeRegister(REG_FRFLSB, 0x8E);
   
  Serial.println(F("FSK Mode Set"));
  
  Serial.print(F("Mode = ")); 
  Serial.println(readRegister(REG_OPMODE));
  
  return;
}
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RFM98_MODE_SLEEP:
      Serial.println(F("Changing to Sleep Mode")); 
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RFM98_MODE_STANDBY:
      Serial.println(F("Changing to Standby Mode"));
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_FSTX:
      Serial.println(F("Changing to FSTx Mode"));
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_TX:
      Serial.println(F("Changing to Tx Mode"));
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_FSRX:
      Serial.println(F("Changing to FSRx Mode"));
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
	case RFM98_MODE_RX:
      Serial.println(F("Changing to Rx Mode"));
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RFM98_MODE_SLEEP){ //test on ModeReady
	while((digitalRead(dio5)) != 1)
    {
	  Serial.println(F("Wait for it..."));
    } 
  }
  Serial.println(F("Mode Change Done"));
  return;
}
void Receiver_Startup()
{

  //initialize
  setMode(RFM98_MODE_STANDBY);
  
  writeRegister(REG_LNA, 0xC0); //boost lna
  writeRegister(REG_RXCONFIG, 0x00);
  writeRegister(REG_RSSICONFIG, 0x03);
  writeRegister(REG_RSSICOLLISION, 0x0A);  
  writeRegister(REG_RSSITHRESH, 0xFF); //C8
  writeRegister(REG_RXBW, 0x17); //11 00000011
  //writeRegister(REG_AFCFEI, 0x00);
  writeRegister(REG_PREAMBLEDETECT, 0xC4);
  writeRegister(REG_RXTIMEOUT1, 0x00);
  writeRegister(REG_RXTIMEOUT2, 0x00);
  writeRegister(REG_RXTIMEOUT3, 0x00); 
  writeRegister(REG_OSC, 0x07); //@standby Clkout turned off
  
  writeRegister(REG_PREAMBLEMSB, 0x00);
  writeRegister(REG_PREAMBLELSB, 0x64);
  writeRegister(REG_SYNCCONFIG, 0x18);
  writeRegister(REG_SYNCVALUE1, 0x48);
  writeRegister(REG_PACKETCONFIG1, 0x10);
  writeRegister(REG_PACKETCONFIG2, 0x40);
  writeRegister(REG_PAYLOADLENGTH, 0x40);//FF
  writeRegister(REG_IMAGECAL, 0xC2);
  int i = readRegister(REG_IMAGECAL);
  while ((readRegister(REG_IMAGECAL) & 0x20) == 0x20) { 	//supposedly 160
  }
  writeRegister(REG_DIOMAPPING1, 0x00);
  writeRegister(REG_DIOMAPPING2, 0xF1);
  writeRegister(REG_PLLHOP, 0x00);
  setMode(RFM98_MODE_FSRX);
  setMode(RFM98_MODE_RX);
  //Serial.print(digitalRead(dio5)); 	//check these values
  //Serial.println(digitalRead(dio0));	//check these values
  return;
}
/*void readFIFO() {
	for (int i = 0; i < 256; i++)
	CurrentCount = receiveMessage(Message, CurrentCount);
	return;
	Serial.println("Finished Reading FIFO");
}*/
void CheckRx()
{
  int j = 0, k = 0;
  int sum;
  char RSSIString[6];
  //SentenceCount
  Serial.print(F("Signal Strength is at ")); 
  Serial.print(-(readRegister(REG_RSSIVALUE))/2);
  Serial.println(F("dBm"));
  uint8_t x = readRegister(0x3E);
  uint8_t y = readRegister(0x3F);	
  if ((x & 0x08) == 0x08)
	writeRegister(0x3E, 0x08);
  //delay(100);
  Serial.print(F("PayloadReady = "));
  //k = digitalRead(dio0);
  Serial.println(digitalRead(dio0));
  Serial.print(F("Fifo Empty = "));
  Serial.println((y & 0x40)==0x40);
  while ((y & 0x40)!=0x40) {
	CurrentCount = receiveMessage(Message, CurrentCount);
	if (CurrentCount == 62) {
		if (digitalRead(dio0) == 1) {
			Payload_Count++;
			usethis = 1;
		}
		else {
			Payload_Fail_Count++;
			usethis = 0;
			// Serial.println("MESSAGE IS ");
			// Serial.println(Message[CurrentCount]);
		}
	}
	if (j == 63) {
		j = 0;
		break;
	}
	j++;
	Serial.print(F("number now is"));
	Serial.println(j);
	//Serial.print("PayloadReady = ");
	//Serial.println(digitalRead(dio0));
  }
  Serial.print(F("Fifo Overrun = "));
  Serial.println((y & 0x10)==0x10);
  //if (digitalRead(dio3) == 0) delay(2000); //detect fifo empty
  Serial.print(F("Preamble Detected = "));
  Serial.println(digitalRead(dio4));
  Serial.print(F("ModeReady = "));
  Serial.println(digitalRead(dio5));
  Serial.print(F("Payload_Count = "));
  Serial.println(Payload_Count);
  Serial.print(F("Payload_Fail_Count = "));
  Serial.println(Payload_Fail_Count);
  Serial.print(F("usethis = "));
  Serial.println(usethis);
  
  //Serial.println(freeRam());
  
  if ((usethis == 1) && (Endflag == 0) && ((Message[62] == 0x00) || (Message[62] == 0x01) || (Message[62] == 0x02) || (Message[62] == 0x03) || (Message[62] == 0x04))) {
	Image_Packet_Count_new = Message[63];
	Serial.println(".....................GOT HERE");
	
	if (Image_Packet_Count_curr != Image_Packet_Count_new) //not necessarily increase by 1 due to packet loss
		used = 0;
	
	Serial.print(F("used = "));
	Serial.println(used);
	
	if (used == 0) {
		writingtofile();
		CorrectPackets++;
		Serial.println(F("I AM READING MESSAGE NOWWWWWWWWWWW"));
		used = 1;
		Image_Packet_Count_curr = Image_Packet_Count_new;
	}
	else { //if used == 1
		if (Message[62] == 4) //number of redundancy
			used = 0;
		else if ((Message[0] == 0) && (Message[63] == 0)) {
			for (int i = 0; i < 63; ++i)
				sum |= Message[i];
			if (sum == 0)
				used = 0;
			Serial.println(F("Used2"));
		}
	}
	//check for signal to Endflag image
	if ((Message[0] == 0xAB) && (Message[1] == 0xCD)) {
		Endflag = receiveEndImagePacket();
		Serial.println(F("_____________________________________________________________________________________________________________________________________END OF IMAGE"));
	}
	usethis = 0;
	
	//if ((CorrectPackets + 255*pseudo_CorrectPackets) != Message[63])
	//Serial.println(F("                                 ++++++++++++++++++++++++ERROR IS HERE++++++++++++++++++++++++"));
  }
  Serial.print(F("Image_Packet_Count_new = "));
  Serial.println(Image_Packet_Count_new);
  Serial.print(F("Image_Packet_Count_curr = "));
  Serial.println(Image_Packet_Count_curr);
  Serial.print(F("CorrectPackets = "));
  Serial.println(CorrectPackets);
  Serial.print(F("pseudo_CorrectPackets = "));
  Serial.println(pseudo_CorrectPackets);
  Serial.print(F("Packets Lost = "));
  Serial.println(Image_Packet_Count_new + 1 - pseudo_CorrectPackets*255 - CorrectPackets);
  Serial.print(F("Percentage Lost = "));
  Serial.println((Image_Packet_Count_new + 1 - pseudo_CorrectPackets*255 - CorrectPackets) / (Image_Packet_Count_new + 1));
  Serial.print(F("Endflag = "));
  Serial.println(Endflag);
  if (CurrentCount == 64) {
	CurrentCount = 0;
  }
  if (CorrectPackets == 255) {//roughly halfway mark
	pseudo_CorrectPackets += 1;
	CorrectPackets = 0;
  }
//  Serial.println("------------------------");
  
  /*if  ((digitalRead(dio3) == 1) && (digitalRead(dio0) == 0) && digitalRead(dio4) == 1) 			
	state = 1;	//Fifo Empty & Payload not ready & Preamble detected
  else if ((digitalRead(dio3) == 1) && (digitalRead(dio0) == 0) && (digitalRead(dio4) == 0)) 	
	state = 2;	//Fifo Empty & Payload not ready & Preamble not detected
  else if ((digitalRead(dio3) == 1) && (digitalRead(dio0) == 1)) 							
	state = 3;	//Fifo Empty & Payload ready 
  else if ((digitalRead(dio3) == 0) && (digitalRead(dio0) == 0)) 							
    state = 4;	//Fifo not Empty & Payload not ready
  else if ((digitalRead(dio3) == 0) && (digitalRead(dio0) == 1)) 							
    state = 5;	//Fifo not Empty & Payload ready
  
  switch (state) {
  case 1:
	Serial.println("Case 1 Triggered");
	/*while (digitalRead(dio3) == 1) { //wait until buffer is not empty
		//CurrentCount = receiveMessage(Message, CurrentCount);
		if ((digitalRead(dio4) & 0x02) != 0) { //if no preamble
			state = 2;
			break;
		}
	}*/
	//CurrentCount = receiveMessage(Message, CurrentCount);
	//readFIFO();
	/*for (int k = 0; k < CurrentCount; k++) {
	  Serial.print(Message[k]);
	}
	if (digitalRead(dio3) == 0 ) {
		CurrentCount = receiveMessage(Message, CurrentCount);
		state = 4;
		Serial.println("state transition from 1 to 4");
	}
	break;
  case 2:
	Serial.println("Case 2 Triggered");
	if (digitalRead(dio4) == 1) { //if preamble detected
	//& 0x02) != 0x02) { //no valid preamble
	//Serial.print("f");
	//Serial.println();
	state = 1;
	Serial.println("state transition from 2 to 1");
	}
	break;
  case 3:
	Serial.println("Case 3 Triggered");
	//check CRC okay, Reset and load to SD card
	for (int k = 0; k < CurrentCount; k++) {
	  Serial.print(Message[k]);
	}
	Serial.println();
	CurrentCount = 0;
	Serial.println("wait for new packet...");
	break;
  case 4:
	Serial.println("Case 4 Triggered");
	//wait for interrupt trigger
	CurrentCount = receiveMessage(Message, CurrentCount);
	if (digitalRead(dio3) == 1) {
		state = 1;
		Serial.println("state transition from 4 to 1");
	}
	else if (digitalRead(dio0) == 1) {
		state = 5;
		Serial.println("state transition from 4 to 5");
	}
	break;
  case 5:
	Serial.println("Case 5 Triggered");
	if (digitalRead(dio3) == 0) {
		state = 2;
		Serial.println("state transition from 5 to 2");
	}
	break;
  }*/
}
int receiveEndImagePacket() {
	int outcome = 0;
	Serial.println(F("Testing...."));
	for(int count = 0; count < 63; count++) {
		if (Message[count] == compMessage[count])
			outcome = 0;
		else {
			outcome = 1;
			Serial.println(F("It is not same"));
		}
	}
	outcome = !outcome;
	Serial.print(F("Outcome is "));
	Serial.println(outcome);
	return outcome; //return 1 if everything is okay and signal end of image
}
int receiveMessage(unsigned char *message, int i)
{
  const int Package = 64;
  
  if (i < Package)
	  message[i] = (unsigned char)readRegister(REG_FIFO);
  if (i == Package) 
	message[i+1] = '\0';
  return i+1;
}  

void writingtofile() {
	//start = micros();
	
	//char *cstr2=new char[8];
	
	//String filename2=String();
	//boolean EndFlag=0;
	//byte a[32];
	byte incomingbyte;
	int ii, j, k, i = 0;

	filename=String(filecount)+".jpg";
	//filename2=String(filecount)+".txt";
	strcpy(cstr, filename.c_str());
	if (SD.exists(cstr)) {
		if (imagedone == 1) //need to end imagedone = 0
			filecount++;
	}
	//write from 0 to 62 cause 63 is my parity bit
	myFile = SD.open(cstr, FILE_WRITE);
	if (imagedone == 0) {
		for(int ii=0; ii<62; ii++) {
			myFile.write(Message[ii]);
			/*if((Message[ii-1]==0xFF)&&(Message[ii]==0xD9))     //tell if the picture is finished
			{
//			  Serial.println(fin);
			  Serial.println(F("jackpot"));
			  imagedone = 1;
			  break;
//			  EndFlag=1;
			}*/
		}
	}	
	//just print for fun
	for(j=0;j<CurrentCount;j++) {
		if(Message[j]<0x10)  
			Serial.print("0");
		Serial.print(Message[j],HEX);
		Serial.print(" ");
	}
	myFile.close();
	Serial.println(F("Finished writing data to file"));
}
