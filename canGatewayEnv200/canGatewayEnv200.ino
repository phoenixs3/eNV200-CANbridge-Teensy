#include <FlexCAN_T4.h>
#include "canmessage-t4.h"

#define LSB 0
#define MSB 1
#define UNSIGNED 0
#define SIGNED 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;   //Connected to vehicle
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;   //Connected to Battery

CAN_message_t outMsg;
CAN_message_t inMsg;

#define LEDpin 13
bool ledState = false;

volatile  uint16_t  main_battery_soc  = 0;
volatile  uint16_t  GIDS        = 0;
volatile  uint8_t   ticks10ms       = 0;
uint8_t    crctable[256]           = {0,133,143,10,155,30,20,145,179,54,60,185,40,173,167,34,227,102,108,233,120,253,247,
114,80,213,223,90,203,78,68,193,67,198,204,73,216,93,87,210,240,117,127,250,107,238,228,97,160,37,47,170,59,190,180,49,19,
150,156,25,136,13,7,130,134,3,9,140,29,152,146,23,53,176,186,63,174,43,33,164,101,224,234,111,254,123,113,244,214,83,89,220,
77,200,194,71,197,64,74,207,94,219,209,84,118,243,249,124,237,104,98,231,38,163,169,44,189,56,50,183,149,16,26,159,14,139,129,
4,137,12,6,131,18,151,157,24,58,191,181,48,161,36,46,171,106,239,229,96,241,116,126,251,217,92,86,211,66,199,205,72,202,79,69,
192,81,212,222,91,121,252,246,115,226,103,109,232,41,172,166,35,178,55,61,184,154,31,21,144,1,132,142,11,15,138,128,5,148,17,
27,158,188,57,51,182,39,162,168,45,236,105,99,230,119,242,248,125,95,218,208,85,196,65,75,206,76,201,195,70,215,82,88,221,255,
122,112,245,100,225,235,110,175,42,32,165,52,177,187,62,28,153,147,22,135,2,8,141};

//----Cyclic Timing Variables----////////////
elapsedMillis ms100Timer;
const int ms100 = 100;          //100 Milliseconds
/////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.print("CAN Gateway eNV200...");

  pinMode(LEDpin, OUTPUT);
  
  can1.begin();
  can1.setBaudRate(500000);
  can2.begin();
  can2.setBaudRate(500000);

  digitalWrite(LEDpin, HIGH);
  
  Serial.setTimeout(10);
  Serial.println("done");
}

void loop() {

  if (Serial.available() > 0){
    int tempint = Serial.parseInt();
    if (tempint > 0){
      Serial.print("Your input: ");
      Serial.println(tempint);
    }
  }

  if (ms100Timer >= ms100) {
    ms100Timer = ms100Timer - ms100;
    ledState = !ledState;
    digitalWrite(LEDpin, ledState);
  }
  
  if (can1.read(inMsg)) {can1Read();}
  if (can2.read(inMsg)) {can2Read();}
}


void can1Read(){    //Read messages coming from Vehicle
  can1.read(inMsg);
  switch (inMsg.id) { 
    
    case 0x1F2:                       //Upon reading VCM originating 0x1F2 every 10ms, send missing message to battery every 40ms
      ticks10ms++;
      if(ticks10ms > 3){
        ticks10ms = 0;
        sendZE1message355();
      }
      break;
    
    case 0x679:                       //Send missing 2018+ startup messages towards battery
      sendZE1startupMessage603();
      sendZE1startupMessage605();
      break;
    
    default:
      can2.write(inMsg);    //Forward any other message onto battery
      break;
  }
}

void can2Read(){   //Read messages coming from Battery
  can2.read(inMsg);
  uint16_t temp;
  switch (inMsg.id) {
    case 0x55B:                      //Upon reading 0x55B coming from battery every 100ms, send missing messages towards battery
      main_battery_soc = (inMsg.buf[0] << 2) | ((inMsg.buf[1] & 0xC0) >> 6); //Capture SOC% needed for QC_rescaling
      main_battery_soc /= 10;                                                //Remove decimals, 0-100 instead of 0-100.0
      sendinstrumentCluster5E3();
      sendZE1message5C5();
      break;
    
    case 0x5BC:             //Fixes range estimation
      outMsg = inMsg;       //First copy entire message contents over
      if((outMsg.buf[5] & 0x10) == 0x00){ 
        GIDS = ((outMsg.buf[0] << 2) | ((outMsg.buf[1] & 0xC0) >> 6));  //LB_MAXGIDS is 0, store GIDS
      }
      //Avoid blinking GOM by always writing remaining GIDS
      outMsg.buf[0] = GIDS >> 2;
      outMsg.buf[1] = (GIDS << 6) & 0xC0;
      outMsg.buf[5] = outMsg.buf[5] & 0x03; //Clear LB_Output_Power_Limit_Reason and LB_MaxGIDS
      can1.write(outMsg);   //Send modified battery to vehicle 
    break;

    case 0x59E:             //Fixes quick-charge issue on some chargers (capacity remaining message)
      outMsg = inMsg;       //First copy entire message contents over
      temp = ((230 * main_battery_soc)/100);                        //Calculate new LBC_QC_CapRemaining value
      outMsg.buf[3] = (outMsg.buf[3] & 0xF0) | ((temp >> 5) & 0xF); //Store the new LBC_QC_CapRemaining
      outMsg.buf[4] = ((temp & 0x1F) <<3) | (outMsg.buf[4] & 0x07); //To the 59E message out to vehicle
      calc_crc8(&outMsg);
      can1.write(outMsg);   //Send modified battery to vehicle
    break;

    default:
      can1.write(inMsg);  //Forward any other message onto Vehicle
      break;
  }
}

void sendZE1message355(){
  outMsg.id  = 0x355;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  outMsg.buf[0] = 0x14;
  outMsg.buf[1] = 0x0A;
  outMsg.buf[2] = 0x13;
  outMsg.buf[3] = 0x97;
  outMsg.buf[4] = 0x10;
  outMsg.buf[5] = 0x00;
  outMsg.buf[6] = 0x40;
  outMsg.buf[7] = 0x00;
  can2.write(outMsg);
}

void sendZE1startupMessage603(){
  outMsg.id  = 0x603;
  outMsg.len = 1;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  outMsg.buf[0] = 0x0;
  can2.write(outMsg);
}

void sendZE1startupMessage605(){
  outMsg.id  = 0x605;
  outMsg.len = 1;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  outMsg.buf[0] = 0x0;
  can2.write(outMsg);
}

void sendinstrumentCluster5E3(){
  outMsg.id  = 0x5E3;
  outMsg.len = 5;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  outMsg.buf[0] = 0x8E;
  outMsg.buf[1] = 0x00;
  outMsg.buf[2] = 0x00;
  outMsg.buf[3] = 0x00;
  outMsg.buf[4] = 0x80;
  can2.write(outMsg);
}

void sendZE1message5C5(){
  outMsg.id  = 0x5C5;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  outMsg.buf[0] = 0x84;
  outMsg.buf[1] = 0x01;
  outMsg.buf[2] = 0x06;
  outMsg.buf[3] = 0x79;
  outMsg.buf[4] = 0x00;
  outMsg.buf[5] = 0x0C;
  outMsg.buf[6] = 0x00;
  outMsg.buf[7] = 0x00;
  can2.write(outMsg);
}

void calc_crc8(CAN_message_t *outMsg){
  //Recalculates the CRC-8 with 0x85 poly
  uint8_t crc = 0;
  for(uint8_t i = 0; i < 7; i++){crc = crctable[(crc ^ ((int) (*outMsg).buf[i])) % 256];}
  (*outMsg).buf[7] = crc;
}
