#include <SPI.h>
#include <RF24.h>
RF24 radio(9,10);

// Code for the video: https://youtu.be/pyhpXnFzNhU
// (C)2019 Pawel A. Hernik
// BLE code based on Dmitry Grinberg and Florian Echtler work

/* PINOUT
  nRF24L01 from pin side/top:
  -------------
  |1 3 5 7    |
  |2 4 6 8    |
  |           |
  |           |
  |           |
  |           |
  |           |
  -------------
  1 - GND  blk   GND
  2 - VCC  wht   3V3
  3 - CE   orng  9
  4 - CSN  yell  10
  5 - SCK  grn   13
  6 - MOSI blue  11
  7 - MISO viol  12
  8 - IRQ  gray  2

 More info about nRF24L01:
 http://arduinoinfo.mywikis.net/wiki/Nrf24L01-2.4GHz-HowTo#po1
*/

#include <OLEDSoftI2C_SSD1306.h>
//#include <OLEDSoftI2C_SH1106.h>
// define USEHW in above header for hw I2C version

OLEDSoftI2C_SSD1306 oled(0x3c);
//OLEDSoftI2C_SH1106 oled(0x3c);
int oled128x32 = 0;

#include "term8x10_font.h"
#include "small5x7_font.h"
#include "chicago_th_font.h"

// -------------------------

void setup()
{
  initBLE();
  Serial.begin(115200);
  Serial.println(F("BLE Mi Bluetooth Temperature & Humidity Monitor"));
  oled.init(oled128x32);  // 0-128x64, 1-128x32
  oled.clrScr();
}

// -------------------------

const uint8_t channel[3]   = {37,38,39};  // BLE advertisement channel number
const uint8_t frequency[3] = { 2,26,80};  // real frequency (2400+x MHz)

struct bleAdvPacket { // for nRF24L01 max 32 bytes = 2+6+24
  uint8_t pduType;
  uint8_t payloadSize;  // payload size
  uint8_t mac[6];
  uint8_t payload[24];
};

uint8_t currentChan=0;
bleAdvPacket buffer;

void initBLE() 
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.disableCRC();
  radio.setChannel( frequency[currentChan] );
  radio.setRetries(0,0);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAddressWidth(4);
  radio.openReadingPipe(0,0x6B7D9171); // advertisement address: 0x8E89BED6 (bit-reversed -> 0x6B7D9171)
  radio.openWritingPipe(  0x6B7D9171);
  radio.powerUp();
}

void hopChannel()
{
  currentChan++;
  if(currentChan >= sizeof(channel)) currentChan = 0;
  radio.setChannel( frequency[currentChan] );
}

bool receiveBLE(int timeout)
{
  radio.startListening();
  delay(timeout);
  if(!radio.available()) return false;
  while(radio.available()) {
    radio.read( &buffer, sizeof(buffer) );
    swapbuf( sizeof(buffer) );
    whiten( sizeof(buffer) );
  }
  return true;
}

// change buffer content to "wire bit order"
void swapbuf(uint8_t len) 
{
  uint8_t* buf = (uint8_t*)&buffer;
  while(len--) {
    uint8_t a = *buf;
    uint8_t v = 0;
    if (a & 0x80) v |= 0x01;
    if (a & 0x40) v |= 0x02;
    if (a & 0x20) v |= 0x04;
    if (a & 0x10) v |= 0x08;
    if (a & 0x08) v |= 0x10;
    if (a & 0x04) v |= 0x20;
    if (a & 0x02) v |= 0x40;
    if (a & 0x01) v |= 0x80;
    *(buf++) = v;
  }
}

void whiten(uint8_t len)
{
  uint8_t* buf = (uint8_t*)&buffer;
  // initialize LFSR with current channel, set bit 6
  uint8_t lfsr = channel[currentChan] | 0x40;
  while(len--) {
    uint8_t res = 0;
    // LFSR in "wire bit order"
    for (uint8_t i = 1; i; i <<= 1) {
      if (lfsr & 0x01) {
        lfsr ^= 0x88;
        res |= i;
      }
      lfsr >>= 1;
    }
    *(buf++) ^= res;
  }
}

// -------------------------

// real width is wd+6
void drawBattBig(int x, int y, int wd, int perc)
{
  if(perc<0) perc=0;
  if(perc>100) perc=100;
  int w = wd*perc/100;
  oled.fillWin(x,y+0,1,1,B11111111);
  oled.fillWin(x,y+1,1,1,B00000011); x++;  // bottom full
  oled.fillWin(x,y+0,1,1,B00000001);
  oled.fillWin(x,y+1,1,1,B00000010); x++;  // bottom empty
  oled.fillWin(x,y+0,w,1,B11111101);
  oled.fillWin(x,y+1,w,1,B00000010);       // fill full
  x+=w;
  w=wd-w;
  if(w>0) {
    oled.fillWin(x,y+0,w,1,B00000001);     // fill empty
    oled.fillWin(x,y+1,w,1,B00000010);
    x+=w;
  }
  oled.fillWin(x,y+0,1,1,B00000001);
  oled.fillWin(x,y+1,1,1,B00000010); x++;  // top empty
  oled.fillWin(x,y+0,1,1,B11111111);
  oled.fillWin(x,y+1,1,1,B00000011); x++;  // top full
  oled.fillWin(x,y+0,1,1,B01111000);
  oled.fillWin(x,y+1,1,1,B00000000); x++;  // tip*2
  oled.fillWin(x,y+0,1,1,B01111000);
  oled.fillWin(x,y+1,1,1,B00000000);
}

// -------------------------

char buf[100];
int temp=-1000;
int hum=-1;
int bat=-1;
int x,cnt,mode,v1,v10;
int tempOld=-123;
int humOld=-123;
int batOld=-123;
int cntOld = -1;
unsigned long tm=0;
char *modeTxt;

// Xiaomi advertisement packet decoding
//          18       21 22 23 24
//          mm       tl th hl hh 
// a8 65 4c 0d 10 04 da 00 de 01  -> temperature+humidity
//          mm       hl hh
// a8 65 4c 06 10 04 da 01        -> humidity
//          mm       bb 
// a8 75 4c 0a 10 01 60           -> battery
// 75 e7 f7 e5 bf 23 e3 20 0d 00  -> ???
//          21 e6 f6 18 dc c6 01  -> ???
// b8 65 5c 0e 10 41 60           -> battery??
// a8 65 4c 46 10 02 d4 01        -> ?? 

void loop() 
{
  receiveBLE(100);
  uint8_t *recv = buffer.payload;
  //if(buffer.mac[5]==0x4c && buffer.mac[0]==0xe)  // my Xiaomi MAC address (1st and last number only)
  if(recv[5]==0x95 && recv[6]==0xfe && recv[7]==0x50 && recv[8]==0x20)
  {
    cnt=recv[11];
    mode=recv[18];
    int mesSize=recv[3];
    int plSize=buffer.payloadSize-6;
    if(mode==0x0d && plSize==25) { // temperature + humidity (missing msb, so cannot be used)
      temp=recv[21]+recv[22]*256;
      modeTxt="TH";
      snprintf(buf,100,"#%02x %02x %s %02x %3d'C (%3d%%)",cnt,mode,modeTxt,recv[3],recv[21]+recv[22]*256,recv[23]);
    } else if(mode==0x04 && plSize==23) {  // temperature
      temp=recv[21]+recv[22]*256;
      modeTxt="T ";
      snprintf(buf,100,"#%02x %02x %s %02x %3d'C       ",cnt,mode,modeTxt,recv[3],recv[21]+recv[22]*256);
    } else if(mode==0x06 && plSize==23) {  // humidity
      hum=recv[21]+recv[22]*256;
      modeTxt="H ";
      snprintf(buf,100,"#%02x %02x %s %02x %3d%%        ",cnt,mode,modeTxt,recv[3],recv[21]+recv[22]*256);
    } else if(mode==0x0a && plSize==22) {  // battery level
      bat=recv[21];
      modeTxt="B ";
      snprintf(buf,100,"#%02x %02x %s %02x %03d%% batt   ",cnt,mode,modeTxt,recv[3],recv[21]);
    } else {
      modeTxt="??";
      snprintf(buf,100,"!!!!!!%02x %02x %s %02x %03d %03d",cnt,mode,modeTxt,recv[3],recv[21],recv[22]);
    }
    Serial.print(buf); 
    snprintf(buf,100,"  [%02x:%02x:%02x:%02x:%02x:%02x] ch%d s=%02d: ",buffer.mac[5],buffer.mac[4],buffer.mac[3],buffer.mac[2],buffer.mac[1],buffer.mac[0],currentChan,plSize);
    Serial.print(buf);
    int n = plSize<=24?plSize:24;
    for(uint8_t i=0; i<n; i++) { snprintf(buf,100,"%02x ",buffer.payload[i]); Serial.print(buf); }
    Serial.println();
  }
  hopChannel();

  if(cntOld!=cnt) {
    cntOld=cnt;
    oled.setFont(Small5x7PL);
    oled.setDigitMinWd(5);
    snprintf(buf,100,"#%02x %ds  ",cnt,(millis()-tm)/1000);
    oled.printStr(ALIGN_LEFT, 0, buf);
    snprintf(buf,100,"<%02x>",mode);
    oled.printStr(ALIGN_LEFT, 1, buf);
    oled.printStr(24, 1, modeTxt);
    tm=millis();
  }

  if(tempOld==temp && humOld==hum && batOld==bat) return;
  tempOld=temp; humOld=hum; batOld=bat;
  
  oled.setFont(Term8x10);
  oled.setDigitMinWd(8);

  if(bat<0 || bat>100)
     strcpy(buf," --");
  else {
    snprintf(buf,100," %2d",bat);
  }
  int wd = oled.strWidth(buf);
  oled.printStr(128-18-wd-6-4, 0, buf);
  drawBattBig(128-18-6,0,18,bat);

  oled.setFont(Chicago21x18th);
  oled.setDigitMinWd(15);
  oled.setCharMinWd(6);

  if(temp<=-400 || temp>800)
     strcpy(buf,"--_- ");
  else {
    v1=temp/10;
    v10=temp-v1*10;
    snprintf(buf,100,"%2d_%d ",v1,v10);
  }
  x=oled.printStr(24, 2, buf);
  oled.printStr(x-12, 2, "`C ");

  if(hum<0 || hum>1000)
     strcpy(buf,"--_- ");
  else {
    v1=hum/10;
    v10=hum-v1*10;
    snprintf(buf,100,"%2d_%d ",v1,v10);
  }
  x=oled.printStr(24, 5, buf);
  oled.printStr(x-12, 5, "% ");
}

