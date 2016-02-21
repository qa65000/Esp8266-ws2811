#include <ESP8266WiFi.h>
#include <WiFiUDP.h>


//////////////////////
// Port Definitions //
//////////////////////
#define  ODATA      16
#define  OLED_BLUE   4

#define DATA_PORT_HIGH    GP16O = 1
#define DATA_PORT_LOW     GP16O = 0
#define WAIT1US           work++;

//////////////////////
// WiFi Definitions //
//////////////////////
const char APPass[] = "12345678";      // 8 charctor
const unsigned int localPort = 65001;

/////////////////////
// Struct          //
/////////////////////
typedef struct Led{
  byte Green;
  byte Red;
  byte Blue;
}TLedRGB;

struct TCommand {
   byte Command;
   TLedRGB RGBBuf[5];
};
/////////////////////
// Data            //
/////////////////////
WiFiUDP Udp;
byte PacketBuff[256]; //buffer to hold incoming and outgoing packets
TLedRGB RGBLatch[5];

/////////////////////////////////////////
/////// SetUp ///////////////////////////
/////////////////////////////////////////





volatile uint8_t  work;
void WS2811_Code0(void) 
{   

  DATA_PORT_HIGH; 
  WAIT1US;         
  DATA_PORT_LOW;          
  WAIT1US;         
  WAIT1US;         
  WAIT1US;         
  WAIT1US;         
  WAIT1US;         
}

void WS2811_Code1(void) 
{   
  DATA_PORT_HIGH;      
  WAIT1US;                   
  WAIT1US;         
  WAIT1US;       
  WAIT1US;                   
  WAIT1US;         
  DATA_PORT_LOW;          
  WAIT1US;         
  WAIT1US;         
  WAIT1US;         
  WAIT1US;         
}

void WS2811_Reset(void) 
{   
  DATA_PORT_LOW;      
  delay(50);  
}


void SendLed( uint8_t * DataArray )
{
uint8_t Index = 0;
uint8_t BCount = 8;
uint8_t BitData;
for ( Index = 0 ; Index < 3 ; Index ++)
  {
      BitData = DataArray[Index]<<1;
      for( BCount = 0 ; BCount < 8 ; BCount++)
      { 
          if(BitData & 0x80)
          {
              WS2811_Code1();
          }
          else
          {
              WS2811_Code0();
          }
          if(Index != 3) 
          {
            BitData = BitData << 1;
          }
      }
  }
}
void setup() 
{
//////////////////////////////////////
  uint8_t MacAdr[WL_MAC_ADDR_LENGTH];
  String StrMac = "";
  int i;
//////////////////////////////////////
  pinMode(ODATA   , OUTPUT);
  pinMode(OLED_BLUE  , OUTPUT);
  
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);       //access point Start!
 
  WiFi.softAPmacAddress(MacAdr);
  ////////////// MacAdr=>String//////////////////
   for ( i = 0; i < sizeof(MacAdr) ; i++)
  {
     StrMac = StrMac + String(MacAdr[i], HEX);
    if(i != (sizeof(MacAdr)-1)) StrMac = StrMac + ":"; 
  }
  Serial.println("\nStart");
  Serial.println(StrMac);

  String APName = "LedColor"
               +String(MacAdr[sizeof(MacAdr)-3],HEX)+'-'
                 +String(MacAdr[sizeof(MacAdr)-2],HEX)+'-'
                  +String(MacAdr[sizeof(MacAdr)-1],HEX);    //Null Auto
                
  WiFi.softAPConfig(IPAddress(192, 168, 0, 1),              // ip
                          IPAddress(192, 168, 0, 1),        // gateway
                           IPAddress(255, 255, 255, 0)  );  //sub net mask
                 
  WiFi.softAP((char *)&APName[0] , APPass );
  Serial.println("Name:"+APName );

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Wifi:"+WiFi.softAPIP());
/////////////////////////////////////////////////////
  if(Udp.begin(localPort))
  {
    Serial.println("Server Success");
  }
  else
  {
    Serial.println("Sever Error");
  }

  Serial.print("Local port: ");  Serial.println(Udp.localPort());
 
  Serial.print("Wifi:"+WiFi.softAPIP());
 RGBLatch[0].Red  = 127;
 RGBLatch[1].Green  = 127;
 RGBLatch[2].Blue  = 127;
 RGBLatch[3].Blue  = 127;
 RGBLatch[3].Green  = 127;
 RGBLatch[4].Blue  = 127;
 RGBLatch[4].Red  = 127;
 RGBLatch[4].Green  = 127;
          noInterrupts ();
          SendLed((uint8_t *)&RGBLatch[0]);
          SendLed((uint8_t *)&RGBLatch[1]);
          SendLed((uint8_t *)&RGBLatch[2]);
          SendLed((uint8_t *)&RGBLatch[3]);
          SendLed((uint8_t *)&RGBLatch[4]);
          WS2811_Reset();

          interrupts ();         
}

/////////////////////////////////////////
/////// Main Loop ///////////////////////
/////////////////////////////////////////
void loop(void) 
{
  unsigned char  ReadCount;
  ReadCount = Udp.parsePacket();
  if ( ReadCount >= 10 )
  {
    Udp.read(PacketBuff, ReadCount); // read the packet into the buffer
    switch ( PacketBuff[0] )
    {
        case  'L' :
          RGBLatch[0]  = ((TCommand*)PacketBuff)->RGBBuf[0];
          RGBLatch[1]  = ((TCommand*)PacketBuff)->RGBBuf[1];
          RGBLatch[2]  = ((TCommand*)PacketBuff)->RGBBuf[2];
          RGBLatch[3]  = ((TCommand*)PacketBuff)->RGBBuf[3];
          RGBLatch[4]  = ((TCommand*)PacketBuff)->RGBBuf[4];
          noInterrupts ();
          SendLed((uint8_t *)&RGBLatch[0]);
          SendLed((uint8_t *)&RGBLatch[1]);
          SendLed((uint8_t *)&RGBLatch[2]);
          SendLed((uint8_t *)&RGBLatch[3]);
          SendLed((uint8_t *)&RGBLatch[4]);
          WS2811_Reset();

          interrupts ();
          break;
        default:
          break;
     }
  } 
#if 0
  {
          RGBLatch[0].Red += 0x10;
          if(RGBLatch[0].Red == 0)
          {
           RGBLatch[1].Red += 0x80;
             RGBLatch[2].Red += 0x80;
             RGBLatch[3].Red+= 0x80;
             RGBLatch[4].Red+= 0x80;
          }
          
     //volatile  
     noInterrupts ();
   Serial.println(RGBLatch[0].Red);
//    ets_intr_lock(0xffff);
//    ets_isr_mask(0X000FFFFL);
  
  ///   delay(100);
    
    noInterrupts ();
 digitalWrite(OLED_BLUE, HIGH);
    cli();
        SendLed((uint8_t *)&RGBLatch[0]);
       SendLed((uint8_t *)&RGBLatch[1]);
       SendLed((uint8_t *)&RGBLatch[2]);
       SendLed((uint8_t *)&RGBLatch[3]);
       SendLed((uint8_t *)&RGBLatch[4]);
       WS2811_Reset();
//        ets_intr_unlock(0xffff);
 digitalWrite(OLED_BLUE, LOW);
      sei();
    //volatile 
    interrupts ();
   // ets_isr_unmask(0XFFFFFFFFL);

  }
#endif
}  
