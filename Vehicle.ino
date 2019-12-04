// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

int pin_5 = 5;
int pin_6 = 6;
int pin_7 = 7;
int pin_8 = 8;
int pin_9 = 9;
int pin_10 = 10;
int pin_A3 = A3;
int pin_A4 = A4;
int pin_A5 = A5;

void setup(){

  pinMode(pin_5, OUTPUT);
  pinMode(pin_6, OUTPUT);
  pinMode(pin_7, OUTPUT);
  pinMode(pin_8, OUTPUT);
  pinMode(pin_9, OUTPUT);
  pinMode(pin_10, OUTPUT);
  pinMode(pin_A3, OUTPUT);
  pinMode(pin_A4, OUTPUT);
  pinMode(pin_A5, OUTPUT);
  
  Serial.begin(9600);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


void loop() {
  char m1char[4];
  char m2char[4];
  char m3char[4];
  char modechar[2];
  int m1_val;
  int m2_val;
  int m3_val;
  int mode;
  
 if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;

      m1char[0] = (char*)buf[0];
      m1char[1] = (char*)buf[1];
      m1char[2] = (char*)buf[2];
      m1char[3] = '\0';
      m2char[0] = (char*)buf[3];
      m2char[1] = (char*)buf[4];
      m2char[2] = (char*)buf[5];
      m2char[3] = '\0';
      m3char[0] = (char*)buf[6];
      m3char[1] = (char*)buf[7];
      m3char[2] = (char*)buf[8];
      m3char[3] = '\0';
      modechar[0] = (char*)buf[9];
      modechar[1] = '\0';

      m1_val = atoi(m1char);
      m2_val = atoi(m2char);
      m3_val = atoi(m3char);
      mode = atoi(modechar);

      Serial.print(m1_val);Serial.print(m2_val);Serial.println(m3_val);Serial.println(mode);

      if(mode == 0){
        if(m1_val < 126){
          digitalWrite(pin_7, LOW);
          digitalWrite(pin_9, HIGH);
          analogWrite(pin_5, 252-(m1_val*2));
        }else if(m1_val > 129){
          digitalWrite(pin_7, HIGH);
          digitalWrite(pin_9, LOW);
          analogWrite(pin_5, (m1_val-129)*2);
        }else if(m1_val >= 126 && m1_val <= 129){
          digitalWrite(pin_7, LOW);
          digitalWrite(pin_9, LOW);
          analogWrite(pin_5, 0);
        }
        
        if(m2_val < 126){
          digitalWrite(pin_8, LOW);
          digitalWrite(pin_10, HIGH);
          analogWrite(pin_6, 252-(m2_val*2));
        }else if(m2_val >129){
          digitalWrite(pin_8, HIGH);
          digitalWrite(pin_10, LOW);
          analogWrite(pin_6, (m2_val-129)*2);
        }else if(m2_val >= 126 && m2_val <= 129){
          digitalWrite(pin_8, LOW);
          digitalWrite(pin_10, LOW);
          analogWrite(pin_6, 0);
        }
      }else{
          digitalWrite(pin_8, LOW);
          digitalWrite(pin_10, LOW);
          analogWrite(pin_6, 0);
          digitalWrite(pin_7, LOW);
          digitalWrite(pin_9, LOW);
          analogWrite(pin_5, 0);
        if(m3_val < 126){
          digitalWrite(pin_A5, HIGH);
          digitalWrite(pin_A4, LOW);
          analogWrite(pin_A3, 252-(m3_val*2));
        }else if(m3_val > 129){
          digitalWrite(pin_A5, LOW);
          digitalWrite(pin_A4, HIGH);
          analogWrite(pin_A3, (m3_val-129)*2);
        }else if(m3_val >= 126 && m3_val <= 129){
          digitalWrite(pin_A5, LOW);
          digitalWrite(pin_A4, LOW);
          analogWrite(pin_A3, 0);
        }
      }
    }
  }
}
