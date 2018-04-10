#include <Arduino.h>
#include <EEPROM.h>
#include "etelex.h"
#include "baudot.h"
#include "lookup.h"




#ifdef ESP
WiFiServer server(MYPORT);
WiFiClient client;
#else
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { MYMACADRESS };

//Set the static IP address to use if the DHCP fails to assign
IPAddress ip(MYIPFALLBACK);
// Initialize the Ethernet client library
// with the IP address and port of the server
EthernetServer server(MYPORT);
EthernetClient client;
byte currentSocknum;
#endif


#ifdef ESP
const byte COMMUTATE_PIN = 5; // umpol pin for the relais
const byte RECIEVE_PIN = 13;     // recieve Pin
const byte SEND_PIN = 12; // send pin for the Mosfet
#else
const byte COMMUTATE_PIN = 2; // umpol pin for the relais
const byte RECIEVE_PIN = 3;     // recieve Pin
const byte SEND_PIN = 5; // send pin for the Mosfet
#endif

const int DATABIT_DURATION = 20; //milliseconds
const int STARTBIT_DURATION = 20; // milliseconds
const int STOPBIT_DURATION = 40; // milliseconds
const int SAMPLEPOS=60; // 60 von 120

byte state;



#define STATE_RUHEZUSTAND  0 //4ma  ump=0, ssr=0
#define STATE_ANFANGSSIGNAL  10 //4ma -> 40 ma -> opto==1 ca. 1000ms delay
#define STATE_WAHLAUFFORDERUNGSSIGNAL 20 //brk=1, 25ms delay
#define STATE_WARTE_AUF_ZIFFER  30  //brk=0, /warte  5000ms auf opto==0 -> 31 sonst -> 34 auf opto==0
#define STATE_WAHLIMPULS  31  //opto==0, warte auf opto==1 -> 32
#define STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS  32  //opto==1, warte auf 200ms auf opto==0 -> 31 sonst -> 33
#define STATE_WAHL_ENDE  34  // nummer zu kurz -> 90 sonst 40
#define STATE_LOOKUP  40  //nummer nicht gefunden -> 90 sonst 50

#define STATE_CONNECTING  50  // verbindungs timeout -> 90 sonst 100
#define STATE_BUSY   90  //ump=1, SSR=1, delay 2000 -> 0
#define STATE_ONLINE   100  //ump=1, SSR=1, disconnect -> 0, opto==0 <2000s -> 110 sonst -> 0, recieve data via tcp ->120
#define STATE_LOCALMODE   110

#define STATE_DISCONNECT 200

byte currentDigit;
byte currentNumberPos;
#define NUMBER_BUFFER_LENGTH 20
char number[NUMBER_BUFFER_LENGTH];

volatile byte recieve_buf; // recieve buffer
volatile byte recieved_char; //

volatile boolean new_char_recieved=false;

unsigned long st_timestamp;



volatile unsigned int bit_pos=0; // aktuelles bit in rcve, startbit=1
volatile boolean recieving=false;
volatile unsigned long last_timestamp;

#ifdef CENTRALEX

boolean configuringNumber=false;
boolean configuringPin=false;
unsigned long lastPing;
unsigned long awaiting_pong; //TODO implement client.stop if >= millis()-PINGTIMEOUT
char ping[] = "\016ping;\017";
boolean logged_in = false;
boolean cmdMode = false;
char textBuffer[TEXTBUFFERSIZE];
char textBufferIndex = 0;
char cmdBuffer[CMDBUFFERSIZE];
char cmdBufferIndex = 0;
char writeTo = TEXT;
#endif

void byte_send(byte _byte) {
  recieving=false;

  digitalWrite(SEND_PIN, HIGH);
  delay(STARTBIT_DURATION);

  for (int _bit = 4; _bit >= 0; _bit--) {
    digitalWrite(SEND_PIN, !bitRead(_byte, _bit));
    delay(DATABIT_DURATION);
  }

  digitalWrite(SEND_PIN, LOW);
  delay(STOPBIT_DURATION);

  recieving=true;
} // end byte_send()

void sendAsciiAsBaudot(char ascii) {
  byte modeswtichcode;
  byte baudot = asciiToBaudot(ascii,&modeswtichcode);

  if(baudot!=BAUDOT_CHAR_UNKNOWN){
    if(modeswtichcode!=BAUDOT_MODE_UNDEFINED){
      byte_send(modeswtichcode);
#ifdef _DEBUG_2
      PgmPrint("Switching mode to ");
      Serial.print(modeswtichcode);
      PgmPrint("\n");
#endif
    }
    byte_send(baudot);
  }
} // end char_send()




#ifdef USEIRQ
#ifdef ESP
void ICACHE_RAM_ATTR onlinepinchange(){
#else
void onlinepinchange(){
#endif

    if(recieving && bit_pos==0){
//RISING/FALLING      if( digitalRead(RECIEVE_PIN)){ //startbit
        last_timestamp=millis();
        bit_pos++;
//      }
    }
 }
#endif

void setup() {
  pinMode(RECIEVE_PIN, INPUT_PULLUP);
  pinMode(SEND_PIN, OUTPUT);
  pinMode(COMMUTATE_PIN, OUTPUT);


  digitalWrite(SEND_PIN, LOW);
  digitalWrite(COMMUTATE_PIN, LOW);
  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

#ifdef USE_SDCARD
  pinMode(10, OUTPUT);                       // set the SS pin as an output (necessary!)
  digitalWrite(10, HIGH);                    // but turn off the W5100 chip!

  if (!SD.begin(4)) {
      PgmPrintln("SD initialization failed!");
  }else{
    PgmPrintln("Success SD init.");
  }
#endif


  PgmPrintln("...starting Network");

#ifdef ESP
  WiFi.mode(WIFI_STA);
  WiFi.hostname(MYHOSTNAME);      // DHCP Hostname (useful for finding device for static lease)
//WiFi.config(staticIP, gateway, subnet);  // (DNS not required)
  WiFi.begin(MYSSID, MYWIFIPASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    PgmPrint(".");
  }
  PgmPrintln("");
  PgmPrint("My IP address: ");
  Serial.println(WiFi.localIP());
#ifndef CENTRALEX
  server.begin();
#endif
#else
   if (Ethernet.begin(mac) == 0) {
    PgmPrintln("Failed to configure Ethernet using DHCP");
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  // give the Ethernet shield a second to initialize:
  delay(1000);
  PgmPrint("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    PgmPrint(".");
  }
Serial.println();
#ifndef CENTRALEX
  server.begin();
#endif
#endif

#ifdef CENTRALEX
  Serial.print("connecting to centralex server...");
  if(client.connect(CENTRALEXHOST,CENTRALEXPORT)){
    Serial.println("success!");
  }else{
    Serial.println("failed!");
  }
#else
  server.begin();
#endif





  PgmPrintln("Welcome to eTelex!");
#ifdef USE_SDCARD
  PgmPrint("Free RAM: ");
  Serial.println(FreeRam());
#endif

  state=STATE_RUHEZUSTAND;

  bit_pos=0;
  last_timestamp=millis();
  st_timestamp=millis();
  new_char_recieved=false;

  currentDigit=0;
  currentNumberPos=0;
#ifdef USEIRQ
  attachInterrupt(digitalPinToInterrupt(RECIEVE_PIN), onlinepinchange, RISING);//CHANGE);
#else
  PgmPrint("no IRQ Version");
#endif
}

void storeMainSocket(){
#ifndef ESP
  for (int i = 0; i < MAX_SOCK_NUM; i++) {
    uint8_t s = W5100.readSnSR(i);
    if(s == 0x17) {
      currentSocknum=i;
    }
  }
#endif
}
void handleClientsWhileConnected(){ //check for new clients which try to connect while connection is already in progress
#ifndef ESP
  for (int i = 0; i < MAX_SOCK_NUM; i++) {
    if(i!=currentSocknum){ //only the other ports
      uint8_t s = W5100.readSnSR(i);
      if(s == 0x17) { //connected -> disconnect
        send(i,(unsigned char*)"occ\r\n",5);
        disconnect(i);
      }
      if(s == 0x0) { // free -> listening
        socket(i,SnMR::TCP,MYPORT,0);
        listen(i);
      }
      if(s == 0x1C) { // disconnected -> free
        close(i);
      }
    }
  }
#endif
}


void loop() {
#ifdef CENTRALEX
  if(!client.connected()){//TODO retry timeout
    logged_in = false;
    client.stop();
    PgmPrint("centralex server ended;reconnecting...");
    if(client.connect(CENTRALEXHOST,CENTRALEXPORT)){
      PgmPrintln("success!");
    }else{
      PgmPrintln("failed!");
      delay(5000);
    }
  }else{
    if(logged_in==false){//TODO retry timeout
      PgmPrintln("logging in");
      //char login[] = {14,'l','i',':','2','2','2','2','2',',','1','2','3','4',';',15};
      //client.print(login);
      client.print("\016li:");
      for(byte address = NUMBER_EEPROM;address<NUMBER_EEPROM+NUMBER_EEPROM_SIZE;address++){
        byte value=EEPROM.read(address);
        if(value==0x00){
          break;
        }else if(value==0xFF&&address==NUMBER_EEPROM){
          Serial.println("number is not configured");
          break;
        }else{
          Serial.print((int)value);
          Serial.print(" -> ");
          Serial.println((char)value);
          client.print((char)value);
        }
      }
      client.print(",");
      Serial.print(",");
      for(byte address = PIN_EEPROM;address<PIN_EEPROM+PIN_EEPROM_SIZE;address++){
        byte value=EEPROM.read(address);
        if(value==0x00){
          break;
        }else if(value==0xFF&&address==PIN_EEPROM){
          Serial.println("pin is not configured");
          break;
        }else{
          Serial.print((int)value);
          Serial.print(" -> ");
          Serial.println((char)value);
          client.print((char)value);
        }
      }
      client.print(";\017");
      logged_in = true; //TODO check if logged in packet is recieved
    }
    if((awaiting_pong == 0)&&(millis()-lastPing>=PINGINTERVAL)){
#ifdef _DEBUG
#ifdef _DEBUGPINGS
      Serial.println();
      PgmPrint("sending ping at ");
      Serial.print(millis()/1000);
      Serial.println("sec");
#endif
#endif
      client.print(ping);
      awaiting_pong = millis();
      lastPing = millis();
    }
    if(client.available()>0){
      char c = client.read();
      if(writeTo==CMD){
        if((int)c==15){
#ifdef _DEBUGPINGS
          Serial.println("writeTo TEXT");
#endif
          writeTo=TEXT;
        }else{
          //Serial.print("cmdBuffer+=");
          //Serial.println(c);
          cmdBuffer[cmdBufferIndex++] = c;
        }
      }else if(writeTo==TEXT){
        if((int)c==14){
#ifdef _DEBUGPINGS
          Serial.println("writeTo CMD");
#endif
          writeTo=CMD;
        }else{
          // Serial.print("textBuffer+=");
          // Serial.println(c);
          textBuffer[textBufferIndex++] = c;
        }
      }else{
        if((int)c==15){
          writeTo=TEXT;
        }else if((int)c==14){
          writeTo=CMD;
        }else{
          writeTo=TEXT;
        }
      }
    }
    if(cmdBufferIndex>0){
      for(int i=0;i<cmdBufferIndex;i++){
        if(cmdBuffer[i]==';'){
#ifdef _DEBUGPINGS
          Serial.print("cmdBuffer: ");
          Serial.println(cmdBuffer);
#endif
          for(int k=0;k<i;k++){
            if(cmdBuffer[k]==':'){
              cmdBuffer[k]=0;
              if(cmdBuffer[0]!=0){
#ifdef _DEBUGPINGS
                Serial.print("response code: ");
                Serial.println(cmdBuffer);
#endif
                switch(String(cmdBuffer).toInt()){
                  case 0:
#ifdef _DEBUGPINGS
                    Serial.println("got pong");
#endif
                    awaiting_pong=0;
                    break;
                  default:
                    Serial.print("unknown response code: ");
                    Serial.println(cmdBuffer[k]);
                }
              }
              break;
            }
          }
          for(int k=0;k<CMDBUFFERSIZE;k++){
            if((k+i+1)<CMDBUFFERSIZE){
              cmdBuffer[k]=cmdBuffer[k+i+1];
            }else{
              cmdBuffer[k]=0;
            }
          }
          cmdBufferIndex-=(i+1);
          break;
        }
      }
    }
  }
#endif
  switch (state){

    case STATE_RUHEZUSTAND:
      baudot_recieving_mode=BAUDOT_MODE_UNDEFINED;
      baudot_sending_mode=BAUDOT_MODE_UNDEFINED;
      digitalWrite(SEND_PIN, LOW);
      digitalWrite(COMMUTATE_PIN, LOW);
      if(digitalRead(RECIEVE_PIN)==0){ // AT
        delay(5);
        if(digitalRead(RECIEVE_PIN)==0){// immernoch AT
          delay(5);
          if(digitalRead(RECIEVE_PIN)==0){// immernoch AT
            last_timestamp=millis();
            state=STATE_ANFANGSSIGNAL;
#ifdef _DEBUG
           PgmPrintln("STATE_ANFANGSSIGNAL");
#endif
          }
        }
      }
#ifndef CENTRALEX
      client = server.available();
      if (client) {
#ifdef _DEBUG
        PgmPrintln("client connecting!");
#endif
        storeMainSocket();

        state=STATE_ONLINE;
        digitalWrite(COMMUTATE_PIN, HIGH);
        delay(1000);
        recieving=true;
      }//end client connecting
#endif

    break;

    case STATE_ANFANGSSIGNAL:
      digitalWrite(SEND_PIN, LOW);
      digitalWrite(COMMUTATE_PIN, LOW);
      if(millis()>last_timestamp+1000){
           state=STATE_WAHLAUFFORDERUNGSSIGNAL;
           last_timestamp=millis();
#ifdef _DEBUG
           PgmPrintln("STATE_WAHLAUFFORDERUNGSSIGNAL");
#endif
      }
    break;

    case STATE_WAHLAUFFORDERUNGSSIGNAL:
      digitalWrite(SEND_PIN, HIGH);
      digitalWrite(COMMUTATE_PIN, LOW);
      currentNumberPos=0;
      if(millis()>last_timestamp+25){
            digitalWrite(SEND_PIN, LOW);
            delay(10);
            last_timestamp=millis();
            state=STATE_WARTE_AUF_ZIFFER;
#ifdef _DEBUG
            PgmPrintln("STATE_WARTE_AUF_ZIFFER: 0");
#endif
      }
    break;

    case STATE_WARTE_AUF_ZIFFER:
      currentDigit=0;
      digitalWrite(SEND_PIN, LOW);
      digitalWrite(COMMUTATE_PIN, LOW);
      if(digitalRead(RECIEVE_PIN)==1){// wahlimpuls
            last_timestamp=millis();
            state=STATE_WAHLIMPULS;
      }
      //todo check number
      if(currentNumberPos>0 && millis()>last_timestamp+5000){
           last_timestamp=millis();
           state=STATE_WAHL_ENDE;
#ifdef _DEBUG
           PgmPrintln("STATE_WAHL_ENDE");
#endif
      }

    break;

    case STATE_WAHLIMPULS:
      digitalWrite(SEND_PIN, LOW);
      digitalWrite(COMMUTATE_PIN, LOW);
      if(digitalRead(RECIEVE_PIN)==0){// wahlimpuls_ende
            currentDigit++;
            last_timestamp=millis();
            state=STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS;
#ifdef _DEBUG
            PgmPrintln("STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS");
#endif
      }
      if(millis()>last_timestamp+30000){
            last_timestamp=millis();
            state=STATE_RUHEZUSTAND;
#ifdef _DEBUG
           PgmPrintln("Timeout during dialimpulse");
           PgmPrintln("STATE_RUHEZUSTAND");
#endif
      }

    break;

    case STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS:
      digitalWrite(SEND_PIN, LOW);
      digitalWrite(COMMUTATE_PIN, LOW);
      if(digitalRead(RECIEVE_PIN)==1){// wahlimpuls
            last_timestamp=millis();
            state=STATE_WAHLIMPULS;
      }
      if(millis()>last_timestamp+200){
            last_timestamp=millis();
            state=STATE_WARTE_AUF_ZIFFER;
            if(currentDigit==0){
#ifdef _DEBUG
              PgmPrintln("kein impuls");
              PgmPrintln("STATE_RUHEZUSTAND");
#endif
              state=STATE_RUHEZUSTAND;
            }
            if(currentDigit==10){
              currentDigit=0;
            }
           number[currentNumberPos++]='0'+currentDigit;
           if(currentNumberPos>=NUMBER_BUFFER_LENGTH){
#ifdef _DEBUG
             PgmPrintln("number to long!");
#endif
             currentNumberPos=NUMBER_BUFFER_LENGTH-1;
           }
#ifdef _DEBUG
           PgmPrint("ZIFFER:");
           Serial.println(currentDigit);
           PgmPrint("STATE_WARTE_AUF_ZIFFER:");
           Serial.println(currentNumberPos);
#endif
      }

    break;


   case STATE_WAHL_ENDE:
#ifdef CENTRALEX
      Serial.print("configuringNumber: ");
      Serial.println(configuringNumber);
      Serial.print("configuringPin: ");
      Serial.println(configuringPin);
      if(configuringNumber||configuringPin){
        if(configuringNumber){
          Serial.print("configured number is:");
          Serial.println(number);
          if(currentNumberPos<=NUMBER_EEPROM_SIZE){
            for(int i=0;i<NUMBER_EEPROM_SIZE;i++){
              if(i<currentNumberPos){
                EEPROM.write(NUMBER_EEPROM+i,number[i]);
              }else{
                EEPROM.write(NUMBER_EEPROM+i,0);
              }
            }
            digitalWrite(COMMUTATE_PIN, HIGH);
            delay(1000);
            char str[] = "bitte eigenen pin waehlen\r\n";
            for(int i=0;i<27;i++){
              sendAsciiAsBaudot(str[i]);
            }
            delay(1000);
            state=STATE_RUHEZUSTAND;
            digitalWrite(SEND_PIN, LOW);
            digitalWrite(COMMUTATE_PIN, LOW);
            delay(1000);
          }else{

          }
          configuringNumber=false;
        }else if(configuringPin){
          Serial.print("configured pin is:");
          Serial.println(number);
          if(currentNumberPos<=PIN_EEPROM_SIZE){
            for(int i=0;i<PIN_EEPROM_SIZE;i++){
              if(i<currentNumberPos){
                EEPROM.write(PIN_EEPROM+i,number[i]);
              }else{
                EEPROM.write(PIN_EEPROM+i,0);
              }
            }
            logged_in = false;
            digitalWrite(COMMUTATE_PIN, HIGH);
            delay(1000);
            char str[] = "configuration abgeschlossen.\r\n";
            for(int i=0;i<30;i++){
              sendAsciiAsBaudot(str[i]);
            }
            delay(1000);
            state=STATE_RUHEZUSTAND;
            digitalWrite(SEND_PIN, LOW);
            digitalWrite(COMMUTATE_PIN, LOW);
            delay(1000);
          }else{

          }
          configuringPin=false;
          for(int i=0;i<NUMBER_EEPROM_SIZE+PIN_EEPROM_SIZE+4;i++){
            Serial.print((int)EEPROM.read(i));
            Serial.print(", ");
          }
          Serial.print("");
        }
      }else{
#endif
        number[currentNumberPos]=0;
#ifdef _DEBUG
       PgmPrint("gewählte Nummer war: ");
       Serial.println(number);
#endif
#ifdef CENTRALEX
      if(currentNumberPos==1&&strncmp(number,"0",1)==0){ //if(currentNumberPos==<<str.length>>&&strncmp(number,<<str>>,<<str.length>>)==0){
        Serial.println("starting configuring");
        configuringPin = true;
        configuringNumber = true;
        digitalWrite(COMMUTATE_PIN, HIGH);
        delay(1000);
        char str[] = "bitte eigene nummer waehlen\r\n";
        for(int i=0;i<30;i++){
          sendAsciiAsBaudot(str[i]);
        }
        delay(1000);
        state=STATE_RUHEZUSTAND;
        digitalWrite(SEND_PIN, LOW);
        digitalWrite(COMMUTATE_PIN, LOW);
        delay(1000);
      }else{
#endif
        digitalWrite(SEND_PIN, LOW);
        digitalWrite(COMMUTATE_PIN, LOW);
        state=STATE_LOOKUP;
        recieving=true;
#ifdef CENTRALEX
      }
    }
#endif
    break;


case STATE_LOOKUP:
  if(lookupNumber(number)){
#ifdef _DEBUG
    PgmPrint("lookup ok: ");
    Serial.print(lookup_host);
    PgmPrint(":");
    Serial.print(lookup_port);
    PgmPrint("*");
    Serial.print(lookup_durchwahl);
    PgmPrint("t");
    Serial.println(lookup_type);
    PgmPrintln("STATE_CONNECTING");
#endif
  state=STATE_CONNECTING;
  }else{
#ifdef _DEBUG
    PgmPrintln("nummer nicht gefunden");
    PgmPrintln("STATE_BUSY");
#endif
    state=STATE_BUSY;
  }


    break;

    case STATE_CONNECTING:
#ifdef _DEBUG
  PgmPrintln("connecting to remote");
#endif
  if (client.connect(lookup_host.c_str(), lookup_port)) {
#ifdef _DEBUG
    PgmPrintln("connected to remote");
#endif

    storeMainSocket();

    if(lookup_durchwahl>0){
#ifdef _DEBUG
      Serial.print('*');
      Serial.print(lookup_durchwahl);
      Serial.print('*');
      Serial.println();
#endif
      client.print('*');
      client.print(lookup_durchwahl);
      client.print('*');
    }
    state=STATE_ONLINE;
    recieving=true;
    digitalWrite(COMMUTATE_PIN, HIGH);
    delay(1000);
  }else{
#ifdef _DEBUG
    PgmPrintln("connecting to remote failed");
    PgmPrintln("STATE_BUSY");
#endif
    state=STATE_BUSY;
  }

    break;



    case STATE_BUSY: //ump=1, SSR=1, delay 2000 -> 0
      digitalWrite(COMMUTATE_PIN, HIGH);
      delay(200);
      PgmPrintln("STATE_DISCONNECT");
      state=STATE_DISCONNECT;
    break;


    case STATE_LOCALMODE:
      if(!digitalRead(RECIEVE_PIN)){ // wollen wir trennen
        st_timestamp=millis();
      }else if(millis()>st_timestamp+ST_DURATION){
#ifdef _DEBUG
        PgmPrintln("We want to disconnect in localmode!");
#endif
        state=STATE_DISCONNECT;
      }
    break;



    case STATE_DISCONNECT:
#ifdef _DEBUG
      PgmPrintln("Disconnecting!");
      PgmPrintln("STATE_RUHEZUSTAND");
#endif
      client.stop();
      state=STATE_RUHEZUSTAND;
      digitalWrite(SEND_PIN, LOW);
      digitalWrite(COMMUTATE_PIN, LOW);
      delay(1000);
    break;


      case STATE_ONLINE:
      digitalWrite(COMMUTATE_PIN, HIGH);

      handleClientsWhileConnected();//check for new clients which try to connect while connection is already in progress




      if(!client.connected()){ // hat der client getrennt ?
        client.stop();
#ifdef _DEBUG
        PgmPrintln("Client disconnected!");
#endif
        state=STATE_DISCONNECT;
      }

      if(!digitalRead(RECIEVE_PIN)){ // wollen wir trennen
        st_timestamp=millis();
      }else if(millis()>st_timestamp+ST_DURATION){
#ifdef _DEBUG
        PgmPrintln("We want to disconnect!");
#endif
        state=STATE_DISCONNECT;
      }
    break;

  }//END of switch




#ifdef _DEBUGSOCKETS
 for (int i = 0; i < MAX_SOCK_NUM; i++) {
    uint8_t s = W5100.readSnSR(i);
    Serial.print(s,HEX);
    Serial.print("\t");
 }

Serial.print(currentSocknum);
Serial.println();
#endif







if(state==STATE_ONLINE || state==STATE_LOCALMODE){ // send and recieve tw-39
    if(bit_pos==0){//wait for startbit
#ifdef USEIRQ
      //do nothing (done in pinchangeinterrupt)
#else
      if(recieving){
        if( digitalRead(RECIEVE_PIN)){ //startbit
          last_timestamp=millis();
          bit_pos++;
        }
      }
#endif
    }else{// we had the startbit and are now waiting for further bits
      unsigned long timestamp=millis();
      long diff=timestamp-last_timestamp;
      if((bit_pos<6) && (diff>= (STARTBIT_DURATION+(DATABIT_DURATION*(bit_pos-1))+(DATABIT_DURATION*SAMPLEPOS/120)))){

#ifdef ESP
if(bit_pos==1)noInterrupts();
#endif

#ifdef _DEBUGTIMINGS
    Serial.print(bit_pos,DEC);
    PgmPrint("\t");
    Serial.print(last_timestamp,DEC);
    PgmPrint("\t");
    Serial.print(timestamp,DEC);
    PgmPrint("\t");
    Serial.print(diff,DEC);
    PgmPrint("\t");
    Serial.print((STARTBIT_DURATION+(DATABIT_DURATION*(bit_pos-1))+(DATABIT_DURATION*SAMPLEPOS/120)));
    PgmPrint("\n");
#endif

      recieve_buf=recieve_buf<<1;
      bit_pos++;
       if(digitalRead(RECIEVE_PIN)){
       }else{
        recieve_buf=recieve_buf+1;
      }
      if(bit_pos==6){
        new_char_recieved=true;
        recieved_char=recieve_buf;
        recieve_buf=0;
#ifdef ESP
        interrupts();
#endif
      }

     }
      if(bit_pos==6 && (diff>= (STARTBIT_DURATION+DATABIT_DURATION*6))){//1/3 stopbit to early to not miss anything
//digitalWrite(DEBUG_PIN, LOW);
#ifdef _DEBUGTIMINGS
    Serial.print(bit_pos,DEC);
    PgmPrint("\t");
    Serial.print(last_timestamp,DEC);
    PgmPrint("\t");
    Serial.print(timestamp,DEC);
    PgmPrint("\t");
    Serial.print(diff,DEC);
    PgmPrint("\t");
    Serial.print((STARTBIT_DURATION+(DATABIT_DURATION*(bit_pos-1))+(DATABIT_DURATION*SAMPLEPOS/120)));
    PgmPrint("\n");

    Serial.println(recieved_char,BIN);
#endif
        bit_pos=0;
     }
    }
}//send and recieve TW-39


  if (new_char_recieved) {
    char c=baudotToAscii(recieved_char);
//bit_pos=0;


#ifdef ITELEX
  if(c!=BAUDOT_CHAR_UNKNOWN && c!='\016' && c!='\017' &&c!=BAUDOT_MODE_BU && c!=BAUDOT_MODE_ZI && c!=0){
    if(client.connected()){
      client.print(c);
    }
    Serial.print(c);
  }
#else
  if(client.connected()){
      client.print(c);
  }
  Serial.print(c);
#endif
  new_char_recieved=false;
 }


   if (Serial.available() > 0) {
    if(state==STATE_ONLINE||state==STATE_LOCALMODE){
        sendAsciiAsBaudot(Serial.read());
    }else{
      #ifdef _DEBUG
      PgmPrintln("STATE_LOCALMODE");
      #endif
      digitalWrite(COMMUTATE_PIN, HIGH);
      delay(1000);
      state=STATE_LOCALMODE;
      recieving=true;
    }
   }

#ifdef CENTRALEX
  if(textBufferIndex > 0){
    char c = textBuffer[0];
    for(int i=0;i<--textBufferIndex;i++){
      textBuffer[i] = textBuffer[i+1];
    }
#else
  if(client.available() > 0){
    char c = client.read();
#endif
     Serial.print(c);
     if(state==STATE_ONLINE||state==STATE_LOCALMODE){
       sendAsciiAsBaudot(c);
     }
   }
#ifndef ESP
  Ethernet.maintain();
#endif
}
