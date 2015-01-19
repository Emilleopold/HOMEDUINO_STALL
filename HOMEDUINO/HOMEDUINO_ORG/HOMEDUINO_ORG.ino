//Ver.: "homeduino_small_07.ino / Stand: 2015.01.06 / Verfasser: Eugen Stall
 
//hier ist immer die aktuelle Version:
//http://www.stall.biz/?project=der-homeduino-2-0-als-vielseitiges-lanwlan-sensormodul-fur-die-homematic&preview=true&preview_id=1115&preview_nonce=9162dd6679
//fuer Arduino Mega 2560, UNO, Duemilanove  mit Arduino 1.5.8
//diese Software steuert referenziert die signale an den Arduino-pins mit entsprechenden systemvariablen in der Homematic ccu
//Quellen:Arduino website plus http://arduino.cc/en/Tutorial/WebClient  und ...
// http://tushev.org/articles/arduino/item/52-how-it-works-ds18b20-and-arduino und ...
 
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>      // Comes with Arduino IDE
#include <OneWire.h>   //für Temperatursensoren DS18B20 http://www.hacktronics.com/code/OneWire.zip
#include <NewPing.h>   //für Ultraschallsensoren SR04 https://arduino-new-ping.googlecode.com/files/NewPing_v1.5.zip
#include "DHT.h"       //für Temperatursensoren SHT22 https://github.com/adafruit/DHT-sensor-library/archive/master.zip
#include <AS_BH1750.h> //für I2C-Luxmeter https://github.com/hexenmeister/AS_BH1750/archive/master.zip
#include <SFE_BMP180.h>//für I2C-Barometer https://github.com/sparkfun/BMP180_Breakout/archive/master.zip
 
byte ccu[] = { 192, 168, 178, 50 };                   //das ist die IP der CCU                                    <<user-eingabe<< 
byte mac[] = { 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };  //muss nur bei mehreren homeduinos geändert werden !        <<user-eingabe<<
byte homeduino[] = { 192, 168, 178, 52 };             //das ist die feste IP des Homeduino, wenn DHCP versagt     <<user-eingabe<< 
const byte board = 54;       //"14" = UNO/Duemilanove;  "54" = MEGA2560                                           <<user-eingabe<< 
String homeduino_nummer = "x";  // indiv. Bezeichnung für diese homeduino-box                                     <<user-eingabe<< 
String hm_systemvariable = "homeduino_" + homeduino_nummer +"_";
 
//Input-Kennung: hier wird die Funktion der Eingänge D0 bis D9 festgelegt 
byte iomodus_D[10] = {0, 0, 
                      1,   //D2  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall; "2" = Infrarot;  <<user-eingabe<< 
                      1,   //D3  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall; "2" = 433Mhz-Rx; <<user-eingabe<< 
                      1,   //D4  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall;                  <<user-eingabe<< 
                      1,   //D5  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall;                  <<user-eingabe<< 
                      1,   //D6  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall;                  <<user-eingabe<< 
                      0,   //D7  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall;                  <<user-eingabe<< 
                      0,   //D8  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall;                  <<user-eingabe<< 
                      0    //D9  :"0" = digital_in; "1" = 1wire; "3" = SHTxx; "4" = Ultraschall;                  <<user-eingabe<< 
                       };                                       
 
//hier wird die Funktion der Eingänge A0 bis A5 festgelegt
byte iomodus_A[6] = {10,   //A0  :"0" = digital_in; "10" = analogin; "11" = NTC; "1" = 1wire;                     <<user-eingabe<< 
                     10,   //A1  :"0" = digital_in; "10" = analogin; "11" = NTC; "1" = 1wire;                     <<user-eingabe<<  
                     10,   //A2  :"0" = digital_in; "10" = analogin; "11" = NTC; "1" = 1wire;                     <<user-eingabe<<   
                     11,   //A3  :"0" = digital_in; "10" = analogin; "11" = NTC; "1" = 1wire;                     <<user-eingabe<<   
                     11,   //A4  :"0" = digital_in; "10" = analogin; "11" = NTC; "1" = 1wire;                     <<user-eingabe<<   
                     11    //A5  :"0" = digital_in; "10" = analogin; "11" = NTC; "1" = 1wire;                     <<user-eingabe<<   
                      };                                              
//hier werden Sensoren am I2C-Eingang aktiviert
byte iomodus_baro  = 1;   //"0" = N.C.;  "1" = BMP180                     <<user-eingabe<< 
byte iomodus_lux  = 1;   //"0" = N.C.; "1" = BH1750;                      <<user-eingabe<< 
 
 
 
//hier werden die zuletzt gesendeten sytemvariablen gespeichert
boolean last_digitalin_value_D[10];
float last_digital_value_D[10];
boolean last_digitalin_value_A[6];
float last_analogin_value_A[6];
float last_IR_value;
float last_RF_value;
float last_lux_value;
double last_baro_value;
double last_baroT_value;
 
boolean complete_loop =1;  // wenn 1, dann einmal komplett durchlaufen
 
boolean value;
String befehl;
String I;
int analogwert;
 
float tempNTC;
float B_wert = 3950; //aus dem Datenblatt des NTC //<<user-eingabe<<
float Tn = 298.15; //25°Celsius in °Kelvin 
float Rv = 10000; //Vorwiderstand
float Rn = 10000; //NTC-Widerstand bei 25°C
float Rt ; 
float temp_tur;
float humidity;
 
float delta_onewire = 0.2; //Deltas für Sendeauslösung 
float delta_sht = 0.2;     
float delta_us = 2;
float delta_analog = 2;
float delta_ntc = 0.2;
float delta_lux = 20;
double delta_baro = 0.2;
double delta_baroT = 0.5;
 
long duration, cm; //variable für Ultraschallsensor
 
unsigned long next_full_loop = 0;
unsigned long delta_time = 3600000; // jede Stunde werden alle Inputs aktualisiert
unsigned long delta_tx = 500;  //in ms, minimaler Abstand der Telegramme an die CCU
unsigned long next_tx = 0;
//**************************************************************************************************  
EthernetClient client;  // Initialize the Ethernet client library
 
AS_BH1750 sensor;   //Initialize BH1750 Luxmeter library
 
#define ALTITUDE 299.0 // seehoehe in metern
SFE_BMP180 pressure;
char status;
double T,P,p0;
 
 
void setup() 
{Serial.begin(9600); // start the Ethernet connection:
 if (Ethernet.begin(mac) == 0) 
   {Serial.println("Failed to configure Ethernet using DHCP"); Ethernet.begin(mac, homeduino);}
 delay(1000);// give the Ethernet shield a second to initialize:
 Serial.println("connecting..."); // if you get a connection, report back via serial:
 if (client.connect(ccu, 8181)) {}
   else {Serial.println("connection failed");} // if you didn't get a connection to the server:
 client.stop(); 
 char myIpString[24];  //IP auslesen
 IPAddress myIp = Ethernet.localIP();
 sprintf(myIpString, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);   
 I = myIpString;
 befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "IP" + "').State('" + I + "')";
 set_sysvar();
}
 
//**************************************************************************************************  
void loop()
{if (millis() > next_full_loop)     //mindestens jede Stunde eine komplette Aktualisierung
   {complete_loop = 1; next_full_loop = millis() + delta_time; 
    if (next_full_loop < millis()) {complete_loop = 0;}  //wichtig wegen Zahlensprung von millis() alle 50 Tage
   } 
 //**************************************************************************************************   
 for (int i = 2; i < 10; i++)  //behandlung digitaleingänge D2 bis D9
   {if (iomodus_D[i] == 0)
     {pinMode(i, INPUT); 
      value =digitalRead(i);
      if ((!value == last_digitalin_value_D[i]) || complete_loop) 
        {I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "D" + I + "').State(" + value + ")";
         set_sysvar();
         last_digitalin_value_D[i] = value;
        } 
     } 
  }
//**************************************************************************************************  
 for (int i = 2; i < 10; i++)  //behandlung onewire D2 bis D9
   {if (iomodus_D[i] == 1)
     {OneWire ds(i); 
      #define DS18S20_ID 0x10
      #define DS18B20_ID 0x28 
 
      byte present = 0;
      byte data[12];
      byte addr[8];
      temp_tur = 1000.0;
      if (!ds.search(addr)) { ds.reset_search(); temp_tur = -1000.0; } //find a device
      if ((OneWire::crc8( addr, 7) != addr[7]) && (temp_tur > -1000.0)) {temp_tur = -1000.0; }
      if ((addr[0] != DS18S20_ID && addr[0] != DS18B20_ID)&& (temp_tur > -1000.0)) {temp_tur = -1000.0;}
      if (temp_tur > -1000.0) 
        {ds.reset(); 
         ds.select(addr); 
         ds.write(0x44, 1); // Start conversion
         delay(850); // Wait some time...
         present = ds.reset(); 
         ds.select(addr);
         ds.write(0xBE); // Issue Read scratchpad command
         for ( int k = 0; k < 9; k++) { data[k] = ds.read(); } // Receive 9 bytes
         temp_tur = ( (data[1] << 8) + data[0] )*0.0625; // Calculate temperature value 18B20
         //temp_tur = ( (data[1] << 8) + data[0] )*0.5    // Calculate temperature value 18S20
        }
      if ((temp_tur > (last_digital_value_D[i] + delta_onewire)) || (temp_tur < (last_digital_value_D[i] - delta_onewire)) || complete_loop) 
        {
          I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "D" + I + "').State(" + temp_tur + ")";
         set_sysvar();
         last_digital_value_D[i] = temp_tur;
        } 
     } 
  }  
//**************************************************************************************************  
 for (int i = 0; i < 6; i++)  //behandlung onewire A0 bis A5
   {if (iomodus_A[i] == 1)
     {OneWire ds(i+54); 
      #define DS18S20_ID 0x10
      #define DS18B20_ID 0x28 
 
      byte present = 0;
      byte data[12];
      byte addr[8];
      temp_tur = 1000.0;
      if (!ds.search(addr)) { ds.reset_search(); temp_tur = -1000.0; } //find a device
      if ((OneWire::crc8( addr, 7) != addr[7]) && (temp_tur > -1000.0)) {temp_tur = -1000.0; }
      if ((addr[0] != DS18S20_ID && addr[0] != DS18B20_ID)&& (temp_tur > -1000.0)) {temp_tur = -1000.0;}
      if (temp_tur > -1000.0) 
        {ds.reset(); 
         ds.select(addr); 
         ds.write(0x44, 1); // Start conversion
         delay(850); // Wait some time...
         present = ds.reset(); 
         ds.select(addr);
         ds.write(0xBE); // Issue Read scratchpad command
         for ( int k = 0; k < 9; k++) { data[k] = ds.read(); } // Receive 9 bytes
         temp_tur = ( (data[1] << 8) + data[0] )*0.0625; // Calculate temperature value 18B20
         //temp_tur = ( (data[1] << 8) + data[0] )*0.5    // Calculate temperature value 18S20
        }
      if ((temp_tur > (last_analogin_value_A[i] + delta_onewire)) || (temp_tur < (last_analogin_value_A[i] - delta_onewire)) || complete_loop) 
        {
          I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "A" + I + "').State(" + temp_tur + ")";
         set_sysvar();
         last_analogin_value_A[i] = temp_tur;
        } 
     } 
  }   
 //**************************************************************************************************   for (int i = 2; i < 10; i++)  //behandlung onewire D2 bis D9
for (int i = 2; i < 10; i++)  //behandlung DHT22 D2 bis D9
  {if (iomodus_D[i] == 3)
     {DHT dht(i, DHT22);   //je nach verwendetem sensor "DHT11" oder "DHT22" (AM2302) oder "DHT 21" (AM2301)
      dht.begin();
      delay(2000); // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      humidity = dht.readHumidity();  // Read temperature as Celsius
      temp_tur = dht.readTemperature();  
      if (isnan(humidity) || isnan(temp_tur) )  // Check if any reads failed and 
        {Serial.println("Failed to read from DHT sensor!"); temp_tur = -1000;}
      if ((temp_tur > (last_digital_value_D[i] + delta_sht)) || (temp_tur < (last_digital_value_D[i] - delta_sht)) || complete_loop) 
        {
         I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "D" + I + "').State(" + temp_tur + ")";
         set_sysvar();
          befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "D" + I + "1').State(" + humidity + ")";
         set_sysvar();
         last_digital_value_D[i] = temp_tur;
        } 
     } 
  }   
//**************************************************************************************************  
 for (int i = 2; i < 10; i++)  //behandlung Ultraschallsensor D2 bis D9
   {if (iomodus_D[i] == 4)
     { NewPing sonar(i, i, 200); // NewPing setup of pin and maximum distance.
       unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
       int cm = uS / US_ROUNDTRIP_CM;
       if ((cm > (last_digital_value_D[i] + delta_us)) || (cm < (last_digital_value_D[i] - delta_us)) || complete_loop) 
       { I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "D" + I + "').State(" + cm + ")";
         set_sysvar();
         last_digital_value_D[i] = cm;
       } 
     } 
  }  
//**************************************************************************************************  
for (int i = 0; i < 6; i++)  //behandlung digitaleingänge A0 bis A5
  {if (iomodus_A[i] == 0)
     {pinMode(i +board, INPUT);
      value =digitalRead(i+board); //achtung beim uno sind 14 zu addieren, beim mega 54
      if ((!value == last_digitalin_value_A[i]) || complete_loop) 
        {I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "A" + I + "').State(" + value + ")";
         set_sysvar();
         last_digitalin_value_A[i] = value;
        } 
     } 
  }
//**************************************************************************************************  
for (int i = 0; i < 6; i++)  //behandlung analogeingänge A0 bis A5
  {if (iomodus_A[i] == 10)
     {analogwert =analogRead(i); 
      if ((analogwert > (last_analogin_value_A[i] + delta_analog)) || (analogwert < (last_analogin_value_A[i] - delta_analog)) || complete_loop) 
        {I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "A" + I + "').State(" + analogwert + ")";
         set_sysvar();
         last_analogin_value_A[i] = analogwert;
        } 
     } 
  }  
//**************************************************************************************************
for (int i = 0; i < 6; i++)   //behandlung NTC an A0 bis A5
  {if (iomodus_A[i] == 11)
     {Rt = Rv/((1024.0/analogRead(i))- 1.0);
      tempNTC = (B_wert * Tn / ( B_wert + (Tn * log(Rt/Rn)))) -Tn +25.0 ;
       if ((tempNTC > (last_analogin_value_A[i] + delta_ntc)) || (tempNTC < (last_analogin_value_A[i] - delta_ntc)) || complete_loop) 
        {I = String(i);
         befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "A" + I + "').State(" + tempNTC + ")";
         set_sysvar();
         last_analogin_value_A[i] = tempNTC;
        } 
     } 
  }  
 
//**************************************************************************************************
//behandlung Luxmeter BH1750 an SCL pin21 und SDA pin 20
// for normal sensor resolution (1 lx resolution, 0-65535 lx, 120ms, no PowerDown) use: sensor.begin(RESOLUTION_NORMAL, false);   
if (iomodus_lux == 1)
  {if(!sensor.begin()) {Serial.println("Sensor not present");}
   float lux = sensor.readLightLevel();   delay(1000);
   if ((lux > (last_lux_value + delta_lux)) || (lux < (last_lux_value - delta_lux)) || complete_loop) 
     {befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "lux" + "').State(" + lux + ")";
      set_sysvar();
      last_lux_value = lux;
     } 
   } 
 
//**************************************************************************************************
//behandlung barometer BMP180 an SCL pin21 und SDA pin 20
if (iomodus_baro == 1)
  {if (pressure.begin()) {status = pressure.startTemperature();}
   if (status) {delay(status); status = pressure.getTemperature(T);} //messung T
   if (status) {status = pressure.startPressure(3);} // //messung P mit resolution 0 bis 3
   if (status) {delay(status); status = pressure.getPressure(P,T);}
   if (status) {p0 = pressure.sealevel(P,ALTITUDE);} // umrechnung auf N.N.
      //Serial.print("Hoehe/m:  "); Serial.print(ALTITUDE); Serial.print("  Temperatur/C: ");  Serial.print(T); Serial.print("  Normaldruck /mb: ");  Serial.println(p0); 
   if ((p0 > (last_baro_value + delta_baro)) || (p0 < (last_baro_value - delta_baro)) || complete_loop) 
     {befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "baro" + "').State(" + p0 + ")";
      set_sysvar();
      last_baro_value = p0;
     }
     if ((T > (last_baroT_value + delta_baroT)) || (p0 < (last_baroT_value - delta_baroT)) || complete_loop) 
     {befehl = "GET /xy.exe?antwort=dom.GetObject('" + hm_systemvariable + "baroT" + "').State(" + T + ")";
      set_sysvar();
      last_baroT_value = T;
     }
  }
 
//**************************************************************************************************
 complete_loop = 0;
}
//##############################################################
// subroutine HTTP request absetzen:
void set_sysvar() 
{ //while (millis() < next_tx) {} //warten bis time > next_tx oder timeout
 next_tx = millis() +delta_tx;
 if (client.connect(ccu, 8181)) 
   {Serial.println(befehl);
    client.println(befehl);
    client.println();
    client.stop();
   }
}
