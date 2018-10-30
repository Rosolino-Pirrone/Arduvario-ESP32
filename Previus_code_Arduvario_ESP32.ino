/////////////////////////////////////////        ESP32 MS5611 Sensor        /////////////////////////

#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#define MS5611_ADDRESS (0x77)
#define CMD_RESET (0x1E)
#define CMD_ADC_D1_4096 (0x48)
#define CMD_ADC_D2_4096 (0x58)
#define CMD_ADC_READ (0x00)
#define CMD_PROM_RD_1 (0xA2)
#define CMD_PROM_RD_2 (0xA4)
#define CMD_PROM_RD_3 (0xA6)
#define CMD_PROM_RD_4 (0xA8)
#define CMD_PROM_RD_5 (0xAA)
#define CMD_PROM_RD_6 (0xAC)

uint16_t C_1;
uint16_t C_2;
uint16_t C_3;
uint16_t C_4;
uint16_t C_5;
uint16_t C_6;

uint32_t D_1;
uint32_t D_2;

int64_t dt;
float TEMP, T_2;

int64_t OFF, OFF_2;
int64_t SENS, SENS_2;

float P;

float Valori[2];
float Media_valori_alt[50];
float Media_valori_alt_suono[30];
float Old_media_altitudine;
float Old_media_altitudine_suono;


unsigned long previousMillis = 0;
unsigned long previousMillis_suono = 0;
unsigned long previousMillis_velocita = 0;
unsigned long previousMillis_velocita_suono = 0;
const long interval = 1000;
int previous_time = 180;
float Periodo_beep;
float Tono_beep = 0;
float Durata_beep;
float Tono_beep_disc;
float Media_P;
float Media_P_suono;
float somma;
float somma_suono;

#include <Adafruit_GPS.h>
#include <WiFi.h>
#include "esp_system.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


const char *ssid = "Arduvario";
const char *password = "PasswordArduvario";
String stringa_nmea_GGA;
String stringa_nmea_RMC;
String fix;
String stringa4;
float altitudine;
int eff;

WiFiServer server(4353);
WiFiClient client;

HardwareSerial serial_Display(2);
HardwareSerial mySerial(1);
Adafruit_GPS GPS(&mySerial);



#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"
hw_timer_t *timer = NULL;
int wdtTimeout = 1000;  //time in ms to trigger the watchdog
long loopTime = millis();
long loopTimeVario = millis();
long loopTimeSuono = millis();
long loopTimeSuonoFalse = millis();
int count = 0;
int count_2 = 0;
float Vario_al_secondo;
float Vario_al_secondo_suono;


//////////////////     Funzione interrupt suono      //////////////////

int freq = 2000;
int freq_Discendenza = 2000;
int channel = 0;
int resolution = 8;
bool state;
bool state_disc;

void IRAM_ATTR suonoVario() {
  //Serial.println("Suono");
  ledcWriteTone(channel, freq);
  state = true;
  //Serial.println("state = true");
}

void IRAM_ATTR suonoVarioDiscendenza() {
  // Serial.println("Suono");
  ledcWriteTone(channel, freq_Discendenza);
  state_disc = true;
  //Serial.println("state_disc = true");
}





void setup() {                                 // put your setup code here, to run once:
  float altitudine;
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(25, channel);


  delay(2000);



  SerialBT.begin("Arduvario"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  //while (!Serial); // wait for leo to be ready
  mySerial.begin(9600);
  serial_Display.begin(115200);
  Serial.begin(115200); // this baud rate doesn't actually matter!
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.begin();
  Serial.println("HTTP server started");
  client.setNoDelay(true);



  Serial.println("Get version!");
  mySerial.println(PMTK_Q_RELEASE);

  // you can send various commands to get it started
  //mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  mySerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);

  timer = timerBegin(0, 80, true);                  //timer 0, div 80




  delay(5000);                                 // attendo 5s
  Wire.begin();                                // inizializzo la i2c senza nessun indirizzo, imposto Arduino come master
  Wire.beginTransmission(MS5611_ADDRESS);      // comunico tramite i2c all'indirizzo del sensore
  Wire.write(CMD_RESET);                       // impartisco uno reset
  Wire.endTransmission();                      // fine comunicazione
  delay (3);                                   // attendo 3 ms

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_1);                   // comando per leggere la prom c1
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);         // chiedo la lettura del valore di due byte
  while (Wire.available())                     // attendo che sia pronto il valore restituito
  {
    uint8_t B_H = Wire.read();                 // leggo il primo byte alto
    uint8_t B_L = Wire.read();                 // leggo il secondo byte basso
    C_1 = ((uint16_t)B_H << 8) | B_L;          // unisco i due byte per formare un int di 16 bit
  }

  //Serial.print(" C_1: ");                     // stampo a monitor seriale
  //Serial.println(C_1);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_2);
  Wire.endTransmission();
  //delay (10);

  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_2 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_2: ");
  //Serial.println(C_2);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_3);
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_3 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_3: ");
  //Serial.println(C_3);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_4);
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_4 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_4: ");
  //Serial.println(C_4);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_5);
  Wire.endTransmission();
  //delay (10);

  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_5 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_5: ");
  //Serial.println(C_5);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_6);
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_6 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_6: ");
  //Serial.println(C_6);


  delay(1000);

}


void loop() {

  // put your main code here, to run repeatedly:


  GPS.read();
  if (GPS.newNMEAreceived()) {
    //Serial.println("newNMEAreceived");
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  if (!client.connected()) { // if client not connected

    client = server.available(); // wait for it to connect
    GPS.read();
    if (GPS.newNMEAreceived()) {
      //Serial.println("newNMEAreceived");
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }

    if (count <= 45) {
      Valori_Alt_Temp();       // richiamo la funzione Valori_Alt_Temp
      //Serial.println(Valori[1]);
      Media_valori_alt[count] = Valori[1];        // memorizzo i valori nell'array Media_Valori_Alt
      Media_valori_alt_suono[count_2] = Media_valori_alt[count];
      // Serial.println("Media_valori_alt[count]");
      // Serial.println(Media_valori_alt[count]);
      //Serial.println("Media_valori_alt_suono[count_2]");
      //Serial.println(Media_valori_alt_suono[count_2]);

      //delay (time);                            // pausa di 78 millisecondi



      // somma dei valori e calcolo della media

      somma = somma + Media_valori_alt[count];
      //Serial.println(somma);
      somma_suono = somma_suono + Media_valori_alt_suono[count_2];
      //Serial.println(somma_suono);
      //Serial.println(somma);
    }

    if (loopTime > millis()) loopTime = millis();
    if (millis() - loopTime > 1000) {//Serial.println(millis() - timer > 1000);
      loopTime = millis(); // reset the timer
      unsigned long Tempo = loopTime - previousMillis_velocita;
      previousMillis_velocita = loopTime;
      //Serial.print("Tempo");
      //Serial.println(Tempo);

      Media_P = somma / (count);

      //Serial.println("Media" + String(Media_P));
      altitudine = 44330.0 * (1.0 - pow(Media_P / 1013.25, 0.1903));

      somma = 0;

      //Serial.println("altitudine " + String(altitudine));
      float Vario = altitudine - Old_media_altitudine;    // sottrazione quota calcolo vario in m/s

      Old_media_altitudine = altitudine;                 // media precedente


      if (Vario > 50)
      {
        Vario = 0;
      }
      Vario_al_secondo = Vario / Tempo * 1000;


      String cmd = "POV,E," + String(Vario_al_secondo).substring(0, 4) + ",P," + String(Media_P) + ",T," + String(Valori[0] / 100); // calcolo stringa NMEA OpenVario
      String checkSum0 = String(checkSum(cmd), HEX);

      Serial.println("$" + cmd + "*" + checkSum0);              //Stringa alla seriale
      SerialBT.println("$" + cmd + "*" + checkSum0);
      stringa_nmea_GGA = GPS.stringa_GGA;
      stringa_nmea_RMC = GPS.stringa_RMC;
      Serial.println(stringa_nmea_GGA);          // print a message out in the serial port
      Serial.println(stringa_nmea_RMC);
      SerialBT.println(stringa_nmea_GGA);
      SerialBT.print(stringa_nmea_RMC + "\n");

      int Media_P_1 = int(Media_P * 100);
      int altitudine_1 = int(altitudine);
      int Vario_al_secondo_1 = int(Vario_al_secondo * 100);
      int temp = int(Valori[0] / 100);
      String cmd_1 = "LK8EX1," + String(Media_P_1) + "," + String(altitudine_1) + "," + String(Vario_al_secondo_1).substring(0, 4) + "," + String(temp) + ",999";
      String checkSum2 = String(checkSum(cmd_1), HEX);
      Serial.println("$" + cmd_1 + "*" + checkSum2);              //Stringa alla seriale
      SerialBT.println("$" + cmd_1 + "*" + checkSum2);


      count = 0;

    }

    //////////////////////////////////////////     Codice stringa seriale display     //////////////////////////////////////////


    stringa4 = stringa_nmea_RMC;
    int q;
    for (int i = 0; i < 7; i++)
    {
      q = stringa4.indexOf(",");
      stringa4.remove(0, (q + 1)) ;
    }
    q = stringa4.indexOf(",");
    stringa4.remove(q);
    int altitudine1 = altitudine;


    float velo;
    velo = (stringa4.toFloat() * 1.852 * 10);                  // questi calcoli potranno sembrare errati, servono per la visualizzazione
    int velo2 = int(velo);
    if (Vario_al_secondo >= 0) {
      eff = 999;                                                         //nel display che accetta solo interi
    }
    else if (velo2 / (Vario_al_secondo * 3.6 * (-1)) > 999) eff = 999;
    else
      eff = velo2 / (Vario_al_secondo * 3.6 * (-1));
    int eff2 = int(eff);
    String ROS = "$ROS," + String(Vario_al_secondo).substring(0, 4) + "," + String(altitudine1) + "," + String(Valori[0] / 100) + ","  + String(velo2) + "," + fix + "," + String(eff2) + "," + ",";
    serial_Display.print(ROS);     //$ROS
    //  Serial.println(ROS);     //$ROS


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////   Suono     Vario    ////////////////////////////////////////////////////////////
    if (loopTimeSuono > millis()) loopTime = millis();
    if (millis() - loopTimeSuono > 500) {//Serial.println(millis() - timer > 1000);
      loopTimeSuono = millis(); // reset the timer
      unsigned long Tempo_suono = loopTimeSuono - previousMillis_velocita_suono;
      // Serial.print("Tempo suono");
      //Serial.println(Tempo_suono);
      previousMillis_velocita_suono = loopTimeSuono;
      Media_P_suono = somma_suono / (count_2);
      float altitudine_suono = 44330.0 * (1.0 - pow(Media_P_suono / 1013.25, 0.1903));
      somma_suono = 0;
      float Vario_suono = altitudine_suono - Old_media_altitudine_suono;    // sottrazione quota calcolo vario in m/s
      Old_media_altitudine_suono = altitudine_suono;                 // media precedente
      if (Vario_suono > 50)
      {
        Vario_suono = 0;
      }
      Vario_al_secondo_suono = Vario_suono / Tempo_suono * 1000;
      //Serial.println(Vario_al_secondo_suono );
      count_2 = 0;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
  Periodo_beep = Vario_al_secondo_suono * 100;
  if (Periodo_beep > 600) Periodo_beep = 600;
  Periodo_beep = map(Periodo_beep, 0, 600, 350, 50);
  Tono_beep = Vario_al_secondo_suono * 100;
  if (Tono_beep > 600) Tono_beep = 600;
  Tono_beep = map(Tono_beep, 0, 600, 1500, 2200);
  Durata_beep =  Vario_al_secondo_suono * 100;
  if (Durata_beep > 600) Durata_beep = 600;
  Durata_beep = map(Durata_beep, 0, 600, 380, 50);
  Tono_beep_disc = Vario_al_secondo_suono * 100;
  if (Tono_beep_disc < -600) Durata_beep = -500;
  Tono_beep_disc = map(Tono_beep_disc, 0, -600, 1250, 800);
  freq = Tono_beep;
  freq_Discendenza = Tono_beep_disc;



    if (Vario_al_secondo_suono >= 0.15) {

      if (state == false)
      {
        if (state_disc == true) {
          ledcWriteTone(channel, 0);
          state_disc = false;

        }
        if (millis() - loopTimeVario > Periodo_beep) {
          loopTimeVario = millis(); // reset the timer
          timerAttachInterrupt(timer, &suonoVario, true);  //attach callback
          timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
          timerAlarmEnable(timer);                          //enable interrupt
          //state = true;
          //Serial.println("state true");
        }
      }
    }

    if (state == true) {
      if (millis() - loopTimeVario > Periodo_beep) {
        loopTimeVario = millis(); // reset the timer
        ledcWriteTone(channel, 0);
        state = false;

      }
    }


    if (Vario_al_secondo_suono <= -1.8 && state_disc == false)
    {


      timerAttachInterrupt(timer, &suonoVarioDiscendenza, true);  //attach callback
      timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
      timerAlarmEnable(timer);                          //enable interrupt


    }
    else if (Vario_al_secondo_suono <= -1.8 && state_disc == true) {
      if (millis() - loopTimeSuonoFalse > 500) {
        loopTimeSuonoFalse = millis(); // reset the
        timerAttachInterrupt(timer, &suonoVarioDiscendenza, true);  //attach callback
        timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
        timerAlarmEnable(timer);
      }
    }

    if (Vario_al_secondo_suono < 0.15 && Vario_al_secondo_suono > -1.8) {
      if (state_disc == true) {
        if (millis() - loopTimeSuonoFalse > 1000) {
          loopTimeSuonoFalse = millis(); // reset

          ledcWriteTone(channel, 0);
          state_disc = false;

        }
      }
    }



    count++;
    count_2++;
    //Serial.println(count);
    //Serial.println(count_2);
    return;

  }



  if (count <= 45) {
    Valori_Alt_Temp();       // richiamo la funzione Valori_Alt_Temp
    //Serial.println(Valori[1]);
    Media_valori_alt[count] = Valori[1];        // memorizzo i valori nell'array Media_Valori_Alt
    Media_valori_alt_suono[count_2] = Media_valori_alt[count];
    // Serial.println("Media_valori_alt[count]");
    // Serial.println(Media_valori_alt[count]);
    //Serial.println("Media_valori_alt_suono[count_2]");
    //Serial.println(Media_valori_alt_suono[count_2]);

    //delay (time);                            // pausa di 78 millisecondi



    // somma dei valori e calcolo della media

    somma = somma + Media_valori_alt[count];
    //Serial.println(somma);
    somma_suono = somma_suono + Media_valori_alt_suono[count_2];
    //Serial.println(somma_suono);
    //Serial.println(somma);
  }

  if (loopTime > millis()) loopTime = millis();
  if (millis() - loopTime > 1000) {//Serial.println(millis() - timer > 1000);
    loopTime = millis(); // reset the timer
    unsigned long Tempo = loopTime - previousMillis_velocita;
    previousMillis_velocita = loopTime;
    //Serial.print("Tempo");
    //Serial.println(Tempo);

    Media_P = somma / (count);
    //Serial.println("Media" + String(Media_P));
    altitudine = 44330.0 * (1.0 - pow(Media_P / 1013.25, 0.1903));
    somma = 0;
    //Serial.println("altitudine " + String(altitudine));
    float Vario = altitudine - Old_media_altitudine;    // sottrazione quota calcolo vario in m/s
    Old_media_altitudine = altitudine;                 // media precedente

    if (Vario > 50)
    {
      Vario = 0;
    }
    Vario_al_secondo = Vario / Tempo * 1000;

    String cmd = "POV,E," + String(Vario_al_secondo).substring(0, 4) + ",P," + String(Media_P) + ",T," + String(Valori[0] / 100); // calcolo stringa NMEA OpenVario
    String checkSum0 = String(checkSum(cmd), HEX);

    Serial.println("$" + cmd + "*" + checkSum0);              //Stringa alla seriale
    client.println("$" + cmd + "*" + checkSum0);
    stringa_nmea_GGA = GPS.stringa_GGA;
    stringa_nmea_RMC = GPS.stringa_RMC;

    Serial.println(stringa_nmea_GGA);          // print a message out in the serial port
    Serial.println(stringa_nmea_RMC);

    client.println(stringa_nmea_GGA);
    client.print(stringa_nmea_RMC + "\n");


    int Media_P_1 = int(Media_P * 100);
    int altitudine_1 = int(altitudine);
    int Vario_al_secondo_1 = int(Vario_al_secondo * 100);
    int temp = int(Valori[0] / 100);
    String cmd_1 = "LK8EX1," + String(Media_P_1) + "," + String(altitudine_1) + "," + String(Vario_al_secondo_1).substring(0, 4) + "," + String(temp) + ",999";
    String checkSum2 = String(checkSum(cmd_1), HEX);
    Serial.println("$" + cmd_1 + "*" + checkSum2);              //Stringa alla seriale
    client.println("$" + cmd_1 + "*" + checkSum2);


    count = 0;
  }

  //////////////////////////////////////////     Codice stringa seriale display     //////////////////////////////////////////


  stringa4 = stringa_nmea_RMC;
  int q;
  for (int i = 0; i < 7; i++)
  {
    q = stringa4.indexOf(",");
    stringa4.remove(0, (q + 1)) ;
  }
  q = stringa4.indexOf(",");
  stringa4.remove(q);
  int altitudine1 = altitudine;


  float velo;
  velo = (stringa4.toFloat() * 1.852 * 10);                  // questi calcoli potranno sembrare errati, servono per la visualizzazione
  int velo2 = int(velo);
  if (Vario_al_secondo >= 0) {
    eff = 999;                                                         //nel display che accetta solo interi
  }
  else if (velo2 / (Vario_al_secondo * 3.6 * (-1)) > 999) eff = 999;
  else
    eff = velo2 / (Vario_al_secondo * 3.6 * (-1));
  int eff2 = int(eff);
  String ROS = "$ROS," + String(Vario_al_secondo).substring(0, 4) + "," + String(altitudine1) + "," + String(Valori[0] / 100) + ","  + String(velo2) + "," + fix + "," + String(eff2) + "," + ",";
  serial_Display.print(ROS);     //$ROS
  // Serial.println(ROS);     //$ROS


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  //////////////////////////////////////////////////////////////   Suono     Vario    ////////////////////////////////////////////////////////////
  if (loopTimeSuono > millis()) loopTime = millis();
  if (millis() - loopTimeSuono > 500) {//Serial.println(millis() - timer > 1000);
    loopTimeSuono = millis(); // reset the timer
    unsigned long Tempo_suono = loopTimeSuono - previousMillis_velocita_suono;
    // Serial.print("Tempo suono");
    //Serial.println(Tempo_suono);
    previousMillis_velocita_suono = loopTimeSuono;
    Media_P_suono = somma_suono / (count_2);
    float altitudine_suono = 44330.0 * (1.0 - pow(Media_P_suono / 1013.25, 0.1903));
    somma_suono = 0;
    float Vario_suono = altitudine_suono - Old_media_altitudine_suono;    // sottrazione quota calcolo vario in m/s
    Old_media_altitudine_suono = altitudine_suono;                 // media precedente
    if (Vario_suono > 50)
    {
      Vario_suono = 0;
    }
    Vario_al_secondo_suono = Vario_suono / Tempo_suono * 1000;
    //Serial.println(Vario_al_secondo_suono );
    count_2 = 0;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  Periodo_beep = Vario_al_secondo_suono * 100;
  if (Periodo_beep > 600) Periodo_beep = 600;
  Periodo_beep = map(Periodo_beep, 0, 600, 350, 50);
  Tono_beep = Vario_al_secondo_suono * 100;
  if (Tono_beep > 600) Tono_beep = 600;
  Tono_beep = map(Tono_beep, 0, 600, 1500, 2200);
  Durata_beep =  Vario_al_secondo_suono * 100;
  if (Durata_beep > 600) Durata_beep = 600;
  Durata_beep = map(Durata_beep, 0, 600, 380, 50);
  Tono_beep_disc = Vario_al_secondo_suono * 100;
  if (Tono_beep_disc < -600) Durata_beep = -500;
  Tono_beep_disc = map(Tono_beep_disc, 0, -600, 1250, 800);
  freq = Tono_beep;
  freq_Discendenza = Tono_beep_disc;



  if (Vario_al_secondo_suono >= 0.15) {

    if (state == false)
    {
      if (state_disc == true) {
        ledcWriteTone(channel, 0);
        state_disc = false;

      }
      if (millis() - loopTimeVario > Periodo_beep) {
        loopTimeVario = millis(); // reset the timer
        timerAttachInterrupt(timer, &suonoVario, true);  //attach callback
        timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
        timerAlarmEnable(timer);                          //enable interrupt
        //state = true;
        //Serial.println("state true");
      }
    }
  }

  if (state == true) {
    if (millis() - loopTimeVario > Periodo_beep) {
      loopTimeVario = millis(); // reset the timer
      ledcWriteTone(channel, 0);
      state = false;

    }
  }


  if (Vario_al_secondo_suono <= -1.8 && state_disc == false)
  {


    timerAttachInterrupt(timer, &suonoVarioDiscendenza, true);  //attach callback
    timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
    timerAlarmEnable(timer);                          //enable interrupt


  }
  else if (Vario_al_secondo_suono <= -1.8 && state_disc == true) {
    if (millis() - loopTimeSuonoFalse > 500) {
      loopTimeSuonoFalse = millis(); // reset the
      timerAttachInterrupt(timer, &suonoVarioDiscendenza, true);  //attach callback
      timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
      timerAlarmEnable(timer);
    }
  }

  if (Vario_al_secondo_suono < 0.15 && Vario_al_secondo_suono > -1.8) {
    if (state_disc == true) {
      if (millis() - loopTimeSuonoFalse > 1000) {
        loopTimeSuonoFalse = millis(); // reset

        ledcWriteTone(channel, 0);
        state_disc = false;

      }
    }
  }



  count++;
  count_2++;
  //Serial.println(count);
  //Serial.println(count_2);
}

/////////////////////////////////////////////////////////////// funzione rilevazione D1
uint32_t ghet_d_1()
{
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_D1_4096);
  Wire.endTransmission();
  delay (10);
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDRESS, 3);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_M = Wire.read();
    uint8_t B_L = Wire.read();
    D_1 = ((int32_t)B_H << 16) | ((int32_t)B_M << 8) | B_L;
  }
  return D_1;
}

// funzione rilevazione D2
uint32_t ghet_d_2()
{
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_D2_4096);
  Wire.endTransmission();
  delay (10);
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDRESS, 3);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_M = Wire.read();
    uint8_t B_L = Wire.read();
    D_2 = ((int32_t)B_H << 16) | ((int32_t)B_M << 8) | B_L;
  }
  return D_2;
}

// funzione calcolo Temperatura e Quota memorizzati in un array di 2
float Valori_Alt_Temp()
{
  D_1 = ghet_d_1();
  D_2 = ghet_d_2();

  dt = D_2 - C_5 * pow(2, 8);
  TEMP = 2000 + dt * C_6 / pow(2, 23);
  OFF = C_2 * pow(2, 16) + C_4 * dt / pow(2, 7);
  SENS = C_1 * pow(2, 15) + C_3 * dt / pow(2, 8);
  T_2 = 0;
  OFF_2 = 0;

  if (TEMP < 2000)
  {
    T_2 = pow(dt, 2) / pow(2, 31);
    OFF_2 = 5 * pow((TEMP - 2000), 2) / 2;
    SENS_2 = 5 * pow((TEMP - 2000), 2) / pow(2, 2);
  }

  if (TEMP < -1500)
  {
    OFF_2 = OFF_2 + 7 * pow((TEMP + 1500), 2);
    SENS_2 = SENS_2 + 11 * pow((TEMP + 1500), 2) / 2;
  }

  OFF = OFF - OFF_2;
  SENS = SENS - SENS_2;
  TEMP = TEMP - T_2;

  P = ((D_1 * SENS / pow(2, 21) - OFF) / pow(2, 15))  / 100;

  float altitudine = 44330.0 * (1.0 - pow(P / 1013.25, 0.1903));


  Valori[0] = TEMP;
  Valori[1] = P;

}

// funzione calcolo checkSum stringa NMEA
int checkSum(String theseChars) {
  char check = 0;
  // iterate over the string, XOR each byte with the total sum:
  for (int c = 0; c < theseChars.length(); c++) {
    check = int(check ^ theseChars.charAt(c));

  }
  // return the result
  return check;
}
