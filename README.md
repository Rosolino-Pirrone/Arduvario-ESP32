# Arduvario-ESP32 
Semplice strumento per il volo libero. Componenti: Esp32, MS5611, Adafruit Ultimate GPS Breakout - 66 canali, display 4DSystems UOLED-160-G2.
Non mi assumo nessuna responsabilità per il corretto funzionamento del progetto.
Successivamente posterò lo schema delle connessioni, anche se per chi si intente di un minimo di progettazione su base Arduino non dovrebbero esserci problemi anche solo analizzando il codice. Sostiruire i file delle corrispettive librerie o modificarle nel modo seguente:
Per quel che riguarda la serial uart1 bisogna assegnare i gpio nel modo seguente nella libreria ESP32

void HardwareSerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert


{
    if(0 > _uart_nr || _uart_nr > 2) {
        log_e("Serial number is invalid, please use 0, 1 or 2");
        return;
    }
    if(_uart) {
        end();
    }
    if(_uart_nr == 0 && rxPin < 0 && txPin < 0) {
        rxPin = 3;
        txPin = 1;
    }
    if(_uart_nr == 1 && rxPin < 0 && txPin < 0) {
        rxPin = 32;
        txPin = 33;
    }
    if(_uart_nr == 2 && rxPin < 0 && txPin < 0) {
        rxPin = 16;
        txPin = 17;
    }
    _uart = uartBegin(_uart_nr, baud, config, rxPin, txPin, 256, invert);
}


Per quel che riguarda invece la funzione aggiuntiva della libreria Adafruit GPS, bisogna soltanto aggiungere due stringhe char al file 
Adafruit_GPS.h 

// Floating point latitude and longitude value in degrees.
  char stringa_GGA[120];
  char stringa_RMC[120];
  
  e successivamente passare il valore delle stringhe NMEA nel parsing nel file Adafruit_GPS.cpp in questo modo
  
  // look for a few common sentences
  
  if (strstr(nmea, "$GPGGA")) {
    // found GGA
    char *p = nmea;
	strncpy(stringa_GGA, p, 120);
    // get time
    
    
    if (strstr(nmea, "$GPRMC")) {
   // found RMC
    char *p = nmea;
	strncpy(stringa_RMC, p, 120);
    // get time
    
https://photos.app.goo.gl/6qEsDHLoZttG9dmz9
