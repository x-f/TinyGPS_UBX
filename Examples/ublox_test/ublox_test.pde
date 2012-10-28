// TinyGPS test for uBlox GPS proprietary PUBX format
// tested on Arduino 0022 and 1.0.1
// x-f, 2012

#include <avr/pgmspace.h>

// latest (former NewSoftSerial), not the one supplied with Arduino 0022
// SoftwareSerial's buffer size must be increased in SoftwareSerial.h:
// #define _SS_MAX_RX_BUFF 128 // RX buffer size
#include <SoftwareSerial.h>
#include <TinyGPS_UBX.h>

SoftwareSerial GPS_Serial(2, 3);
TinyGPS gps;

#define DEBUG true

byte gps_hour, gps_minute, gps_second;
long gps_lat, gps_lon;
unsigned long gps_fix_age;


// no need to store these in the RAM anyway
static char str_buffer[25];
prog_char GPSstr_poll[] PROGMEM = "$PUBX,00*33";
prog_char GPSstr_setup1[] PROGMEM = "$PUBX,40,ZDA,0,0,0,0*44";
prog_char GPSstr_setup2[] PROGMEM = "$PUBX,40,GLL,0,0,0,0*5C";
prog_char GPSstr_setup3[] PROGMEM = "$PUBX,40,VTG,0,0,0,0*5E";
prog_char GPSstr_setup4[] PROGMEM = "$PUBX,40,GSV,0,0,0,0*59";
prog_char GPSstr_setup5[] PROGMEM = "$PUBX,40,GSA,0,0,0,0*4E";
prog_char GPSstr_setup6[] PROGMEM = "$PUBX,40,GGA,0,0,0,0*5A";
prog_char GPSstr_setup7[] PROGMEM = "$PUBX,40,RMC,0,0,0,0*47";
PROGMEM const char *str_table[] = {
  GPSstr_poll, GPSstr_setup1, GPSstr_setup2, GPSstr_setup3, 
  GPSstr_setup4, GPSstr_setup5, GPSstr_setup6, GPSstr_setup7
};


void setup() {
  Serial.begin(9600);

  GPS_setup();
  
  // if you're using Arduino 1.0, you can have
  // Serial.println(F("we are go!"));
  // to print text strings from flash memory
  Serial.println("we are go!");
}

void loop() {

  GPS_poll();

  gps.crack_time(&gps_hour, &gps_minute, &gps_second, &gps_fix_age);
  gps.get_position(&gps_lat, &gps_lon, &gps_fix_age);

  char time[8];
  sprintf(time, "%02d:%02d:%02d", gps_hour, gps_minute, gps_second);

  Serial.println();
  Serial.print("time: "); Serial.println(time);
  //Serial.print("latitude: "); Serial.println(gps_lat, DEC);
  //Serial.print("longitude: "); Serial.println(gps_lon, DEC);
  Serial.print("latitude: "); Serial.println(gps_lat/100000.0, 5);
  Serial.print("longitude: "); Serial.println(gps_lon/100000.0, 5);
  Serial.print("altitude: "); Serial.print(gps.altitude()/100.0, 0); Serial.println(" m");
  //Serial.print("speed: "); Serial.print(gps.speed()/100.0, 0); Serial.println(" km/h");
  Serial.print("speed: "); Serial.print(gps.f_speed_kmph(), 2); Serial.println(" km/h");
  Serial.print("vert. speed: "); Serial.print(gps.vspeed(), DEC); Serial.println(" cm/s");
  Serial.print("bearing: "); Serial.println(gps.course()/100, DEC);
  Serial.print("satellites: "); Serial.println(gps.sats(), DEC);
  Serial.print("has fix: "); Serial.println(gps.has_fix(), DEC);
  Serial.print("fix quality: "); Serial.println(gps.fix_quality(), DEC);
  Serial.print("fix age: "); Serial.println(gps_fix_age, DEC);
  Serial.println("------------");

  delay(5000);
}


void GPS_setup() {
  GPS_Serial.begin(9600);
  // switch baudrate to 4800 bps
  //GPS_Serial.println("$PUBX,41,1,0007,0003,4800,0*13"); 
  //GPS_Serial.begin(4800);
  //GPS_Serial.flush();
  
  delay(500);
  
  // turn off all NMEA sentences for the uBlox GPS module
  // ZDA, GLL, VTG, GSV, GSA, GGA, RMC
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[1])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[2])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[3])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[4])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[5])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[6])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[7])));

  delay(500);
}

// request uBlox to give fresh data
boolean GPS_poll() {
  //GPS_Serial.println("$PUBX,00*33");
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[0])));
  delay(300);
  unsigned long starttime = millis();
  while (true) {
    if (GPS_Serial.available()) {
      char c = GPS_Serial.read();
      #if DEBUG
        Serial.print(c);
      #endif
      if (gps.encode(c))
        return true;
    }
    // that's it, can't wait any longer
    // i have short attention span..
    if (millis() - starttime > 1000) {
      #if DEBUG
        Serial.println("timeout");
      #endif
      break;
    }
  }
  return false;
}