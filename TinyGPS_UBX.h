/*
  TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
  Based on work by and "distance_to" courtesy of Maarten Lamers.
  Copyright (C) 2008-2011 Mikal Hart
  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  -----------------------------------------------------
  This stripped down version works only with uBlox proprietary protocol – PUBX.
  Modified by x-f, 2012
*/

#ifndef TinyGPS_UBX_h
#define TinyGPS_UBX_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
  
#define _GPS_VERSION 101 // software version of this library
// #define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112

class TinyGPS
{
  public:
    TinyGPS();
    bool encode(char c); // process one character received from GPS
    TinyGPS &operator << (char c) {encode(c); return *this;}
    
    // lat/long in hundred thousandths of a degree and age of fix in milliseconds
    inline void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
    {
      if (latitude) *latitude = _latitude;
      if (longitude) *longitude = _longitude;
      if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_position_fix;
    }

    // time as hhmmsscc, and age in milliseconds
    inline void get_time(unsigned long *time, unsigned long *fix_age = 0)
    {
      if (time) *time = _time;
      if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_time_fix;
    }

    // signed altitude in centimeters
    inline long altitude() { return _altitude; }

    // course in 100th of a degree
    inline unsigned long course() { return _course; }
    
    // speed in 100ths of a km/h
    unsigned long speed() { return _speed; }
    
    unsigned int sats() { return _sats; }
    bool has_fix() { 
      return (_gps_data_good && (_sats > 2 && _sats < 99));
    }
    int fix_quality() { return _gps_fix_quality; }
    long vspeed() { return _vspeed; }
    
    inline void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0)
    {
      long lat, lon;
      get_position(&lat, &lon, fix_age);
      *latitude = lat / 100000.0;
      *longitude = lon / 100000.0;
    }

    inline void crack_time(byte *hour, byte *minute, byte *second, unsigned long *fix_age = 0)
    {
      unsigned long time;
      get_time(&time, fix_age);
      if (hour) *hour = time / 1000000;
      if (minute) *minute = (time / 10000) % 100;
      if (second) *second = (time / 100) % 100;
    }

    inline float f_altitude()    { return altitude() / 100.0; }
    inline float f_course()      { return course() / 100.0; }
    // inline float f_speed_knots() { return speed() / _GPS_KMPH_PER_KNOT / 100.0; }
    inline float f_speed_mph()   { return speed() * _GPS_MILES_PER_METER * 10.0; }
    inline float f_speed_kmph()  { return speed() / 100.0; }

    static int library_version() { return _GPS_VERSION; }

    enum {GPS_INVALID_AGE = 0xFFFFFFFF, GPS_INVALID_ANGLE = 9999999, GPS_INVALID_ALTITUDE = -1,       GPS_INVALID_TIME = 0xFFFFFFFF, GPS_INVALID_SPEED = -1, GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 99};


    //static float distance_between (float lat1, float long1, float lat2, float long2);

private:
    enum {_GPS_SENTENCE_PUBX, _GPS_SENTENCE_OTHER};
    
    // properties
    unsigned long _time, _new_time;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    long _altitude, _new_altitude;
    unsigned long  _speed, _new_speed;
    unsigned long  _course, _new_course;

  	byte _sats, _new_sats;
  	byte _gps_fix_quality, _new_gps_fix_quality;
    long _vspeed, _new_vspeed;

    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    byte _term_offset;
    bool _gps_data_good;


    // internal utilities
    int from_hex(char a);
    unsigned long parse_decimal();
    unsigned long parse_degrees();
    bool term_complete();
    bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
    long gpsatol(const char *str);
    int gpsstrcmp(const char *str1, const char *str2);
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round 

#endif
