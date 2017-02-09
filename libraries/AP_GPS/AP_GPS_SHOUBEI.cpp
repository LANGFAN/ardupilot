// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// SHOUBEI parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic SHOUBEI parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//

/// @file	AP_GPS_SHOUBEI.cpp
/// @brief	SHOUBEI protocol parser
///
/// This is a lightweight SHOUBEI parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include "GCS_MAVLink/GCS.h"

#include "AP_GPS_SHOUBEI.h"

extern const AP_HAL::HAL& hal;

// optionally log all SHOUBEI data for debug purposes
// #define SHOUBEI_LOG_PATH "SHOUBEI.log"

#ifdef SHOUBEI_LOG_PATH
#include <stdio.h>
#endif

// See LOCATION_SCALING_FACTOR_INV In AP_Common/Location.cpp
// This factor used to convert offsets between primary GPS antenna with center of UAV to lat and lon diff
// And then we can get the location of UAV center
#define CM_TO_LOCATION_SCALING_FACTOR 0.8983204953368922f

// SiRF init messages //////////////////////////////////////////////////////////
//
// Note that we will only see a SiRF in SHOUBEI mode if we are explicitly configured
// for SHOUBEI.  GPS_AUTO will try to set any SiRF unit to binary mode as part of
// the autodetection process.
//

/*
 *  "$JASC,GPGGA,5\r\n"\
"$JASC,GPRMC,5\r\n"\
"$JASC,GPVTG,5\r\n"\
*/
#define UniStrong_INIT_MSG\
	"$JOFF\r\n"\
	"$JDIFF,RTK\r\n"\
	"$JASC,GPGGA,10\r\n"\
	"$JASC,GPVTG,10\r\n"\
	"$JASC,GPHPR,15\r\n"\
	"$JBIN,1,15\r\n"\
	"$JBIN,2,5\r\n"\
	"$JSAVE\r\n"



const char AP_GPS_SHOUBEI::_initialisation_blob[] = UniStrong_INIT_MSG;

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+(x):'0'+(x))

AP_GPS_SHOUBEI::AP_GPS_SHOUBEI(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _parity(0),
    _is_checksum_term(false),
    _sentence_type(0),
    _term_number(0),
    _term_offset(0),
    _gps_data_good(false)
{
    gps.send_blob_start(state.instance, _initialisation_blob, sizeof(_initialisation_blob));
    // this guarantees that _term is always nul terminated
    memset(_term, 0, sizeof(_term));
}

bool AP_GPS_SHOUBEI::read(void)
{
    int16_t numc;
    bool parsed = false;

    numc = port->available();
    while (numc--) {
        char c = port->read();
#ifdef SHOUBEI_LOG_PATH
        static FILE *logf = NULL;
        if (logf == NULL) {
            logf = fopen(SHOUBEI_LOG_PATH, "wb");
        }
        if (logf != NULL) {
            ::fwrite(&c, 1, 1, logf);
        }
#endif
        if (_decode_BIN(c)||_decode(c)) {
            parsed = true;
        }
    }
		fill_uav_center_pos();
    return parsed;
}

bool AP_GPS_SHOUBEI::_decode(char c)
{
    bool valid_sentence = false;

    switch (c) {
    case ',': // term terminators
        _parity ^= c;
        /* no break */
    case '\r':
    case '\n':
    case '*':
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        _parity = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _gps_data_good = false;
        return valid_sentence;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term)
        _parity ^= c;

    return valid_sentence;
}

//added by LSH
bool
AP_GPS_SHOUBEI::_decode_BIN(char ch)
{
	static bool bin_rcv_start=false;
	static bool bin_is_receving=false;
	static uint16_t len=0;
	static int count=0;
	static uint16_t crc_caculate=0;
	static uint16_t crc_src=0;
	bool valid_sentence = false;

	 if(ch == '$' &&  !bin_is_receving){
		len=0;
		count=0;
		crc_caculate=0;
		crc_src=0;
		memset(buf_bin,0,200);
		bin_rcv_start=true;
		return valid_sentence;
	  }
	  if(bin_rcv_start){
		  buf_bin[count]=ch;
		 count ++;
		  switch(count){
			  case 3:
				  	  	  	  if(strncmp((char *)buf_bin,"BIN",3)!=0){
				 						 bin_rcv_start=false;
										 len=0;
										 count=0;
										 crc_caculate=0;
										 crc_src=0;
				 						 memset(buf_bin,0,200);
				 					} else{
				  	  	  						 bin_is_receving=true;
				  	  	  	  	  }
				  	  	  return valid_sentence;
			  case 7:
						 memcpy((void *)(&len),&buf_bin[5],2);
						  return valid_sentence;
				  case 8:
					  crc_caculate=ch;
					  return valid_sentence;
				  default : break;
		  }
		  if(count>8 && count <len+8){
						  crc_caculate+=ch;
					  }

	    if(count==len+9){ //received completed
			  bin_rcv_start=false;
			  bin_is_receving=false;
			  memcpy((void *)(&crc_src),&buf_bin[len+7],2);
			  if(crc_caculate == crc_src)//msg received correct
			  {
				  valid_sentence=true;
				  handle_BIN();
			  }
			 return valid_sentence;
	    }
	  }
		return valid_sentence;
}

//added by LSH

bool
AP_GPS_SHOUBEI::handle_BIN(void)
{
	struct BIN1_msg_st BIN1_msg;
	struct BIN2_msg_st BIN2_msg;

	uint16_t bin_id=0;
	 memcpy((void*)(&bin_id),&buf_bin[3],2);
//	 GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING,"msgod %d",bin_id);
	 switch(bin_id)
	 {
	 case 1:{
		 memcpy((void*)(&BIN1_msg),&buf_bin[7],sizeof(struct BIN1_msg_st));
//		        		 GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING,"sizeof %d",sizeof(struct BIN1_msg_st));
//		        			state.num_sats=BIN1_msg.NumOfSats; ///< Number of visible satelites
//							state.location.alt=BIN1_msg.Height*100; //Altitude in centimeters (meters * 100)
			state.location.lat= (int32_t)(BIN1_msg.Latitude*1e7);
			state.location.lng= (int32_t)(BIN1_msg.Longitude*1e7);
			state.have_vertical_velocity=true;
			state.velocity.z = -BIN1_msg.VUp;  //the up velocity is negative
			state.velocity.x= BIN1_msg.VNorth;
			state.velocity.y= BIN1_msg.VEast;
			state.time_week=BIN1_msg.GPSWeek;
			state.time_week_ms=BIN1_msg.GPSTimeOfWeek*1000;
			switch(BIN1_msg.NavMode)
			{
			case 0:
				state.status=AP_GPS::NO_FIX;
				break;
			case 1:
				state.status=AP_GPS::GPS_OK_FIX_2D;
				break;
			case 2:
				state.status=AP_GPS::GPS_OK_FIX_3D;
				break;
			case 5:
				state.status=AP_GPS::GPS_OK_FIX_3D_DGPS;
				break;
			case 6:
				state.status=AP_GPS::GPS_OK_FIX_3D_RTK;
				break;
			default: break;
			}
			 if(state.status>2){
				 state.last_gps_time_ms=AP_HAL::millis();
			 }
		 break;
	 }

	 case 2:{
		 memcpy((void*)(&BIN2_msg),&buf_bin[7],sizeof(struct BIN2_msg_st));
//		        		 GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING,"sizeof %d",sizeof(struct BIN2_msg_st));
		 state.hdop=BIN2_msg.HDOPTimes10*10;
		 state.vdop=BIN2_msg.VDOPTimes10*10;
		 break;
	 }
	 default: break;
	 }
		        	 return true;
}


//
// internal utilities
//
int16_t AP_GPS_SHOUBEI::_from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

int32_t AP_GPS_SHOUBEI::_parse_decimal_100(const char *p)
{
    char *endptr = nullptr;
    long ret = 100 * strtol(p, &endptr, 10);
    int sign = ret < 0 ? -1 : 1;

    if (ret >= (long)INT32_MAX) {
        return INT32_MAX;
    }
    if (ret <= (long)INT32_MIN) {
        return INT32_MIN;
    }
    if (endptr == nullptr || *endptr != '.') {
        return ret;
    }

    if (isdigit(endptr[1])) {
        ret += sign * 10 * DIGIT_TO_VAL(endptr[1]);
        if (isdigit(endptr[2])) {
            ret += sign * DIGIT_TO_VAL(endptr[2]);
            if (isdigit(endptr[3])) {
                ret += sign * (DIGIT_TO_VAL(endptr[3]) >= 5);
            }
        }
    }
    return ret;
}

/*
  parse a SHOUBEI latitude/longitude degree value. The result is in degrees*1e7
 */
uint32_t AP_GPS_SHOUBEI::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    // scan for decimal point or end of field
    for (p = _term; *p && isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees
    while ((p - q) > 2 && *q) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }

    // convert minutes
    while (p > q && *q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }

    // convert fractional minutes
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (*q && isdigit(*q)) {
            frac_min += DIGIT_TO_VAL(*q) * frac_scale;
            q++;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t) (frac_min * (1.0e7f / 60.0f));
    return ret;
}

/*
  see if we have a new set of SHOUBEI messages
 */
bool AP_GPS_SHOUBEI::_have_new_message()
{
    if (_last_RMC_ms == 0 ||
        _last_GGA_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _last_RMC_ms > 150 ||
        now - _last_GGA_ms > 150) {
        return false;
    }
    if (_last_VTG_ms != 0 &&
        now - _last_VTG_ms > 150) {
        return false;
    }
    // prevent these messages being used again
    if (_last_VTG_ms != 0) {
        _last_VTG_ms = 1;
    }
    _last_GGA_ms = 1;
    _last_RMC_ms = 1;
    return true;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_SHOUBEI::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        uint8_t checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
        if (checksum == _parity) {
            if (_gps_data_good||_sentence_type==_GPS_SENTENCE_HPR) {
                uint32_t now = AP_HAL::millis();
                switch (_sentence_type) {
                case _GPS_SENTENCE_HPR:
                	//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "GPS Yaw %f", state.gps_heading);
                	state.gps_heading= ToRad((float)_new_gps_heading/100); //added by LSH     at here gps not good  the gps heading is  useless
                	if(_new_gps_heading_mode=='N') {
                		state.have_gps_heading=true;
                		//GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "have_gps_heading  %d", state.have_gps_heading);
                	}else{
                		state.have_gps_heading=false;
                	}
                	break;
                case _GPS_SENTENCE_RMC:
                    _last_RMC_ms = now;
                    //time                        = _new_time;
                    //date                        = _new_date;
                    state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course    = wrap_360(_new_course*0.01f);
                    make_gps_time(_new_date, _new_time * 10);
                    state.last_gps_time_ms = now;
                    // To-Do: add support for proper reporting of 2D and 3D fix
                    //state.status           = AP_GPS::GPS_OK_FIX_3D;
                    //fill_3d_velocity();
                    break;
                case _GPS_SENTENCE_GGA:
                    _last_GGA_ms = now;
                    state.location.alt  = _new_altitude;
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
                    state.num_sats      = _new_satellite_count;
                    state.hdop          = _new_hdop;
                    // To-Do: add support for proper reporting of 2D and 3D fix
                    //state.status        = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case _GPS_SENTENCE_VTG:
                    _last_VTG_ms = now;
                    state.ground_speed  = _new_speed*0.01f;
                    state.ground_course = wrap_360(_new_course*0.01f);
                  //  fill_3d_velocity();
                    // VTG has no fix indicator, can't change fix status
                    break;
                }
            } else {
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                case _GPS_SENTENCE_GGA:
                    // Only these sentences give us information about
                    // fix status.
                    state.status = AP_GPS::NO_FIX;
                }
            }
            // see if we got a good message
            return _have_new_message();
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        /*
          The first two letters of the SHOUBEI term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = _GPS_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "RMC") == 0) {
            _sentence_type = _GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {
            _sentence_type = _GPS_SENTENCE_GGA;
        } else if (strcmp(term_type, "VTG") == 0) {
            _sentence_type = _GPS_SENTENCE_VTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            _gps_data_good = true;
        }else if(strcmp(&_term[0], "PSAT") == 0){			///added by LSH for gps_heading
        	_sentence_type =_GPS_SENTENCE_HPR;
        }else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }

        return false;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
        // operational status
        //
        case _GPS_SENTENCE_HPR+3:
			_new_gps_heading=_parse_decimal_100(_term);		///added by LSH
			break;
        case _GPS_SENTENCE_HPR+4:
			_new_gps_pitch=_parse_decimal_100(_term);		///added by LSH
			break;
        case _GPS_SENTENCE_HPR+5:
			_new_gps_roll=_parse_decimal_100(_term);		///added by LSH
			break;
        case _GPS_SENTENCE_HPR+6:
			_new_gps_heading_mode=_term[0];		///added by LSH
			if(state.have_gps_heading && (_new_gps_heading_mode=='G' || _new_gps_heading_mode==0))
				GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING,"GPS_heading  Lost");
			else if(!state.have_gps_heading &&_new_gps_heading_mode=='N')
				GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING,"GPS_heading  Got");
			break;
        case _GPS_SENTENCE_RMC + 2: // validity (RMC)
            _gps_data_good = _term[0] == 'A';
            break;
        case _GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            _gps_data_good = _term[0] > '0';
            break;
        case _GPS_SENTENCE_VTG + 9: // validity (VTG) (we may not see this field)
            _gps_data_good = _term[0] != 'N';
            break;
        case _GPS_SENTENCE_GGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GGA + 8: // HDOP (GGA)
            _new_hdop = (uint16_t)_parse_decimal_100(_term);
            break;

        // time and date
        //
        case _GPS_SENTENCE_RMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GGA + 1: // Time (GGA)
            _new_time = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

        // location
        //
        case _GPS_SENTENCE_RMC + 3: // Latitude
        case _GPS_SENTENCE_GGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 4: // N/S
        case _GPS_SENTENCE_GGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_RMC + 5: // Longitude
        case _GPS_SENTENCE_GGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 6: // E/W
        case _GPS_SENTENCE_GGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal_100(_term);
            break;

        // course and speed
        //
        case _GPS_SENTENCE_RMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_VTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal_100(_term) * 514) / 1000;       // knots-> m/sec, approximiates * 0.514
            break;
        case _GPS_SENTENCE_RMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_VTG + 1: // Course (VTG)
            _new_course = _parse_decimal_100(_term);
            break;
        }
    }



    return false;
}

/*
  detect a SHOUBEI GPS. Adds one byte, and returns true if the stream
  matches a SHOUBEI string
 */
bool
AP_GPS_SHOUBEI::_detect(struct SHOUBEI_detect_state &state, uint8_t data)
{
	switch (state.step) {
	case 0:
		state.ck = 0;
		if ('$' == data) {
			state.step++;
		}
		break;
	case 1:
		if ('*' == data) {
			state.step++;
		} else {
			state.ck ^= data;
		}
		break;
	case 2:
		if (hexdigit(state.ck>>4) == data) {
			state.step++;
		} else {
			state.step = 0;
		}
		break;
	case 3:
		if (hexdigit(state.ck&0xF) == data) {
            state.step = 0;
			return true;
		}
		state.step = 0;
		break;
    }
    return false;

//	static bool bin_rcv_start=false;
//	static bool bin_is_receving=false;
//	static uint16_t len=0;
//	static int count=0;
//	static uint16_t crc_caculate=0;
//	static uint16_t crc_src=0;
//	bool valid_sentence = false;
//	static uint8_t tmp_buf[200];
//
//	 if(ch == '$' &&  !bin_is_receving){
//		len=0;
//		count=0;
//		crc_caculate=0;
//		crc_src=0;
//		memset(tmp_buf,0,200);
//		bin_rcv_start=true;
//		return valid_sentence;
//	  }
//	  if(bin_rcv_start){
//		  tmp_buf[count++]=ch;
//		  switch(count){
//			  case 9: //received completed
//							  bin_rcv_start=false;
//							  bin_is_receving=false;
//							  memcpy((void *)(&crc_src),&tmp_buf[len+7],2);
//							  if(crc_caculate == crc_src)//msg received correct
//							  {
//								  valid_sentence=true;
//							  }
//							 return valid_sentence;
//			  case 3:
//				  	  	  	  if(strncmp((char *)tmp_buf,"BIN",3)!=0){
//				 						 bin_rcv_start=false;
//										 len=0;
//										 count=0;
//										 crc_caculate=0;
//										 crc_src=0;
//				 						 memset(tmp_buf,0,200);
//				 					} else{
//				  	  	  						 bin_is_receving=true;
//				  	  	  	  	  }
//				  	  	  return valid_sentence;
//			  case 7:
//						 memcpy((void *)(&len),&tmp_buf[5],2);
//						  return valid_sentence;
//				  case 8:
//					  crc_caculate=ch;
//					  return valid_sentence;
//				  default : break;
//		  }
//		  if(count>8 && count <len+8){
//						  crc_caculate+=ch;
//					  }
//		}
//	  return valid_sentence;
}

void
AP_GPS_SHOUBEI::fill_uav_center_pos(void)
{
	if(state.have_gps_heading)
	{
		float& heading = state.gps_heading;
		// distance between primary gps with UAV center in cm
		float distance = norm(gps._x_offset, gps._y_offset) * CM_TO_LOCATION_SCALING_FACTOR;
		if(!(is_zero(distance) || isnan(distance) || isinf(distance)))
		{
			float scale = cosf(state.location.lat * 1.0e-7f * DEG_TO_RAD);
			constrain_float(scale, 0.01f, 1.0f);
			state.location.lng += (int32_t)(distance * sinf(heading) / scale);
			state.location.lat += (int32_t)(distance * cosf(heading));
		}
	}
}
