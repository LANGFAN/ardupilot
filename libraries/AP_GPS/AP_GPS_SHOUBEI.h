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
//

/// @file	AP_GPS_SHOUBEI.h
/// @brief	SHOUBEI protocol parser
///
/// This is a lightweight SHOUBEI parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// SHOUBEI mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"



#define SHOUBEI_SET_BINARY "$JASC,GPGGA,5\r\n$JBAUD,57600\r\n"//added by LSH
/// SHOUBEI parser
///
class AP_GPS_SHOUBEI : public AP_GPS_Backend
{
    friend class AP_GPS_SHOUBEI_Test;

public:
	AP_GPS_SHOUBEI(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /// Checks the serial receive buffer for characters,
    /// attempts to parse SHOUBEI data and updates internal state
    /// accordingly.
    bool        read();

	static bool _detect(struct SHOUBEI_detect_state &state, uint8_t data);

private:
    /// Coding for the GPS sentences that the parser handles
    enum _sentence_types {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
		_GPS_SENTENCE_HPR=128,			////added by LSH
        _GPS_SENTENCE_OTHER = 0
    };
    uint8_t buf_bin[200];

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the SHOUBEI input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);
    bool                        _decode_BIN(char ch);//added by LSH
    bool                       handle_BIN(void);//added by LSH

    /// Return the numeric value of an ascii hex character
    ///
    /// @param	a		The character to be converted
    /// @returns		The value of the character as a hex digit
    ///
    int16_t                     _from_hex(char a);

    /// Parses the @p as a SHOUBEI-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a SHOUBEI-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of SHOUBEI messages
    bool _have_new_message(void);

    /// If primary GPS locates not at the UAV center
    /// here we convert its lat/lon to uav center location according to gps heading and offsets
    void fill_uav_center_pos(void);

    uint8_t _parity;                                                    ///< SHOUBEI message checksum accumulator
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[15];                                                     ///< buffer for the current term within the current sentence
    uint8_t _sentence_type;                                     ///< the sentence type currently being processed
    uint8_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    bool _gps_data_good;                                        ///< set when the sentence indicates data is good

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    int32_t _new_gps_heading;							///added by LSH   used for gps heading
    int32_t _new_gps_pitch;							///added by LSH   used for gps heading
    int32_t _new_gps_roll;							///added by LSH   used for gps heading
    uint8_t _new_gps_heading_mode;
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term



    uint32_t _last_RMC_ms = 0;
    uint32_t _last_GGA_ms = 0;
    uint32_t _last_VTG_ms = 0;

    /// @SHOUBEI	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _MTK_init_string[];                  ///< init string for MediaTek units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];
};

struct BIN1_msg_st{
	uint8_t AgeOfDiff;
	uint8_t NumOfSats;
	uint16_t GPSWeek;
	double GPSTimeOfWeek;
	double Latitude;
	double  Longitude;
	float Height;
	float VNorth;
	float VEast;
	float VUp;
	float StdDevResid;
	uint16_t NavMode;
	uint16_t ExtendedAge;
}__attribute__ ((__packed__));

struct BIN2_msg_st{
	uint32_t MaskSatsTracked;
	uint32_t MaskSatsUsed;
	uint16_t GPSUtcDiff;
	uint16_t HDOPTimes10;
	uint16_t VDOPTimes10;
	uint16_t WAAS_PRN_bitmask;
}__attribute__ ((__packed__));
