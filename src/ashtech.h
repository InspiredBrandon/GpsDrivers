/****************************************************************************
 *
 *   Copyright (C) 2013. All rights reserved.
 *   Author: Boriskin Aleksey <a.d.boriskin@gmail.com>
 *           Kistanov Alexander <akistanov@gramant.ru>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/** @file ASHTECH protocol definitions */

#pragma once

#include "gps_helper.h"
#include "base_station.h"
#include "../../definitions.h"

#include <math.h>

class RTCMParsing;

#define ASHTECH_RECV_BUFFER_SIZE 512
#define ASH_RESPONSE_TIMEOUT     200    // ms, timeout for waiting for a response

class GPSDriverAshtech : public GPSBaseStationSupport
{
public:
	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverAshtech(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position,
			 satellite_info_s *satellite_info, float heading_offset = 0.f);

	virtual ~GPSDriverAshtech();

	int configure(unsigned &baudrate, OutputMode output_mode) override;

	int receive(unsigned timeout) override;

private:
	enum class AshtechBoard {
		trimble_mb_two,
		other
	};

	// Option descriptions are from the Trimble MB-Two GNSS OEM Reference Manual, Rev F, Dec 2019
	enum class AshtechOption {
		GEOFENCING_WW,			// Option ID @1	Worldwide enabled receiver
		GEOFENCING_CHINA,		// Option ID @2	China-only enabled receiver
		GEOFENCING_JAPAN		// Option ID @3	Japan-only enabled receiver
		GEOFENCING_BRAZIL,		// Option ID @4	Brazil-only enabled receiver
		GEOFENCING_N_AMERICA,	// Option ID @5	North-America-only enabled receiver
		GEOFENCING_L_AMERICA,	// Option ID @6	Latin-America-only enabled receiver
		GEOFENCING_RUSSIA,		// Option ID @7	Russia-only enabled receiver
		GEOFENCING_INDIA,		// Option ID @8	India-only enabled receiver
		GEOFENCING_TURKEY,		// Option ID @9	Turkey-only-enabled receiver

		2HZ,		// Option ID 2	Enables output rate up to 2 Hz (for position/raw data)
		5HZ,		// Option ID 5	Enables output rate up to 5 Hz (for position/raw data)
		10HZ,		// Option ID 6	Enables output rate up to 10 Hz (for position/raw data)
		20HZ,		// Option ID W	Enables output rate up to 20 Hz (for position/raw data)
		50HZ,		// Option ID 8	Enables output rate up to 50 Hz (for position/raw data)

		DUO,		// Option ID D	Allows use of dual-sensor mode

		RTKROVER,	// Option ID J	Allows fixed RTK computaitons
		RTKBASE,	// Option ID K	Enables the reciever to generate differential messages

		BEIDOU,		// Option ID B	BEIDOU satellites tracking
		GLONASS,	// Option ID G	GLONASS satellites tracking
		GPS,		// Option ID N	GPS+SBAS+QZSS satellites tracking
		GALILEO,	// Option ID O	GALILEO satellites tracking

		L1TRACKING	// Option ID X	Enables L1 Tracking
		L2TRACKING	// Option ID Y	Enables L2 Tracking

		N			// Option ID -	Option is not installed

	};


	enum class NMEACommand {
		Acked, // Command that returns a (N)Ack
		PRT,   // port config
		RID,   // board identification
		RECEIPT// board identification
	};

	enum class NMEACommandState {
		idle,
		waiting,
		nack,
		received
	};

	enum class NMEADecodeState {
		uninit,
		got_sync1,
		got_asteriks,
		got_first_cs_byte,
		decode_rtcm3
	};

	/**
	 * enable output of correction output
	 */
	void activateCorrectionOutput();

	void activateRTCMOutput();

	void decodeInit(void);

	int handleMessage(int len);

	int parseChar(uint8_t b);

	/**
	 * receive data for at least the specified amount of time
	 */
	void receiveWait(unsigned timeout_min);

	void sendSurveyInStatusUpdate(bool active, bool valid, double latitude = static_cast<double>(NAN),
				      double longitude = static_cast<double>(NAN), float altitude = NAN);

	/**
	 * Write a command and wait for a (N)Ack
	 * @return 0 on success, <0 otherwise
	 */
	int writeAckedCommand(const void *buf, int buf_length, unsigned timeout);

	int waitForReply(NMEACommand command, const unsigned timeout);

	bool _correction_output_activated{false};
	bool _configure_done{false};
	bool _got_pashr_pos_message{false}; /**< If we got a PASHR,POS message, we will ignore GGA messages */

	char _port{'A'}; /**< port we are connected to (e.g. 'A') */

	uint8_t _rx_buffer[ASHTECH_RECV_BUFFER_SIZE];
	uint16_t _rx_buffer_bytes{};
	uint64_t _last_timestamp_time{0};

	float _heading_offset;

	gps_abstime _survey_in_start{0};

	sensor_gps_s *_gps_position {nullptr};

	satellite_info_s *_satellite_info {nullptr};

	AshtechBoard _board{AshtechBoard::other}; /**< board we are connected to */

	AshtechOption _geofence{AshtechOption::GEOFENCING_WW};
	AshtechOption _output_rate{AshtechOption::2HZ};
	AshtechOption _duo{AshtechOption::N};
	AshtechOption _rtk{AshtechOption::RTKROVER};
	AshtechOption _beidou{AshtechOption::N};
	AshtechOption _glonass{AshtechOption::N};
	AshtechOption _gps{AshtechOption::N};
	AshtechOption _galileo{AshtechOption::N};
	AshtechOption _l1tracking{AshtechOption::N};	
	AshtechOption _l2tracking{AshtechOption::N};


	NMEACommand _waiting_for_command;

	NMEACommandState _command_state{NMEACommandState::idle};

	NMEADecodeState _decode_state{NMEADecodeState::uninit};

	OutputMode _output_mode{OutputMode::GPS};

	RTCMParsing *_rtcm_parsing{nullptr};
};

