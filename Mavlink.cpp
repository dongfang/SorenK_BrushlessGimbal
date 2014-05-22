#include "Definitions.h"
#if SUPPORT_MAVLINK == 1
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "Util.h"
#include "Globals.h"

#define DEBUG_MAVLINK 1

#define MAVLINK_GET_CHANNEL_BUFFER
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink/mavlink_types.h"

extern "C" {
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
}

mavlink_system_t mavlink_system;

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t c) {
	serial0.put(c);
}

// Unfortunately I cannot find any way to get MAVLINK_MESSAGE_CRC defined before
// mavlink_helpers is loaded than to redefine this.
#define MAVLINK_STX 254

#include "mavlink/ardupilotmega/ardupilotmega.h"

// There is a 256 bytes table of CRC spices. We move that into PGMSPACE.
//#define MAVLINK_MESSAGE_CRC
static const uint8_t mavlink_message_crcs[256] PROGMEM = MAVLINK_MESSAGE_CRCS;
#define MAVLINK_MESSAGE_CRC(msgid) pgm_read_byte(mavlink_message_crcs+msgid)

#include "mavlink/ardupilotmega/mavlink.h"
#define CHAN 0

struct MavlinkStoredGimbalOrientation {
	bool valid;
	int32_t pitchOrLat;
	int32_t rollOrLon;
	int32_t yawOrAlt;
	uint16_t crc;

	uint16_t calculateCRC() {
		return crc16((uint8_t*) this, sizeof(MavlinkStoredGimbalOrientation) - 2);
	}
};

static MavlinkStoredGimbalOrientation eeGimbalOrientation EEMEM;
static MavlinkStoredGimbalOrientation gimbalOrientation;
static uint8_t mavlinkGimbalMode;

void mavlink_init() {
	mavlink_system.sysid = config.mavlinkSystemId;
	mavlink_system.compid = config.mavlinkComponentId;
	mavlink_system.type = MAV_TYPE_ANTENNA_TRACKER; // Bah there is no gimbal type?

	eeprom_read_block(&gimbalOrientation, &eeGimbalOrientation, sizeof(MavlinkStoredGimbalOrientation));
	if (gimbalOrientation.calculateCRC() != gimbalOrientation.crc) {
		gimbalOrientation.valid = false; // do not set true otherwise. The eeprom value should be valid, no need to overwrite it.
	}
}

// We use the same message both in parsing and in interpretation.
// That means we have to fish out all data needed from an incoming
// message before we even begin parsing another.
// (interrupt based parsing is a no no)
static mavlink_message_t receivedMessage;
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
	return &receivedMessage;
}

//uint8_t mavlinkUseRelativealtitudes = 0;
//int16_t mavlinkTargetPitch;
int16_t mavlinkTargetBearing;

// How many meters per time unit (one unit being 510 clock cycles)
float vlat_m_unit;
float vlon_m_unit;

float lonDiff_m;
float latDiff_m;

uint32_t unitsAccountedFor = -1;

/*
 * This instance is used in communication with the client (us).
 * There is another internal instance also.
 */
static mavlink_status_t mavlinkStatus;

/*
 * We send this one back to ground control.
 *
 {
 int32_t pointing_a; ///< pitch(deg*100) or lat, depending on mount mode
 int32_t pointing_b; ///< roll(deg*100) or lon depending on mount mode
 int32_t pointing_c; ///< yaw(deg*100) or alt (in cm) depending on mount mode
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 } mavlink_mount_status_t;
 */
//static mavlink_mount_status_t myStatus;
/*
 * We might as well store the whole global pos. message; we will need most of it.
 */
static mavlink_global_position_int_t globalPosition;

/*
 * typedef struct __mavlink_status {
 uint8_t msg_received;               ///< Number of received messages
 uint8_t buffer_overrun;             ///< Number of buffer overruns
 uint8_t parse_error;                ///< Number of parse errors
 mavlink_parse_state_t parse_state;  ///< Parsing state machine
 uint8_t packet_idx;                 ///< Index in current packet
 uint8_t current_rx_seq;             ///< Sequence number of last packet received
 uint8_t current_tx_seq;             ///< Sequence number of last packet sent
 uint16_t packet_rx_success_count;   ///< Received packets
 uint16_t packet_rx_drop_count;      ///< Number of packet drops
 } mavlink_status_t;

 typedef enum MAV_MOUNT_MODE
 {
 MAV_MOUNT_MODE_RETRACT=0, / * Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | * /
 MAV_MOUNT_MODE_NEUTRAL=1, / * Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | * /
 MAV_MOUNT_MODE_MAVLINK_TARGETING=2, / * Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | * /
 MAV_MOUNT_MODE_RC_TARGETING=3, / * Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | * /
 MAV_MOUNT_MODE_GPS_POINT=4, / * Load neutral position and start to point to Lat,Lon,Alt | * /
 MAV_MOUNT_MODE_ENUM_END=5, / *  | * /
 } MAV_MOUNT_MODE;
 */

/*
 typedef struct __mavlink_mount_configure_t
 {
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t mount_mode; ///< mount operating mode (see MAV_MOUNT_MODE enum)
 uint8_t stab_roll; ///< (1 = yes, 0 = no)
 uint8_t stab_pitch; ///< (1 = yes, 0 = no)
 uint8_t stab_yaw; ///< (1 = yes, 0 = no)
 } mavlink_mount_configure_t;
 */
static void processConfigureMount() {
	// There shoud be no reason to care about any other than this field. We never stabilize to foreign IMU data anyway.
	mavlinkGimbalMode = mavlink_msg_mount_configure_get_mount_mode(&receivedMessage);
}

/*
 typedef struct __mavlink_mount_control_t
 {
 int32_t input_a; ///< pitch(deg*100) or lat, depending on mount mode
 int32_t input_b; ///< roll(deg*100) or lon depending on mount mode
 int32_t input_c; ///< yaw(deg*100) or alt (in cm) depending on mount mode
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t save_position; ///< if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
 } mavlink_mount_control_t;
 */
static void processControlMount() {
	// target position
	if (mavlink_msg_mount_control_get_target_system(&receivedMessage) == config.mavlinkSystemId
			&& mavlink_msg_mount_control_get_target_component(&receivedMessage) == config.mavlinkComponentId) {
		gimbalOrientation.pitchOrLat = mavlink_msg_mount_control_get_input_a(&receivedMessage);
		gimbalOrientation.rollOrLon = mavlink_msg_mount_control_get_input_b(&receivedMessage);
		gimbalOrientation.yawOrAlt = mavlink_msg_mount_control_get_input_c(&receivedMessage);
		gimbalOrientation.valid = true;
	}

	uint8_t save = mavlink_msg_mount_control_get_save_position(&receivedMessage);
	if (save) {
		gimbalOrientation.crc = gimbalOrientation.calculateCRC();
		eeprom_update_block(&gimbalOrientation, &eeGimbalOrientation, sizeof(MavlinkStoredGimbalOrientation));
	}
}

/*
 typedef struct __mavlink_global_position_int_t
 {
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t lat; ///< Latitude, expressed as * 1E7
 int32_t lon; ///< Longitude, expressed as * 1E7
 int32_t alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 int32_t relative_alt; ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
 int16_t vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
 int16_t vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
 int16_t vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
 uint16_t hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 } mavlink_global_position_int_t;
 */
static void processGlobalPosition() {
	mavlink_msg_global_position_int_decode(&receivedMessage, &globalPosition);
	//if (mavlinkGimbalMode == MAV_MOUNT_MODE_GPS_POINT) {
	if (globalPosition.lat == 0 && globalPosition.lon == 0)
		return; // we don't know where we are
	if (!gimbalOrientation.valid)
		return; // We don't know what to aim at
	latDiff_m = (gimbalOrientation.pitchOrLat - globalPosition.lat) * 0.011112f;
	// from cm/sec to m/units: Multiply by sec/unit
	// n2 = v / (m/unit) = v * unit/m
	// n =  v / (cm/sec) = v * sec/cm
	// n2 = f n
	// f = n2/n = unit/m / sec/cm = cm*COARSE_TIME_UNITS*sec / (sec*m) = COARSE_TIME_UNITS/100
	vlat_m_unit = globalPosition.vx * (COARSE_TIME_UNITS / 100);
	float latAsRadians = (float) globalPosition.lat / (1E7 * 180.0 / M_PI);
	float lonScale = cosf(latAsRadians); // should now be on a -PI/2..PI/2 scale.
	lonDiff_m = (gimbalOrientation.rollOrLon - globalPosition.lon) * lonScale * 0.011112f;
	vlon_m_unit = globalPosition.vy * (COARSE_TIME_UNITS / 100);
	unitsAccountedFor = coarseTime();
}

bool mavlink_parse() {
	bool valid = false;
	while (serial0.available()) {
		uint8_t parseResult = mavlink_parse_char(CHAN, serial0.get(), &receivedMessage, &mavlinkStatus);
		if (parseResult) {
			valid = true;
			uint8_t msgid = receivedMessage.msgid;
			if (msgid == MAVLINK_MSG_ID_MOUNT_CONFIGURE) {
				processConfigureMount();
				if (DEBUG_MAVLINK)
					printf_P(PSTR("MAVLINK_MSG_ID_MOUNT_CONFIGURE, mode %d\r\n"), mavlinkGimbalMode);
			} else if (msgid == MAVLINK_MSG_ID_MOUNT_CONTROL) {
				processControlMount();
				if (DEBUG_MAVLINK)
					printf_P(PSTR("MAVLINK_MSG_ID_MOUNT_CONTROL, orientation %ld %ld %ld\r\n"),
							gimbalOrientation.pitchOrLat, gimbalOrientation.rollOrLon, gimbalOrientation.yawOrAlt);
			} else if (msgid == MAVLINK_MSG_ID_MOUNT_STATUS) {
				// What, we are really supposed just to send these not receive them.
			} else if (msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
				processGlobalPosition();
				if (DEBUG_MAVLINK)
					printf_P(PSTR("MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "
							"lat %ld, long %ld, alt %ld ralt %ld\r\n"), globalPosition.lat, globalPosition.lon,
							globalPosition.alt, globalPosition.relative_alt);
			}
		}
	}

	return valid;
}

/*
 * We are looking at a range of about 31.34 each 90 degrees, or 286.875 centi-degrees/step
 */

#ifdef SUPPORT_YAW_SERVO

extern void setYawServoOut(uint16_t usec);

void setYawServo() {
	static int16_t prevAngle;
	int16_t angle = mavlinkTargetBearing;
	int16_t yawServoLimit = config.yawServoLimit * 100;
	if (angle > yawServoLimit) {
		angle = yawServoLimit;
		// Avoid full end to end turns when flying away from target: If we were left before and now full right, stay as before.
		if (prevAngle < 0)
			angle = -angle;
	} else if (angle < -yawServoLimit) {
		angle = -yawServoLimit;
		// Avoid full end to end turns when flying away from target: If we were right before and now full left, stay as before.
		if (prevAngle > 0)
			angle = -angle;
	}
	prevAngle = angle;
	// There are 1000 usec to 90 degrees.
	// That is 0.1111111 usec to a centidegree
	setYawServoOut((config.yawServoDirection * angle * 1.0f / 9.0f) + 1500);
}
#endif

void mavlink_track() {
	uint32_t timeUnits = coarseTime();
	uint32_t unitsToDo = timeUnits - unitsAccountedFor;
	unitsAccountedFor = timeUnits;
	lonDiff_m -= unitsToDo * vlon_m_unit;
	latDiff_m -= unitsToDo * vlat_m_unit;
	float targetBearing_rad = Rajan_FastArcTan2(lonDiff_m, latDiff_m);

	int16_t targetBearing = targetBearing_rad * (18000.0 / M_PI);
	float dist_m = sqrtf(latDiff_m * latDiff_m + lonDiff_m * lonDiff_m);
// Transform to a -180..180 range
	int32_t temp = (uint32_t) targetBearing - globalPosition.hdg;
	if (temp < 0)
		mavlinkTargetBearing = temp + 36000;
	else if (temp >= 36000)
		mavlinkTargetBearing = temp - 36000;
	else
		mavlinkTargetBearing = temp;

	float dAlt;
	if (config.mavlinkUseRelativealtitudes)
		dAlt = (gimbalOrientation.yawOrAlt - globalPosition.relative_alt) / 1000.0f;
	else
		dAlt = (gimbalOrientation.yawOrAlt - globalPosition.alt) / 1000.0f;
	float pitch = Rajan_FastArcTan2(dAlt, dist_m);

	targetSources[TARGET_SOURCE_MAVLINK][ROLL] = 0;
	targetSources[TARGET_SOURCE_MAVLINK][PITCH] = pitch * (32768.0 / M_PI);
}

extern int16_t getTarget(uint8_t axis);

void mavlink_sendStatus() {
	//mavlink_msg_mount_status_send();
	//myStatus.pointing_a = NDToCentidegrees(getTarget(PITCH));
	//myStatus.pointing_b = NDToCentidegrees(getTarget(ROLL));
	//myStatus.pointing_c = mavlinkTargetBearing;
}
#endif
