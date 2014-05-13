#include "Definitions.h"
#if SUPPORT_MAVLINK == 1
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "Util.h"

#define MAVLINK_GET_CHANNEL_BUFFER
#include "mavlink/mavlink_types.h"

extern "C" {
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
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
#include "Globals.h"

#define CHAN 0

// We use the same message both in parsing and in interpretation.
// That means we have to fish out all data needed from an incoming
// message before we even begin parsing another.
// (interrupt based parsing is a no no)
static mavlink_message_t receivedMessage;
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
	return &receivedMessage;
}

/*
 * This instance is used in communication with the client (us).
 * There is another internal instance also.
 */
static mavlink_status_t mavlinkStatus;

/*
 * We send this one back to ground control.
 */
static mavlink_mount_status_t myStatus;

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

struct MavlinkStoredGimbalOrientation {
	int32_t pitchOrLat;
	int32_t rollOrLon;
	int32_t yawOrAlt;
	int16_t crc;

	void calculateCRC() {
		crc = crc16((uint8_t*) this, sizeof(MavlinkStoredGimbalOrientation) - 2);
	}
};

static MavlinkStoredGimbalOrientation eeGimbalOrientation EEMEM;
static MavlinkStoredGimbalOrientation gimbalOrientation;
static uint8_t mavlinkGimbalMode;

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
void processConfigureMount() {
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
void processControlMount() {
	// target position
	gimbalOrientation.pitchOrLat = mavlink_msg_mount_control_get_input_a(&receivedMessage);
	gimbalOrientation.rollOrLon = mavlink_msg_mount_control_get_input_b(&receivedMessage);
	gimbalOrientation.yawOrAlt = mavlink_msg_mount_control_get_input_c(&receivedMessage);
	gimbalOrientation.calculateCRC();

	uint8_t save = mavlink_msg_mount_control_get_save_position(&receivedMessage);
	if (save) {
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
void processGlobalPosition() {
	mavlink_msg_global_position_int_decode(&receivedMessage, &globalPosition);
}

bool mavlinkInput() {
	bool valid = false;
	while (serial0.available()) {
		uint8_t parseResult = mavlink_parse_char(CHAN, serial0.get(), &receivedMessage, &mavlinkStatus);
		if (parseResult) {
			valid = true;
			uint8_t msgid = receivedMessage.msgid;
			if (msgid == MAVLINK_MSG_ID_MOUNT_CONFIGURE) {
				processConfigureMount();
			} else if (msgid == MAVLINK_MSG_ID_MOUNT_CONTROL) {
				processControlMount();
			} else if (msgid == MAVLINK_MSG_ID_MOUNT_STATUS) {
				// What, we are really supposed just to send these not receive them.
			} else if (msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
			}
		}
	}

	return valid;
}

int16_t mavlinkTargetPitch;
int16_t mavlinkTargetBearing ;

void calculateAim() {
	if (mavlinkGimbalMode == MAV_MOUNT_MODE_GPS_POINT) {
		if (globalPosition.lat == 0 && globalPosition.lon == 0)
			return; // we don't know where we are
		if (gimbalOrientation.pitchOrLat == 0 && gimbalOrientation.rollOrLon == 0) return; // We don't know what to aim at
		float latDiff_m = (gimbalOrientation.pitchOrLat - globalPosition.lat) * 0.011112f;
		float lonScale = cosf((float)globalPosition.lat / (10E7 * 180 / M_PI)); // should now be on a -PI/2..PI/2 scale.
		float lonDiff_m = (gimbalOrientation.pitchOrLat - globalPosition.lat) * lonScale * 0.011112f;
		int16_t targetBearing = Rajan_FastArcTan2(latDiff_m, lonDiff_m) * 18000/M_PI;
		float dist_m = sqrtf(latDiff_m * latDiff_m * + lonDiff_m * lonDiff_m);

		// It is on an [0..35999] interval, we want.. well anything is fine.
		// suggest [-17999..18000]
		int16_t airframeBearing = globalPosition.hdg > 18000 ? globalPosition.hdg-36000 : globalPosition.hdg;
		// relative.
		mavlinkTargetBearing = targetBearing - airframeBearing;

		float dAlt;
		if (config.mavlinkUseRelativealtitudes)
			dAlt= gimbalOrientation.yawOrAlt - globalPosition.relative_alt/1000.0f;
		else
			dAlt = gimbalOrientation.yawOrAlt - globalPosition.alt/1000.0f;
		float pitch = Rajan_FastArcTan2(dist_m, dAlt);
		mavlinkTargetPitch = pitch * 32768 / M_PI;
	}
}

#endif