#include "Definitions.h"
#ifdef SUPPORT_MAVLINK
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "Util.h"
#include "Globals.h"
#include "Commands.h"
#include "Mavlink.h"

//#define DEBUG_MAVLINK 1
bool mavlinkDetected;

#define MAVLINK_GET_CHANNEL_BUFFER
#define MAVLINK_GET_CHANNEL_STATUS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink/mavlink_types.h"

// IDs of what we believe is our controller station.
// These could be used for verifying that control messages are really from there
// (not done now).
// For some strange reason, the mavlink_msg_mount_status message has target system
// and component IDs. We use these IDs for that.
static int myGCSSystemId;
static int myGCSComponentId;

extern "C" {
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
}

mavlink_system_t mavlink_system;

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t c) {
	serial0.put(c);
}

#define MAVLINK_MESSAGE_CRC(msgid) pgm_read_byte(mavlink_message_crcs+msgid)
extern const uint8_t mavlink_message_crcs[256] PROGMEM;

#include "mavlink/ardupilotmega/mavlink.h"
#include "mavlink/ardupilotmega/ardupilotmega.h"

const uint8_t mavlink_message_crcs[256] PROGMEM = MAVLINK_MESSAGE_CRCS;

#define CHAN MAVLINK_COMM_0

struct MavlinkStoredGimbalOrientation {
	bool valid;
	int32_t pitchOrLat;	// in centidegrees or in lat*10E7
	int32_t rollOrLon;	// in centidegrees or in lon*10E7
	int32_t yawOrAlt;	// in centidegrees or in mm
	uint16_t crc;

	uint16_t calculateCRC() {
		return crc16((uint8_t*) this, sizeof(MavlinkStoredGimbalOrientation) - 2);
	}
};

static MavlinkStoredGimbalOrientation eeMavlinkGimbalOrientation EEMEM;
static MavlinkStoredGimbalOrientation mavlinkGimbalOrientation;
static uint8_t mavlinkGimbalMode;

void restoreMavlinkTarget() {
	eeprom_read_block(&mavlinkGimbalOrientation, &eeMavlinkGimbalOrientation, sizeof(MavlinkStoredGimbalOrientation));
	if (mavlinkGimbalOrientation.calculateCRC() != mavlinkGimbalOrientation.crc) {
		mavlinkGimbalOrientation.valid = false; // do not set true otherwise. The eeprom value should be valid, no need to overwrite it.
	}
}

void mavlink_init() {
	mavlink_system.sysid = config.mavlinkSystemId;
	mavlink_system.compid = config.mavlinkComponentId;
	mavlink_system.type = MAV_TYPE_ANTENNA_TRACKER; // Bah there is no gimbal type?
	restoreMavlinkTarget();
}

// We use the same message both in parsing and in interpretation.
// That means we have to fish out all data needed from an incoming
// message before we even begin parsing another.
// (interrupt based parsing is a no no)
static mavlink_message_t receivedMessage;
// static mavlink_message_t sendingMessage;
//uint8_t mavlinkUseRelativealtitudes = 0;
//int16_t mavlinkTargetPitch;
//int16_t mavlinkTargetBearing;

// How many meters per time unit (one unit being 510 clock cycles)
static float vlat_m_unit;
static float vlon_m_unit;

static float lonDiff_m;
static float latDiff_m;

static int16_t airframeRoll_nd;
static int16_t airframePitch_nd;
static int16_t airframeYaw_nd;

static int16_t prevYawAngle;

uint32_t unitsAccountedFor = -1;

/*
 * This instance is used in communication with the client (us).
 * There is another internal instance also.
 */
static mavlink_status_t mavlinkStatus;


mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
	return &receivedMessage;
}

mavlink_status_t* mavlink_get_channel_status(uint8_t chan) {
	return &mavlinkStatus;
}

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
 // My interpretation: Retract. Ignore the "load and keep safe position" part, just do whatever is best for retracting.

 MAV_MOUNT_MODE_NEUTRAL=1, / * Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | * /
  * My interpretation: Track the attitude of the airframe.

 MAV_MOUNT_MODE_MAVLINK_TARGETING=2, / * Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | * /
  * My interpretation: Stabilize to attitudes given by the MOUNT_CONTROL message (we need to be able to _not_ assume these data
  * are angles in the case they are a GPS position for GPS_POINT mode. So we can't always regard them as angle input).
  *
 MAV_MOUNT_MODE_RC_TARGETING=3, / * Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | * /
  * My interpretation: Do plain old RC mode.

 MAV_MOUNT_MODE_GPS_POINT=4, / * Load neutral position and start to point to Lat,Lon,Alt | * /
  * My interpretation: Do POI. RC may be added, why not?

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
	mavlink_updateTarget();
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
	myGCSSystemId = receivedMessage.sysid;
	myGCSComponentId = receivedMessage.compid;
	// target position
	if (mavlink_msg_mount_control_get_target_system(&receivedMessage) == config.mavlinkSystemId
			&& mavlink_msg_mount_control_get_target_component(&receivedMessage) == config.mavlinkComponentId) {
		mavlinkGimbalOrientation.pitchOrLat = mavlink_msg_mount_control_get_input_a(&receivedMessage);
		mavlinkGimbalOrientation.rollOrLon = mavlink_msg_mount_control_get_input_b(&receivedMessage);
		mavlinkGimbalOrientation.yawOrAlt = mavlink_msg_mount_control_get_input_c(&receivedMessage);
		mavlinkGimbalOrientation.valid = true;
	}

	uint8_t save = mavlink_msg_mount_control_get_save_position(&receivedMessage);

	if (save) {
		mavlinkGimbalOrientation.crc = mavlinkGimbalOrientation.calculateCRC();
		eeprom_update_block(&mavlinkGimbalOrientation, &eeMavlinkGimbalOrientation, sizeof(MavlinkStoredGimbalOrientation));
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
	if (!mavlinkGimbalOrientation.valid)
		return; // We don't know what to aim at
	latDiff_m = (mavlinkGimbalOrientation.pitchOrLat - globalPosition.lat) * 0.011112f;
	// from cm/sec to m/units: Multiply by sec/unit
	// n2 = v / (m/unit) = v * unit/m
	// n =  v / (cm/sec) = v * sec/cm
	// n2 = f n
	// f = n2/n = unit/m / sec/cm = cm*COARSE_TIME_UNITS*sec / (sec*m) = COARSE_TIME_UNITS/100
	vlat_m_unit = globalPosition.vx * (COARSE_TIME_UNITS / 100.0f);
	float latAsRadians = (float) globalPosition.lat / (1E7 * 180.0 / M_PI);
	float lonScale = cosf(latAsRadians); // should now be on a -PI/2..PI/2 scale.
	lonDiff_m = (mavlinkGimbalOrientation.rollOrLon - globalPosition.lon) * lonScale * 0.011112f;
	vlon_m_unit = globalPosition.vy * (COARSE_TIME_UNITS / 100.0f);
	unitsAccountedFor = coarseTime();
}

void processAttitude() {
	airframeRoll_nd = radiansToND(mavlink_msg_attitude_get_roll(&receivedMessage));
	airframePitch_nd = radiansToND(mavlink_msg_attitude_get_pitch(&receivedMessage));
	airframeYaw_nd = radiansToND(mavlink_msg_attitude_get_yaw(&receivedMessage));
}

#if defined (DEBUG_MAVLINK)
// Using this function DESTROYS the received message, watch out!
void sendMavlinkTextMessage_P(const char* pattern, ...) {
	char buffer[50];
	va_list argp;
	va_start(argp, pattern);
	sprintf_P(buffer, pattern, argp);
	va_end(argp);
	//mavlink_msg_statustext_send(CHAN, MAV_SEVERITY_DEBUG, buffer);
	mavlink_msg_statustext_send_buf(
			&receivedMessage, CHAN, MAV_SEVERITY_DEBUG, buffer);
}
#endif

bool mavlink_parse() {
	bool valid = false;
	if (!mavlinkDetected) serial0.mark();

	while (serial0.available()) {
		uint8_t parseResult = mavlink_parse_char(CHAN, serial0.get(), &receivedMessage, &mavlinkStatus);
		if (parseResult) {
			LEDEvent(LED_MAVLINK_RX);
			valid = true;
			uint8_t msgid = receivedMessage.msgid;
			if (msgid == MAVLINK_MSG_ID_MOUNT_CONFIGURE) {
				// Expected source: Our ground station
				processConfigureMount();
#if defined (DEBUG_MAVLINK)
					sendMavlinkTextMessage_P(PSTR("MAVLINK_MSG_ID_MOUNT_CONFIGURE, mode %d\r\n"), mavlinkGimbalMode);
#endif
			} else if (msgid == MAVLINK_MSG_ID_MOUNT_CONTROL) {
				// Expected source: Our ground station
				processControlMount();
#if defined (DEBUG_MAVLINK)
					sendMavlinkTextMessage_P(PSTR("MAVLINK_MSG_ID_MOUNT_CONTROL, orientation %ld %ld %ld\r\n"),
							gimbalOrientation.pitchOrLat, gimbalOrientation.rollOrLon, gimbalOrientation.yawOrAlt);
#endif
			} else if (msgid == MAVLINK_MSG_ID_ATTITUDE) {
				// Expected source: Own aircraft
				processAttitude();
			} else if (msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
				// Expected source: Own aircraft.
				// (Future addition: Also receive this from target aircraft)
				processGlobalPosition();
#if defined (DEBUG_MAVLINK)
					sendMavlinkTextMessage_P(PSTR("MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "
							"lat %ld, long %ld, alt %ld ralt %ld\r\n"), globalPosition.lat, globalPosition.lon,
							globalPosition.alt, globalPosition.relative_alt);
#endif
			}
		}
	}

	if (valid) mavlinkDetected = true;
	// Un-consume the data if there was no MAVLink
	else if (!mavlinkDetected) serial0.restore();

	return valid;
}

/*
 * We are looking at a range of about 31.34 each 90 degrees, or 286.875 centi-degrees/step
 */

#ifdef SUPPORT_YAW_SERVO

extern void setYawServoOut(uint16_t usec);

void setYawServo(int16_t angle) {
	int16_t yawServoLimit = degreesToND(config.yawServoLimit);
	if (angle > yawServoLimit) {
		angle = yawServoLimit;
		// Avoid full end to end turns when flying away from target: If we were left before and now full right, stay as before.
		if (prevYawAngle < 0)
			angle = -angle;
	} else if (angle < -yawServoLimit) {
		angle = -yawServoLimit;
		// Avoid full end to end turns when flying away from target: If we were right before and now full left, stay as before.
		if (prevYawAngle > 0)
			angle = -angle;
	}
	prevYawAngle = angle;

	// There are 16384 nd to 90 degrees=1000 usec
	uint16_t usec = (config.yawServoDirection * angle / 16) + 1500;
	setYawServoOut(usec);
}
#endif

void mavlink_trackPOI(int16_t* pitchResult) {
	uint32_t timeUnits = coarseTime();
	uint32_t unitsToDo = timeUnits - unitsAccountedFor;
	unitsAccountedFor = timeUnits;

	lonDiff_m -= unitsToDo * vlon_m_unit;
	latDiff_m -= unitsToDo * vlat_m_unit;
	float targetBearing_rad = Rajan_FastArcTan2(lonDiff_m, latDiff_m);

	int16_t targetBearing = radiansToND(targetBearing_rad);
	float dist_m = sqrtf(latDiff_m * latDiff_m + lonDiff_m * lonDiff_m);

	targetBearing -= airframeYaw_nd;

	/*
	if (targetBearing <= -18000)
		targetBearing = targetBearing + 36000;
	else if (targetBearing > 18000)
		targetBearing = targetBearing - 36000;
	*/

	printf("Yaw O shit %d\r\n", airframeYaw_nd);

	setYawServo(targetBearing);

	float dAlt;
	if (config.mavlinkUseRelativealtitudes)
		dAlt = (mavlinkGimbalOrientation.yawOrAlt - globalPosition.relative_alt) / 1000.0f;
	else
		dAlt = (mavlinkGimbalOrientation.yawOrAlt - globalPosition.alt) / 1000.0f;
	float pitch = Rajan_FastArcTan2(dAlt, dist_m);

	*pitchResult = radiansToND(pitch);
}

// Call when 1)MAVLink is engaged 2) When
void mavlink_updateTarget() {
	switch(mavlinkGimbalMode){
	case MAV_MOUNT_MODE_RETRACT: /* Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | */
		retract();
		break;

	case MAV_MOUNT_MODE_MAVLINK_TARGETING: /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
		// restoreMavlinkTarget();
		// intentionally no break
	case MAV_MOUNT_MODE_NEUTRAL: /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
	case MAV_MOUNT_MODE_RC_TARGETING: /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
	case MAV_MOUNT_MODE_GPS_POINT:	 /* Load neutral position and start to point to Lat,Lon,Alt | */
		run();
		break;
	}
}

// Called at regular intervals
void mavlink_update() {
	int16_t tempRoll = 0;
	int16_t tempPitch = 0;

	if (mavlinkGimbalMode == MAV_MOUNT_MODE_GPS_POINT && mavlinkGimbalOrientation.valid) {
		mavlink_trackPOI(&tempPitch);
	} else if (mavlinkGimbalMode == MAV_MOUNT_MODE_MAVLINK_TARGETING && mavlinkGimbalOrientation.valid) {
		// Use MOUNT_CONTROL data directly as angles
		tempRoll = centidegreesToND(mavlinkGimbalOrientation.rollOrLon);
		tempPitch = centidegreesToND(mavlinkGimbalOrientation.pitchOrLat);
	} else if (mavlinkGimbalMode == MAV_MOUNT_MODE_RC_TARGETING) {
		// Copy RC values
		// tempRoll = 0;  //targetSources[TARGET_SOURCE_RC][ROLL];
		// tempPitch = 0; //targetSources[TARGET_SOURCE_RC][PITCH];
	} else if (mavlinkGimbalMode == MAV_MOUNT_MODE_NEUTRAL) {
		// Follow airframe (FPV-a-like). This is different from MAVLink original intention but better suited for a brushless.
		tempRoll = airframeRoll_nd;
		tempPitch = airframePitch_nd;
	}

	targetSources[TARGET_SOURCE_MAVLINK][ROLL] = tempRoll + targetSources[TARGET_SOURCE_RC][ROLL];
	targetSources[TARGET_SOURCE_MAVLINK][PITCH] = tempPitch + targetSources[TARGET_SOURCE_RC][PITCH];
}

void mavlink_sendStatus() {
	mavlink_msg_mount_status_send
	(CHAN, myGCSSystemId, myGCSComponentId,
			NDToCentidegrees(imu.angle_i16[PITCH]),
			NDToCentidegrees(imu.angle_i16[ROLL]),
			prevYawAngle);
	//myStatus.pointing_a = NDToCentidegrees(getTarget(PITCH));
	//myStatus.pointing_b = NDToCentidegrees(getTarget(ROLL));
	//myStatus.pointing_c = mavlinkTargetBearing;
}

void mavlink_sendHeartbeat() {
	/*
	uint16_t length = mavlink_msg_heartbeat_pack_chan(
			config.mavlinkSystemId,
			config.mavlinkComponentId,
			CHAN,
			&sendingMessage,
			MAV_TYPE_ANTENNA_TRACKER,
			18,
			MAV_MODE_FLAG_STABILIZE_ENABLED,
			0,
			0);

	for (uint8_t i=0; i<length; i++) {
		serial0.put(sendingMessage[i]);
	}
	*/

	mavlink_msg_heartbeat_send(
			CHAN,
			MAV_TYPE_ANTENNA_TRACKER,
			18,
			MAV_MODE_FLAG_STABILIZE_ENABLED,
			0,
			0);
}

#endif
