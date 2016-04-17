#include <FastSerial.h>
#include <SoftwareSerial.h>
#include <GCS_MAVLink.h>
#include <TM1637Display.h>
#include <TinyGPS++.h>
#include <WGS84_in_GK.h>
#include <GK_in_WGS84.h>

#define CLK 5
#define DIO 3
#define Lat 0
#define Lon 1
#define R 0
#define H 1
#define distancetouser 10
// A
//   ---
// F|   | B
//   -G-
// E|   | C
//   ---
// 
const uint8_t SEG_STOP[] = {
	SEG_A | SEG_F | SEG_G| SEG_C | SEG_D,           // d
	SEG_F | SEG_G | SEG_E | SEG_D,   // O
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,                           // n
	SEG_A | SEG_B | SEG_E | SEG_F| SEG_G,        // E
};
uint8_t    ap_type = MAV_TYPE_GENERIC;
uint8_t    ap_autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 1;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

//<Message #76 COMMAND_LONG - using the mavlink_msg_command_long_pack() function
uint8_t target_system = 1;// - not really sure why this has to be at a valu of 1
uint8_t target_component = 0;
uint16_t CMD_LONG_command = 0;
uint8_t  CMD_LONG_confirmation = 0;
double copter_lat;
double copter_lon;
float CMD_LONG_param1 = 0;
float CMD_LONG_param2 = 0;
float CMD_LONG_param3 = 0;
float CMD_LONG_param4 = 0;
float CMD_LONG_param5 = 0;
float CMD_LONG_param6 = 0;
float CMD_LONG_param7 = 0;

enum autopilot_modes 
{
	STABILIZE = 0,  // manual airframe angle with manual throttle
	ACRO = 1,  // manual body-frame angular rate with manual throttle
	ALT_HOLD = 2,  // manual airframe angle with automatic throttle
	AUTO = 3,  // fully automatic waypoint control using mission commands
	GUIDED = 4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
	LOITER = 5,  // automatic horizontal acceleration with automatic throttle
	RTL = 6,  // automatic return to launching point
	CIRCLE = 7,  // automatic circular flight with automatic throttle
	LAND = 9,  // automatic landing with horizontal position control
	DRIFT = 11,  // semi-automous position, yaw and throttle control
	SPORT = 13,  // manual earth-frame angular rate control with manual throttle
	FLIP = 14,  // automatically flip the vehicle on the roll axis
	AUTOTUNE = 15,  // automatically tune the vehicle's roll and pitch gains
	POSHOLD = 16,  // automatic position hold with manual override, with automatic throttle
	BRAKE = 17   // full-brake using inertial/GPS system, no pilot input
};
//Mavlink COmms
unsigned long hb_timer;
uint8_t     MavLink_Connection_Status;//no communication initialy



//FastSerialPort1(Serial);
 FastSerialPort0(Serial);
SoftwareSerial sserial(9, 6);
TM1637Display display(CLK, DIO);
TinyGPSPlus gps;
//Standard requireemnts for packets to have
uint8_t system_id = 255;
uint8_t component_id = 200;
#define CLK 5
#define DIO 4

int REL_ALT = 0;
int DES_REL_ALT = 30;



WGS84_in_GK WGS_TO_GK;
GK_in_WGS84 GK_TO_WGS84;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long Poll = 1500;
unsigned long currentMillisB = 0;
unsigned long previousMillisB = 0;
const long PollB = 300;
boolean state = false;

void setup() 
{
	
	Serial.begin(57600);
	//Serial.println("tttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt");
	//Serial.println("test");
	//Serial.begin(9600);
	sserial.begin(9600);

	//Mavlink COMMS
	MavLink_Connection_Status = 0;
	hb_timer = millis();
	MAV_ALTITUDE_REQUEST();
	pinMode(13, OUTPUT);
	pinMode(4, INPUT_PULLUP);
	pinMode(2, INPUT_PULLUP);
	pinMode(7, INPUT_PULLUP);
	delay(300);
	
	while (digitalRead(4))
	{

	}
	//
	//display.setBrightness(0x0F);
	//display.showNumberDec(0000, true);
	//WGS_TO_GK.Convert_to_GK(51, 10);

	//Serial.println(WGS_TO_GK.Convert_to_GK(51, 10)[R]);
	//Serial.println(WGS_TO_GK.Convert_to_GK(51, 10)[H]);
	/*String la(GK_TO_WGS84.Convert_to_WGS(5291963.114, 3512987.517)[Lat],6);
	String lo(GK_TO_WGS84.Convert_to_WGS(5291963.114, 3512987.517)[Lon],6);*/

}

void loop()
{
	if (!digitalRead(2))
	{
		if (!digitalRead(7))
		{
			MAV_SET_WP(copter_lat, copter_lon, DES_REL_ALT);
			display.setSegments(SEG_STOP);
			while(1){
				delay(1000);
			}
		}

	}
	
	while (!digitalRead(4))
	{
		if (!digitalRead(2))
		{
			DES_REL_ALT++;
			delay(200);

		}

		if (!digitalRead(7))
		{
			DES_REL_ALT--;
			delay(200);
		}
		currentMillisB = millis();
		if (currentMillisB - previousMillisB >= PollB) {

			previousMillisB = currentMillisB;
			if (state) {
				state = false;
				display.setBrightness(0x0F);
				display.showNumberDec(DES_REL_ALT, true);
			}
			else {
				state = true;
				display.setBrightness(0x00);
				display.showNumberDec(DES_REL_ALT, true);
			}


		}
	}
	display.setBrightness(0x0F);
	if (gps.satellites.value() > 4)
	{
		display.showNumberDec(REL_ALT, true);
	}
	

	//display.setBrightness(0x0F);
	//display.showNumberDec(REL_ALT, true);
	
	//if (Serial.available()) 
	//{
	//	int Read = Serial.read();
	//	if (Read == 97)
	//	{
	//		Serial.println("ARMING-");
	//		//
	//		MAV_SEND_COMMAND(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0);
	//	}
	//	if (Read == 98)
	//	{
	//		MAV_RC_OVERRIDE(0, 0, 5000, 0, 0, 0, 0, 0);
	//		Serial.println("OVERRIDE");
	//	}
	//	if (Read == 100)
	//	{
	//		Serial.println("-DISARMING");
	//		MAV_SEND_COMMAND(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0);
	//	}
	//	if (Read == 99)
	//	{
	//		Serial.println("CLEARED");
	//		MAV_MISSION_CLEAR();
	//	}
	//	if (Read == 101)
	//	{
	//		MAV_SET_MODE(GUIDED);
	//	}
	//	if (Read == 98) //B
	//	{
	//	}
	//	if (Read == 110) //N
	//	{
	//		if (Serial.available()) {
	//			int Read = Serial.read();
	//			if (Read == 97)
	//			{
	//			}
	//		}
	//	}
	//	if (Read == 109) //M
	//	{
	//	}
	
	comm_receive();
	while (sserial.available() > 0)
	{
		gps.encode(sserial.read());
	}
	
	currentMillis = millis();
	
	if (currentMillis - previousMillis >= Poll) {
		
		previousMillis = currentMillis;
		if (gps.satellites.value()> 4)
		{
			UpdatePosition();
		}
		else
		{
			display.showNumberDec(gps.satellites.value()+1000, true);
		}
			
	}
	
}

void UpdatePosition()
{
	
	
	double newLat;
	double newLon;
	calc_offset(gps.location.lat()/*47.77502632*/, gps.location.lng()/*9.13501382*/, 10, newLat, newLon);
    MAV_SEND_COMMAND(201, MAV_ROI_LOCATION, 1, 1, 0, gps.location.lat(), gps.location.lng(), DES_REL_ALT); //REL ALT BECAUSE NO GIMBAL
	MAV_SET_WP(newLat, newLon, DES_REL_ALT);
	
}
//double *copter()
//{
//	double test[2];
//	test[Lat] = 47.765543;
//	test[Lon] = 9.172114;
//	return test;
//}
void calc_offset(double my_lat, double my_lon, int offset, double& newLat, double& newLon)
{
	double my_R;
	double my_H;
	WGS_TO_GK.Convert_to_GK(my_lat, my_lon, my_R, my_H);
	double copter_R;
	double copter_H;
	WGS_TO_GK.Convert_to_GK(copter_lat, copter_lon, copter_R, copter_H);
	double dR = my_R - copter_R;
	double dH = my_H - copter_H;
	double len = sqrt(dR*dR + dH*dH);
	double newcopterR = copter_R;
	double newcopterH = copter_H;
	if (len > distancetouser)
	{
		dR /= len;
		dH /= len;
		newcopterR = my_R - dR * offset;
		newcopterH = my_H - dH * offset;
	}
	GK_TO_WGS84.Convert_to_WGS(newcopterH, newcopterR, newLat, newLon);
}
void comm_receive() {
	

	//MAV_MISSION_REQUEST();
	mavlink_message_t msg;
	mavlink_status_t status;

	//receive data over Serial 
	while (Serial.available() > 0) {
		uint8_t c = Serial.read();

		//try to get a new message 
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch (msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				digitalWrite(13, HIGH);
				mavlink_heartbeat_t packet;
				mavlink_msg_heartbeat_decode(&msg, &packet);
				//Serial.println(packet.system_status);


				//Serial.println(packet.count);
				break;
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
				//digitalWrite(13, HIGH);
				mavlink_global_position_int_t packet;
				mavlink_msg_global_position_int_decode(&msg, &packet);
				//Serial.println(packet.alt / 1000);
				//display.showNumberDec(packet.relative_alt / 1000, true);
				REL_ALT = packet.relative_alt / 1000;
				copter_lat = static_cast<double>(packet.lat)/1E7;
				copter_lon = static_cast<double>(packet.lon)/1E7;
			}
													 break;
													 /*
										 case MAVLINK_MSG_ID_ACTION:
										 // EXECUTE ACTION
									 break;
													 */
			default:
				//Do nothing
				break;
			}
		}
		// And get the next one
	}
}
void Send_Heart_Beat() 
{
	// Initialize the required buffers 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	// mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
	mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, ap_base_mode, ap_custom_mode, ap_system_status);

	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
}

void MAV_SEND_COMMAND(unsigned int cmdid, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	//mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg, target_system, target_component, 10, 10, 10, 10, 10, 10, 10, 10);
	CMD_LONG_param1 = param1;
	CMD_LONG_param2 = param2;
	CMD_LONG_param3 = param3;
	CMD_LONG_param4 = param4;
	CMD_LONG_param5 = param5;
	CMD_LONG_param6 = param6;
	CMD_LONG_param7 = param7;
	CMD_LONG_command = cmdid;

	mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, CMD_LONG_command, CMD_LONG_confirmation, CMD_LONG_param1, CMD_LONG_param2, CMD_LONG_param3, CMD_LONG_param4, CMD_LONG_param5, CMD_LONG_param6, CMD_LONG_param7);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);

}
void MAV_RC_OVERRIDE(float param1, float param2, float param3, float param4, float param5, float param6, float param7, float param8)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg, target_system, target_component, param1, param2, param3, param4, param5, param6, param7, param8);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_MISSION_CLEAR()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_mission_clear_all_pack(system_id, component_id, &msg, target_system, target_component);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_MISSION_LIST_REQUEST()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_mission_request_list_pack(system_id, component_id, &msg, target_system, target_component);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_ALTITUDE_REQUEST()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, MAV_DATA_STREAM_POSITION, 10000000, 1);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_MISSION_REQUEST()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_mission_request_pack(system_id, component_id, &msg, target_system, target_component, 1);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_SET_MODE(int mode)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_set_mode_pack(system_id, component_id, &msg, target_system, 1, mode);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_SET_GUID(float LAT, float LONG, int ALT)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];


	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);


}

void MAV_SET_ROI(float LAT, float LONG, int ALT)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	//mavlink_(system_id, component_id, &msg, target_system, 1, mode);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);

}

void ROI(int roi)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	//mavlink_msg_set_mode_pack(system_id, component_id, &msg, target_system, 1, mode);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);


}

void MAV_SET_WP(float x, float y, float z)
{
	
	uint16_t seq = 0; // Sequence is always set to 0
	uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // Set target frame to global default
	uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for APM as it uses different paramters than the standard MAVLINK implementation
	uint8_t current = 2; // Guided mode waypoint
	uint8_t autocontinue = 0; // Always 0
	float param1 = 0; // Loiter time
	float param2 = 1; // Acceptable range from target - radius in meters
	float param3 = 0; // Pass through waypoint
	float param4 = 0; // Desired yaw angle


				   // Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	// NOTE : _target_systen & _target_component are defined in setup
	// found by searching mission item .h from mavlink library
	mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

void MAV_SET_CURR()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_mission_set_current_pack(system_id, component_id, &msg, target_system, target_component, 1);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	Serial.write(buf, len);
}

