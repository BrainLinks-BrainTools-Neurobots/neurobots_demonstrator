#ifndef WSG_GRIPPER_H
#define WSG_GRIPPER_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <wsg_gripper/GripperCommandAction.h>
#include <stdexcept>

#define MAX_PACKET_SIZE 4096
#define RX_TIMEOUT 100000
#define POLLING_PERIOD 100

// System state flags
enum e_system_state
{
	SF_REFERENCED = 0,
	SF_MOVING,
	SF_BLOCKED_MINUS,
	SF_BLOCKED_PLUS,
	SF_SOFT_LIMIT_MINUS,
	SF_SOFT_LIMIT_PLUS,
	SF_AXIS_STOPPED,
	SF_TARGET_POS_REACHED,
	SF_OVERDRIVE_MODE,
	SF_RESERVED_9,
	SF_RESERVED_10,
	SF_RESERVED11,
	SF_FAST_STOP,
	SF_TEMP_WARNING,
	SF_TEMP_FAULT,
	SF_POWER_FAULT,
	SF_CURR_FAULT,
	SF_FINGER_FAULT,
	SF_CMD_FAILURE,
	SF_SCRIPT_RUNNING,
	SF_SCRIPT_FAILURE
};

//! Error codes
enum e_error
{
	E_SUCCESS = 0,
	E_NOT_AVAILABLE,
	E_NO_SENSOR,
	E_NOT_INITIALIZED,
	E_ALREADY_RUNNING,
	E_FEATURE_NOT_SUPPORTED,
	E_INCONSISTENT_DATA,
	E_TIMEOUT,
	E_READ_ERROR,
	E_WRITE_ERROR,
	E_INSUFFICIENT_RESOURCES,
	E_CHECKSUM_ERROR,
	E_NO_PARAM_EXPECTED,
	E_NOT_ENOUGH_PARAMS,
	E_CMD_UNKNOWN,
	E_CMD_FORMAT_ERROR,
	E_ACCESS_DENIED,
	E_ALREADY_OPEN,
	E_CMD_FAILED,
	E_CMD_ABORTED,
	E_INVALID_HANDLE,
	E_NOT_FOUND,
	E_NOT_OPEN,
	E_IO_ERROR,
	E_INVALID_PARAMETER,
	E_INDEX_OUT_OF_BOUNDS,
	E_CMD_PENDING,
	E_OVERRUN,
	E_RANGE_ERROR,
	E_AXIS_BLOCKED,
	E_FILE_EXISTS
};

// system state messages
#define SF_SCRIPT_FAILURE_MSG "Script Error"
#define SF_SCRIPT_RUNNING_MSG "A script is currently running"
#define SF_CMD_FAILURE_MSG "Command Error"
#define SF_FINGER_FAULT_MSG "Finger Fault"
#define SF_CURR_FAULT_MSG "Engine Current Error"
#define SF_POWER_FAULT_MSG "Power Error"
#define SF_TEMP_FAULT_MSG "Temperature Error"
#define SF_TEMP_WARNING_MSG "Temperature Warning"
#define SF_FAST_STOP_MSG "Fast Stop"
#define SF_OVERDRIVE_MODE_MSG "Overdrive Mode"
#define SF_TARGET_POS_REACHED_MSG "Target position reached"
#define SF_AXIS_STOPPED_MSG "Axis stopped"
#define SF_SOFT_LIMIT_PLUS_MSG "Positive direction soft limit reached"
#define SF_SOFT_LIMIT_MINUS_MSG "Negative direction soft limit reached"
#define SF_BLOCKED_PLUS_MSG "Axis is blocked in positive moving direction"
#define SF_BLOCKED_MINUS_MSG "Axis is blocked in negative moving direction"
#define SF_MOVING_MSG "The Fingers are currently moving"
#define SF_REFERENCED_MSG "Fingers Referenced"

// error messages
#define E_SUCCESS_MSG "No error occurred, operation was successful"
#define E_NOT_AVAILABLE_MSG "Function or data is not available"
#define E_NO_SENSOR_MSG "No measurement converter is connected"
#define E_NOT_INITIALIZED_MSG "Device was not initialized"
#define E_ALREADY_RUNNING_MSG "The data acquisition is already running"
#define E_FEATURE_NOT_SUPPORTED_MSG "The requested feature is currently not available"
#define E_INCONSISTENT_DATA_MSG "One or more parameters are inconsistent"
#define E_TIMEOUT_MSG "Timeout error"
#define E_READ_ERROR_MSG "Error while reading data"
#define E_WRITE_ERROR_MSG "Error while writing data"
#define E_INSUFFICIENT_RESOURCES_MSG "No more memory available"
#define E_CHECKSUM_ERROR_MSG "Checksum error"
#define E_NO_PARAM_EXPECTED_MSG "A Parameter was given, but none expected"
#define E_NOT_ENOUGH_PARAMS_MSG "Not enough parameters for executing the command"
#define E_CMD_UNKNOWN_MSG "Unknown command"
#define E_CMD_FORMAT_ERROR_MSG "Command format error"
#define E_ACCESS_DENIED_MSG "Access denied"
#define E_ALREADY_OPEN_MSG "Interface is already open"
#define E_CMD_FAILED_MSG "Error while executing a command"
#define E_CMD_ABORTED_MSG "Command execution was aborted by the user"
#define E_INVALID_HANDLE_MSG "Invalid handle"
#define E_NOT_FOUND_MSG "Device or file not found"
#define E_NOT_OPEN_MSG "Device or file not open"
#define E_IO_ERROR_MSG "Input/Output Error"
#define E_INVALID_PARAMETER_MSG "Wrong parameter"
#define E_INDEX_OUT_OF_BOUNDS_MSG "Index out of bounds"
#define E_CMD_PENDING_MSG "No error, but the command was not completed, yet."\
" Another return message will follow including an error code,"\
" if the function was completed."
#define E_OVERRUN_MSG "Data overrun"
#define E_RANGE_ERROR_MSG "Range error"
#define E_AXIS_BLOCKED_MSG "Axis blocked"
#define E_FILE_EXISTS_MSG "File already exists"

class gripperError
{
public:
	explicit gripperError(const std::string& what_arg,
			uint16_t code_arg) :
					what_s(what_arg),
					code(code_arg)
	{
	}
	;
	std::string what()
	{
		return what_s;
	}
	std::string what_s;
	uint16_t code;
};

// crc16 lookup table
const unsigned short CRC_TABLE_CCITT16[256] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
		0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
		0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
		0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
		0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
		0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
		0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
		0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
		0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
		0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
		0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
		0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
		0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
		0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
		0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
		0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
		0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
		0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
		0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
		0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
		0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
		0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

// helper class for parsing the tcp stream
class WsgParser
{
public:
	enum parserstate_t
	{
		Error,
		WaitingForPreamble,
		WaitingForId,
		WaitingForPacketSize,
		WaitingForData,
		WaitingForChecksum,
		PacketReceived
	};

	WsgParser() :
					state(WaitingForPreamble),
					byte_count(0),
					size_LSB(0),
					size_MSB(0),
					packet_size(0)
	{

	}
	// state variables
	parserstate_t state;
	uint32_t byte_count;
	uint8_t size_LSB;
	uint8_t size_MSB;
	uint16_t packet_size;

	// data storage
	boost::array<unsigned char, MAX_PACKET_SIZE + 10> buf;

	void
	onByteReceived(uint8_t byte);
	void
	reset();
};

// helper class for low-level packet management
class WsgPacket
{

public:
	WsgPacket(uint8_t id) :
					id(id),
					size(0),
					checksum(0xFFFF)
	{
	}

public:
	// type-specific getter functions
	uint32_t
	getSystemState();
	std::vector<uint8_t> getFingerData();
	uint8_t getFingerFlags();
	uint8_t getFingerType();
	uint8_t getFingerDataSize();
	float getAcceleration();
	float getGraspingForceLimit();
	float getOpeningWidth();

	uint16_t
	getErrorCode();
	std::vector<uint8_t> getPayload()
	{
		return payload;
	}
	std::vector<uint8_t> getRaw()
	{
		return raw_data;
	}
	uint8_t getId()
	{
		return id;
	}
	void
	setId(uint8_t new_id);
	void
	setPayload(std::vector<uint8_t>& new_payload);
	void
	setRaw(std::vector<uint8_t>& new_raw_data);
	std::string
	print();

private:
	uint8_t id;
	uint16_t size;
	std::vector<uint8_t> payload;
	uint16_t checksum;
	std::vector<uint8_t> raw_data;

	void
	generatePacketData();
	void
	generateChecksum(std::vector<uint8_t>& data);
	void
	parseRawData();
	bool
	validateChecksum();
};

// gripper class which provides a subset of the wsg gripper command API
class WsgGripper
{

public:
	WsgGripper(const char* ip_addr,
			uint32_t tcp_port,
			long connection_timeout_seconds);

	void connected_callback(const boost::system::error_code& error,
			boost::asio::deadline_timer* timer);
	// gripper API
	void
	ackFastStop();
	void
	graspPart(float width,
			float speed);
	void
	releasePart(float width,
			float speed);
	void
	setAcceleration(float acc);
	void getAcceleration();
	void
	setGraspingForceLimit(float force);
	void getGraspingForceLimit();
	void
	getFingerInfo(uint8_t index);
	void
	getFingerFlags(uint8_t index);
	void
	fingerPowerControl(uint8_t index,
			bool on_off);
	void
	getFingerData(uint8_t index);
	void
	Homing();
	void
	moveFingers(float opening_width,
			float traveling_speed,
			bool relative_movement = false,
			bool stop_on_block = false);
	void
	getSystemState();

	void
	getOpeningWidth();

	void
	readWithTimeout(uint32_t timeout);

	const char*
	resolveErrorCode(uint16_t code);
	const char*
	resolveStatusCode(uint32_t code);

	uint8_t last_error;
	bool cmd_successful;
	bool connected_to_gripper;
	bool cmd_pending;
	float acc_limit;
	float force_limit;
	uint32_t system_state;
	float opening_width;
	uint8_t finger_type;
	uint8_t finger_data_size;
	std::vector<uint8_t> finger_data;
	uint8_t finger_flags;
	std::vector<float> finger_force;
	private:
	const char* ip_addr;
	uint32_t tcp_port;
	long connection_timeout_sec;

	boost::asio::io_service* io_service;
	boost::asio::ip::tcp::socket* socket;
	boost::array<unsigned char, MAX_PACKET_SIZE> rx_buf;
	WsgPacket rx_packet;
	size_t rx_packet_size;
	WsgParser parser;

	static const char* sf_messages[];
	static const char* error_messages[];

	void
	handlePending(WsgPacket& p);
	void
	flushSocket();
	void
	openConnection();
	WsgPacket
	sendAndReceive(WsgPacket p);
	void
	checkError(uint16_t code);
	void
	checkRxPacket(WsgPacket& p,
			WsgPacket& r);
	WsgPacket
	processTcpPacket(boost::array<unsigned char, MAX_PACKET_SIZE> buf,
			size_t len);

};

#endif
