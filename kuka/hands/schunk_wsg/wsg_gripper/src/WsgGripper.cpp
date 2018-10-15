#include <wsg_gripper/WsgGripper.h>

const char* WsgGripper::sf_messages[] = { SF_REFERENCED_MSG, SF_MOVING_MSG, SF_BLOCKED_MINUS_MSG,
SF_BLOCKED_PLUS_MSG, SF_SOFT_LIMIT_MINUS_MSG, SF_SOFT_LIMIT_PLUS_MSG, SF_AXIS_STOPPED_MSG,
SF_TARGET_POS_REACHED_MSG, SF_OVERDRIVE_MODE_MSG, "reserved", "reserved", "reserved",
SF_FAST_STOP_MSG, SF_TEMP_WARNING_MSG, SF_TEMP_FAULT_MSG, SF_POWER_FAULT_MSG, SF_CURR_FAULT_MSG,
SF_FINGER_FAULT_MSG, SF_CMD_FAILURE_MSG, SF_SCRIPT_RUNNING_MSG, SF_SCRIPT_FAILURE_MSG };

const char* WsgGripper::error_messages[] = { E_SUCCESS_MSG, E_NOT_AVAILABLE_MSG, E_NO_SENSOR_MSG,
E_NOT_INITIALIZED_MSG, E_ALREADY_RUNNING_MSG, E_FEATURE_NOT_SUPPORTED_MSG,
E_INCONSISTENT_DATA_MSG, E_TIMEOUT_MSG, E_READ_ERROR_MSG, E_WRITE_ERROR_MSG,
E_INSUFFICIENT_RESOURCES_MSG, E_CHECKSUM_ERROR_MSG, E_NO_PARAM_EXPECTED_MSG,
E_NOT_ENOUGH_PARAMS_MSG, E_CMD_UNKNOWN_MSG, E_CMD_FORMAT_ERROR_MSG, E_ACCESS_DENIED_MSG,
E_ALREADY_OPEN_MSG, E_CMD_FAILED_MSG, E_CMD_ABORTED_MSG, E_INVALID_HANDLE_MSG, E_NOT_FOUND_MSG,
E_NOT_OPEN_MSG, E_IO_ERROR_MSG, E_INVALID_PARAMETER_MSG, E_INDEX_OUT_OF_BOUNDS_MSG,
E_CMD_PENDING_MSG, E_OVERRUN_MSG, E_RANGE_ERROR_MSG, E_AXIS_BLOCKED_MSG, E_FILE_EXISTS_MSG };

WsgGripper::WsgGripper(const char* ip_addr,
		uint32_t tcp_port,
		long connection_timeout_seconds) :
				connected_to_gripper(false),
				cmd_pending(false),
				system_state(0),
				ip_addr(ip_addr),
				tcp_port(tcp_port),
				connection_timeout_sec(connection_timeout_seconds),
				rx_packet(0),
				rx_packet_size(0)
{
	try
	{
		openConnection();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Could not connect to gripper: ");
		ROS_FATAL_STREAM(e.what());
		throw;
	}
	finger_force.resize(2);

}
void WsgGripper::checkRxPacket(WsgPacket& p,
		WsgPacket& r)
{
	// returned id must be identical to the sent one
	if (r.getId() != p.getId())
	{
		throw std::runtime_error(std::string("Rx packet has wrong id!"));
	}
	// any errors?
	checkError(r.getErrorCode());
}

void WsgGripper::getSystemState()
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_SYSTEM_STATE);
		std::vector<uint8_t> payload(3, 0);
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		system_state = r.getSystemState();

		// analyze system state
		std::stringstream sys;
		bool no_flag = true;
		sys << "System state: " << std::endl;
		uint8_t flag_ind = 0;
		ROS_DEBUG_STREAM("System state is " << system_state << ", analyzing...");
		while (flag_ind < 21)
		{
			if (flag_ind > 8 && flag_ind < 12)
			{ // reserved bits
				flag_ind++;
				continue;
			}
			if (system_state & (1 << flag_ind))
			{
				sys << "flag " << (int) flag_ind << ": " << resolveStatusCode(flag_ind) << std::endl;
				no_flag = false;
			}
			flag_ind++;
		}
		if (no_flag)
			sys << "No flags set." << std::endl;
		ROS_DEBUG_STREAM(sys.str());
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		throw;
		return;
	}
	cmd_successful = true;
}

void WsgGripper::getAcceleration()
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_ACCELERATION);
		WsgPacket r(0);
		std::vector<uint8_t> payload;
		p.setPayload(payload);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		acc_limit = r.getAcceleration();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::getGraspingForceLimit()
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_GRASPING_FORCE_LIMIT);
		WsgPacket r(0);
		std::vector<uint8_t> payload;
		p.setPayload(payload);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		force_limit = r.getGraspingForceLimit();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::getOpeningWidth()
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_OPENING_WIDTH);
		WsgPacket r(0);
		std::vector<uint8_t> payload(3, 0);
		p.setPayload(payload);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		opening_width = r.getOpeningWidth();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::Homing()
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::HOMING);
		std::vector<uint8_t> payload(1, 0);
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::ackFastStop()
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::ACK_FAST_STOP);
		std::vector<uint8_t> payload;
		payload.push_back('a');
		payload.push_back('c');
		payload.push_back('k');
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::graspPart(float width,
		float speed)
{
	cmd_successful = false;
	WsgPacket r(0);
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GRASP_A_PART);
		std::vector<uint8_t> payload;
		uint8_t* o = (uint8_t *) &width;
		uint8_t* t = (uint8_t *) &speed;
		payload.push_back(*o);
		payload.push_back(*(o + 1));
		payload.push_back(*(o + 2));
		payload.push_back(*(o + 3));
		payload.push_back(*t);
		payload.push_back(*(t + 1));
		payload.push_back(*(t + 2));
		payload.push_back(*(t + 3));
		p.setPayload(payload);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::releasePart(float width,
		float speed)
{
	cmd_successful = false;
	WsgPacket r(0);
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::MOVE_WITHOUT_FORCE_CONTROL);
		std::vector<uint8_t> payload;
		uint8_t* o = (uint8_t *) &width;
		uint8_t* t = (uint8_t *) &speed;
		payload.push_back(*o);
		payload.push_back(*(o + 1));
		payload.push_back(*(o + 2));
		payload.push_back(*(o + 3));
		payload.push_back(*t);
		payload.push_back(*(t + 1));
		payload.push_back(*(t + 2));
		payload.push_back(*(t + 3));
		p.setPayload(payload);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::setAcceleration(float acc)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::SET_ACCELERATION);
		std::vector<uint8_t> payload;
		uint8_t* o = (uint8_t *) &acc;
		payload.push_back(*o);
		payload.push_back(*(o + 1));
		payload.push_back(*(o + 2));
		payload.push_back(*(o + 3));
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::setGraspingForceLimit(float force)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::SET_GRASPING_FORCE_LIMIT);
		std::vector<uint8_t> payload;
		uint8_t* o = (uint8_t *) &force;
		payload.push_back(*o);
		payload.push_back(*(o + 1));
		payload.push_back(*(o + 2));
		payload.push_back(*(o + 3));
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::getFingerInfo(uint8_t index)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_FINGER_INFO);
		std::vector<uint8_t> payload;
		payload.push_back(index);
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		finger_type = r.getFingerType();
		finger_data_size = r.getFingerDataSize();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::getFingerFlags(uint8_t index)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_FINGER_FLAGS);
		std::vector<uint8_t> payload;
		payload.push_back(index);
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		finger_flags = r.getFingerFlags();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::fingerPowerControl(uint8_t index,
		bool on_off)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::FINGER_POWER_CONTROL);
		std::vector<uint8_t> payload;
		payload.push_back(index);
		payload.push_back(on_off);
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::getFingerData(uint8_t index)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::GET_FINGER_DATA);
		std::vector<uint8_t> payload;
		payload.push_back(index);
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
		finger_data = r.getFingerData();
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void WsgGripper::moveFingers(float width,
		float speed,
		bool relative_movement,
		bool stop_on_block)
{
	cmd_successful = false;
	try
	{
		if (!socket->is_open())
		{
			openConnection();
		}
		WsgPacket p(wsg_gripper::GripperCommandGoal::MOVE_FINGERS);
		std::vector<uint8_t> payload;
		payload.push_back((relative_movement << 0) | (stop_on_block << 1)); //FLAGS
		uint8_t* o = (uint8_t *) &width;
		uint8_t* t = (uint8_t *) &speed;
		payload.push_back(*o);
		payload.push_back(*(o + 1));
		payload.push_back(*(o + 2));
		payload.push_back(*(o + 3));
		payload.push_back(*t);
		payload.push_back(*(t + 1));
		payload.push_back(*(t + 2));
		payload.push_back(*(t + 3));
		p.setPayload(payload);
		WsgPacket r(0);
		r = sendAndReceive(p);
		checkRxPacket(p, r);
		handlePending(p);
	}
	catch (std::exception& e)
	{
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ros::shutdown();
		return;
	}
	cmd_successful = true;
}

void timeout_callback(const boost::system::error_code& error)
{
	if (error == boost::asio::error::operation_aborted)
	{
		ROS_INFO_STREAM("Cancelled Timout, Error code " << error);
	}
	if (error)
	{
		ROS_INFO_STREAM("Timeout error code " << error);
		//ROS_FATAL("Error in connection to driver");
		//throw boost::system::system_error(error);
	}
	else
	{
		ROS_ERROR("Timeout for connection to driver");
		throw std::runtime_error("Connection to gripper timed out");
	}
}
void WsgGripper::connected_callback(const boost::system::error_code& error,
		boost::asio::deadline_timer* timer)
{
	timer->expires_from_now(boost::posix_time::pos_infin);
	if (error)
	{
		ROS_FATAL("Error in connection to driver");
		throw boost::system::system_error(error);
	}
	else
	{
		ROS_INFO("Established connection to driver");
		connected_to_gripper = true;
	}
}
void WsgGripper::openConnection()
{
	connected_to_gripper = false;
	io_service = new boost::asio::io_service;
	socket = new boost::asio::ip::tcp::socket(*io_service);
	// create endpoint with ip addr and port
	ROS_DEBUG_STREAM_NAMED("net", "using address " << ip_addr << ", port " << tcp_port);
	boost::asio::ip::tcp::endpoint e(boost::asio::ip::address::from_string(ip_addr), tcp_port);
	ROS_DEBUG_STREAM_NAMED("net", "creating tcp socket...");
	ROS_DEBUG_STREAM_NAMED("net", "done...");
	//boost::system::error_code error = boost::asio::error::host_not_found;
	// Construct a timer without setting an expiry time.
	boost::asio::deadline_timer* timer = new boost::asio::deadline_timer(*io_service);
	// Set an expiry time relative to now.
	timer->expires_from_now(boost::posix_time::seconds(connection_timeout_sec));
	timer->async_wait(timeout_callback);
	//timer->async_wait(boost::bind(&WsgGripper::connected_callback, this, _1));

	ROS_INFO("Waiting for connection to be established");
	ROS_DEBUG_STREAM_NAMED("net", "connect socket...");
	socket->async_connect(e, boost::bind(&WsgGripper::connected_callback, this, _1, timer));
	io_service->run();
}

WsgPacket WsgGripper::sendAndReceive(WsgPacket p)
{
	flushSocket();
	ROS_DEBUG_STREAM_NAMED("net", "Sending packet: ");
	ROS_DEBUG_STREAM_NAMED("net", std::endl << p.print());
	ROS_DEBUG_STREAM_NAMED("net", "sending packet with " << (int) p.getRaw().size() << " bytes");
	bool retry = false;
	size_t sent = 0;
	try
	{
		sent = socket->send(boost::asio::buffer(p.getRaw()));
	}
	catch (std::exception& e)
	{
		retry = true;
		ROS_FATAL_STREAM("Error: ");
		ROS_FATAL_STREAM(e.what());
		ROS_INFO("Trying to reopen connection...");
	}
	if (retry)
	{ // this time, don't catch exceptions
		openConnection();
		ROS_INFO("Successful, trying to send data...");
		sent = socket->send(boost::asio::buffer(p.getRaw()));
		ROS_INFO("done.");
	}
	ROS_DEBUG_STREAM_NAMED("net", "done, sent " << sent << " bytes...");
	readWithTimeout(RX_TIMEOUT);
	return processTcpPacket(rx_buf, rx_packet_size);
}

WsgPacket WsgGripper::processTcpPacket(boost::array<unsigned char, MAX_PACKET_SIZE> buf
		,
		size_t len)
{

	ROS_DEBUG_NAMED("net", "Parsing %d bytes: ", (int )len);
	std::stringstream s;
	for (uint32_t i = 0; i < len; ++i)
	{
		s << std::hex << "0x" << (unsigned int) buf.at(i) << " ";
	}
	ROS_DEBUG_STREAM_NAMED("net", s.str());
	// convert the received data to a vector
	std::vector<uint8_t> raw_data;
	raw_data.resize(len);
	for (uint32_t i = 0; i < len; ++i)
	{
		raw_data.at(i) = (uint8_t) buf.at(i);
	}
	WsgPacket r(0);
	r.setRaw(raw_data);
	ROS_DEBUG_STREAM("Received packet: ");
	ROS_DEBUG_STREAM(std::endl << r.print());
	return r;
}

void WsgGripper::checkError(uint16_t code)
{
	std::stringstream msg;
	cmd_pending = false;
	last_error = code;
	if (!(code == E_CMD_PENDING || code == E_SUCCESS))
	{
		msg << "Error received, code " << code << " (" << resolveErrorCode(code) << ")";
		throw gripperError(msg.str(), code);
	}
	else
	{
		if (code == E_CMD_PENDING)
		{
			cmd_pending = true;
		}
		ROS_DEBUG_STREAM("Gripper returned message: " << code << " (" << resolveErrorCode(code) << ")");
	}
}

const char* WsgGripper::resolveErrorCode(uint16_t code)
{
	std::stringstream msg;
	if (code > (uint16_t) e_error(E_FILE_EXISTS))
	{
		msg << "Request to resolve non-existing error code: " << code;
		throw std::runtime_error(msg.str());
	}
	return error_messages[code];
}

const char* WsgGripper::resolveStatusCode(uint32_t code)
{
	std::stringstream msg;
	if (code > (uint16_t) e_system_state(SF_SCRIPT_FAILURE))
	{
		msg << "Request to resolve non-existing status code: " << code;
		throw std::runtime_error(msg.str());
	}
	return sf_messages[code];
}

void WsgGripper::handlePending(WsgPacket& p)
{
	if (cmd_pending)
	{
		ROS_INFO("Command pending, waiting for completion...");

		while (cmd_pending)
		{
			try
			{
				readWithTimeout(RX_TIMEOUT);
			}
			catch (std::exception& e)
			{
				ROS_FATAL_STREAM("Error while waiting for command completion: " << e.what());
				ros::shutdown();
				cmd_pending = false;
				return;
			}
			rx_packet = processTcpPacket(rx_buf, rx_packet_size);
			checkRxPacket(p, rx_packet);
		}
		ROS_INFO_STREAM(
				"Command execution finished, gripper returned " << rx_packet.getErrorCode() << " (" << resolveErrorCode(rx_packet.getErrorCode()) << ")");
	}
}

void WsgGripper::readWithTimeout(uint32_t timeout)
{
	uint32_t w = 0;
	uint32_t inc = POLLING_PERIOD;
	boost::array<unsigned char, 1> rec_byte;
	size_t len = 0;
	parser.reset();

	while (1)
	{
		while (socket->available())
		{
			// get next byte
			len = boost::asio::read(*socket, boost::asio::buffer(rec_byte));
			ROS_ASSERT_MSG(len == 1, "error while reading one byte from tcp stream.");
			// trigger state machine
			parser.onByteReceived(rec_byte.at(0));
			// packet received or buffer overflow?
			if (parser.state == WsgParser::PacketReceived || parser.state == WsgParser::Error)
			{
				break;
			}
		}
		// packet received?
		if (parser.state == WsgParser::PacketReceived)
		{
			rx_packet_size = parser.byte_count;
			// copy packet data
			for (uint32_t i = 0; i < rx_packet_size; ++i)
			{
				rx_buf.at(i) = parser.buf.at(i);
			}
			// read successful
			ROS_DEBUG_NAMED("net", "Read %lu bytes.", rx_packet_size);
			break;
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(inc));
		w += inc;
		if (w > timeout)
		{
			throw std::runtime_error("Timeout while waiting for wsg packet!");
		}
	}
}

void WsgGripper::flushSocket()
{
	//const size_t a = socket->available();
	boost::array<uint8_t, MAX_PACKET_SIZE> ar;
	boost::system::error_code error = boost::asio::error::not_connected;
	while (socket->available())
	{
		socket->read_some(boost::asio::buffer(ar), error);
	}
}

float WsgPacket::getAcceleration()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_ACCELERATION)
	{
		throw std::runtime_error(std::string("No acceleration available for this packet!"));
	}
	uint32_t raw = (payload.at(2) + (payload.at(3) << 8) + (payload.at(4) << 16)
			+ (payload.at(5) << 24));
	uint32_t *pRaw = &raw;
	return *(reinterpret_cast<float *>(pRaw));
}

float WsgPacket::getGraspingForceLimit()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_GRASPING_FORCE_LIMIT)
	{
		throw std::runtime_error(std::string("No force limit available for this packet!"));
	}
	uint32_t raw = (payload.at(2) + (payload.at(3) << 8) + (payload.at(4) << 16)
			+ (payload.at(5) << 24));
	uint32_t *pRaw = &raw;
	return *(reinterpret_cast<float *>(pRaw));
}

uint32_t WsgPacket::getSystemState()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_SYSTEM_STATE)
	{
		throw std::runtime_error(std::string("No system state available for this packet!"));
	}
	return (payload.at(2) + (payload.at(3) << 8) + (payload.at(4) << 16) + (payload.at(5) << 24));
}

float WsgPacket::getOpeningWidth()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_OPENING_WIDTH)
	{
		throw std::runtime_error(std::string("No opening width available for this packet!"));
	}
	uint32_t raw = (payload.at(2) + (payload.at(3) << 8) + (payload.at(4) << 16)
			+ (payload.at(5) << 24));
	uint32_t *pRaw = &raw;
	return *(reinterpret_cast<float *>(pRaw));
}

uint8_t WsgPacket::getFingerFlags()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_FINGER_FLAGS)
	{
		throw std::runtime_error(std::string("No finger flags available for this packet!"));
	}
	return payload.at(2);
}

uint8_t WsgPacket::getFingerType()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_FINGER_INFO)
	{
		throw std::runtime_error(std::string("No finger type available for this packet!"));
	}
	return payload.at(2);
}
uint8_t WsgPacket::getFingerDataSize()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_FINGER_INFO)
	{
		throw std::runtime_error(std::string("No finger data size available for this packet!"));
	}
	return payload.at(3) + (payload.at(4) << 8);
}

std::vector<uint8_t> WsgPacket::getFingerData()
{
	if (id != wsg_gripper::GripperCommandGoal::GET_FINGER_DATA)
	{
		throw std::runtime_error(std::string("No finger data available for this packet!"));
	}
	return payload;
}

uint16_t WsgPacket::getErrorCode()
{
	if (payload.size() < 2)
	{
		throw std::runtime_error(std::string("No error code available!"));
	}
	return payload.at(0) + (payload.at(1) << 8);
}

void WsgPacket::setId(uint8_t new_id)
{
	id = new_id;
	if (id == wsg_gripper::GripperCommandGoal::GET_SYSTEM_STATE)
	{
		size = 0;
		payload.clear();
	}
	generatePacketData();
}
void WsgPacket::setPayload(std::vector<uint8_t>& new_payload)
{
	payload = new_payload;
	size = payload.size();
	generatePacketData();
}

void WsgPacket::setRaw(std::vector<uint8_t>& new_raw_data)
{
	raw_data = new_raw_data;
	parseRawData();
}

std::string WsgPacket::print()
{
	std::stringstream s;
	s << "Packet id: " << std::hex << "0x" << (int) id << std::endl;
	s << "Payload size: " << "0x" << (int) size << std::endl;
	s << "Payload: ";
	if (payload.size())
	{
		for (uint32_t i = 0; i < payload.size(); ++i)
		{
			s << "0x" << (int) payload.at(i) << " ";
		}
	}
	else
	{
		s << " (empty)";
	}
	s << std::endl;
	s << "checksum: " << "0x" << (int) checksum << std::endl;
	return s.str();
}

void WsgPacket::generatePacketData()
{
	raw_data.clear();
	// preamble
	raw_data.push_back(0xAA);
	raw_data.push_back(0xAA);
	raw_data.push_back(0xAA);

	raw_data.push_back(id);
	raw_data.push_back(size);
	raw_data.push_back((size >> 8));
	raw_data.insert(raw_data.end(), payload.begin(), payload.end());

	// checksum
	generateChecksum(raw_data);
	raw_data.push_back(checksum);
	raw_data.push_back((checksum >> 8));
}

void WsgPacket::generateChecksum(std::vector<uint8_t>& data)
{
	checksum = 0xFFFF;
	for (uint32_t i = 0; i < data.size(); i++)
	{
		checksum = CRC_TABLE_CCITT16[(checksum ^ data.at(i)) & 0x00FF] ^ (checksum >> 8);
	}
}

void WsgPacket::parseRawData()
{
	std::stringstream msg;
	// check minimal size
	if ((uint32_t) raw_data.size() < (uint32_t) 7)
		throw std::runtime_error(std::string("Parse error: invalid packet contents (size)!"));
	int p_ind = 0;
	// check preamble
	for (int i = 0; i < 3; i++)
	{
		if (raw_data.at(p_ind++) != 0xAA)
			throw std::runtime_error(std::string("Parse error: invalid packet contents (preamble)!"));
	}
	id = raw_data.at(p_ind++);
	uint8_t size_LSB = raw_data.at(p_ind++);
	uint8_t size_MSB = raw_data.at(p_ind++);
	size = size_LSB + (((uint16_t) size_MSB) << 8);
	// payload: all the data to the end minus 2 (checksum has two bytes)
	uint16_t s = raw_data.size() - 2 - p_ind;
	ROS_DEBUG("expected payload size: %d, measured payload size: %d", size, s);
	if (size != s)
	{
		msg << "Parse error: wrong packet size!" << std::endl << "Expected payload size: "
				<< (uint16_t) size << std::endl << "Measured payload size: " << (uint16_t) s << std::endl;
		throw std::runtime_error(msg.str());
	}
	payload.resize(size);
	for (uint16_t i = 0; i < size; ++i)
	{
		payload.at(i) = raw_data.at(p_ind++);
	}
	if (!validateChecksum())
	{
		throw std::runtime_error(std::string("Parse error: invalid checksum!"));
	}
}
// TODO: validate chksum
bool WsgPacket::validateChecksum()
{
	return true;
}

void WsgParser::onByteReceived(uint8_t byte)
{
	ROS_DEBUG_NAMED("tcp_parser", "byte received: %d.", byte);
	if (state == Error)
	{
		// do nothing
		ROS_DEBUG_NAMED("tcp_parser", "byte rejected due to error condition.");
		return;
	}
	buf.at(byte_count) = byte;
	byte_count++;
	switch (static_cast<int>(state))
	{
		case static_cast<int>(WaitingForPreamble):
			if (byte == 0xAA)
			{
				if (byte_count == 3)
				{
					// preamble received
					ROS_DEBUG_NAMED("tcp_parser", "New state: Waiting for id.");
					state = WaitingForId;
				}
				break;
			}
			// unexpected byte
			state = Error;
			break;
		case static_cast<int>(WaitingForId):
			// don't filter this...
			ROS_DEBUG_NAMED("tcp_parser", "New state: Waiting for packet size.");
			state = WaitingForPacketSize;
			break;
		case static_cast<int>(WaitingForPacketSize):
			if (byte_count == 5)
			{
				size_LSB = byte;
			}
			else
			{
				size_MSB = byte;
				packet_size = size_LSB + (((uint16_t) size_MSB) << 8);
				// check if packet_size is ok
				if (packet_size > MAX_PACKET_SIZE)
				{
					state = Error;
					ROS_DEBUG_NAMED("tcp_parser", "Invalid packet size: %d.", packet_size);
				}
				if (packet_size)
				{
					ROS_DEBUG_NAMED("tcp_parser", "New state: Waiting for packet size: %d bytes.", packet_size);
					state = WaitingForData;
				}
				else
				{
					ROS_DEBUG_NAMED("tcp_parser", "New state: Waiting for checksum, skipped waiting for data.");
					state = WaitingForChecksum;
				}
			}
			break;
		case static_cast<int>(WaitingForData):
			// packet size reached?
			if (byte_count == (uint32_t) (packet_size + (uint16_t) 6))
			{
				ROS_DEBUG_NAMED("tcp_parser", "New state: Waiting for checksum.");
				state = WaitingForChecksum;
			}
			break;
		case static_cast<int>(WaitingForChecksum):
			if (byte_count == (uint32_t) (packet_size + (uint16_t) 8))
			{
				ROS_DEBUG_NAMED("tcp_parser", "New state: packet received.");
				state = PacketReceived;
			}
			break;
		case static_cast<int>(PacketReceived):
			ROS_DEBUG_NAMED("tcp_parser", "error: byte received after packet was received.");
			state = Error;
			break;
	}
}

void WsgParser::reset()
{
	ROS_DEBUG_NAMED("tcp_parser", "Parser resetted.");
	ROS_DEBUG_NAMED("tcp_parser", "New state: Waiting for preamble.");
	state = WaitingForPreamble;
	byte_count = 0;
	size_LSB = 0;
	size_MSB = 0;
	packet_size = 0;
}
