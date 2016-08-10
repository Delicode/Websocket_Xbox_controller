///General Header files///
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <map>
#include <new>
#include <math.h>

///XBox 360 controller headers///
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Xinput.h>
#pragma comment(lib, "XInput.lib") // Library containing necessary 360

///Websocket++ Header files///
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

///Boost Header files///
#include <boost/thread/thread.hpp>

///JSON Header files///
#include "src\json.hpp"
using json = nlohmann::json;

///Define///
#define MAX_RETRIES 5				//Nr of time the program will try to reconnect to the websocket server before closing
#define WAIT_TIME 8					//Define wait time for limiting the number of messages sent per second
#define TIME_BETWEEN_RECONNECT 5	//Defined the time in seconds we wait between each reconnect try

///Websocket++ typedef///
typedef websocketpp::client<websocketpp::config::asio_client> client;

///Websocket++ Variable///
bool disconnected = true;			//If we are disconnected from server
int retries = 0;					//If connection to server is lost we save the number of retries, exit program after MAX_RETRIES
bool start = false;					//Variable that is checked before we start to send data
bool stop = false;					//Variable to stop the program
bool pause = false;					//Variable to pause the data being sent / the program
int intensity_param = 0;			//The parameter of the motor speed, used for when the "start_motor" function is called

///Xbox Controller Variables///
float old_val_LT = 0;
float old_val_RT = 0;
float old_val_LStickX = 0;
float old_val_LStickY = 0;
float old_val_RStickX = 0;
float old_val_RStickY = 0;

///Time varibles///
boost::posix_time::ptime time_start;		//When to start counting towards the wait time
boost::posix_time::ptime time_end;			//Used to compare/determine how long time has passed
boost::posix_time::time_duration diff;		//The time difference between time_start and time_end

class connection_metadata {

public:
	typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

	connection_metadata(int id, websocketpp::connection_hdl hdl, std::string uri)
		: m_id(id)
		, m_hdl(hdl)
		, m_status("Connecting")
		, m_uri(uri)
		, m_server("N/A")
	{}

	void on_open(client * c, websocketpp::connection_hdl hdl) {
		m_status = "Open";

		client::connection_ptr con = c->get_con_from_hdl(hdl);
		m_server = con->get_response_header("Server");
	}

	void on_fail(client * c, websocketpp::connection_hdl hdl) {
		m_status = "Failed";

		client::connection_ptr con = c->get_con_from_hdl(hdl);
		m_server = con->get_response_header("Server");
		m_error_reason = con->get_ec().message();
	}

	void on_close(client * c, websocketpp::connection_hdl hdl) {
		m_status = "Closed";
		client::connection_ptr con = c->get_con_from_hdl(hdl);
		std::stringstream s;
		s << "close code: " << con->get_remote_close_code() << " ("
			<< websocketpp::close::status::get_string(con->get_remote_close_code())
			<< "), close reason: " << con->get_remote_close_reason();
		m_error_reason = s.str();
	}

	void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
		//std::cout << msg->get_payload().c_str() << std::endl;						//In case we need to debug the messages we receive
		if (msg->get_opcode() == websocketpp::frame::opcode::text) {

			std::string incoming_msg = msg->get_payload(); 							//Convert to string
			json json_msg = json::parse(incoming_msg);								//Parse to Json

			//Check which function is sent in the Json message
			//Extract what is needed, the speed of the controller
			//Run the function
			if (start) {
				if (json_msg.find("type") != json_msg.end()) {						//Check Json message if it contains the type of message received
					std::string type_str = json_msg["type"].get<std::string>();		//Get the type of message
					if (type_str == "parameters") {									//Check to see if we received a parameters message
						if (json_msg.find("value") != json_msg.end()) {
							std::string temp_str = incoming_msg;
							std::string temp_str2;
							int found = temp_str.find("\"value\":");				//Find where the parameter values start
							int found2 = temp_str.find('}');						//Find where the parameter values end
							if (found > 0) {
								temp_str2 = temp_str.substr(found + 8, found2 - found - 7);
								json json_tmp = json::parse(temp_str2);				//Parse our values to Json for easier access
								if (json_tmp.find("intensity") != json_tmp.end()) {	//If we find the "Speed" parameter
									intensity_param = json_tmp["intensity"].get<int>();		//Get the speed and save it to the intensity_param variable
									//std::cout << intensity_param << std::endl;			//For debugging the speed that is received
								}
							}
						}
					}
					else if (type_str == "command") {						//Check to see if we received a command message
						if (json_msg.find("value") != json_msg.end()) {		//Find the value of the command message
							std::string cmd_str = json_msg["value"].get<std::string>();	//Save the command
							if (cmd_str == "rumble") {						//Check if the command was our "start_motor" command
							//rumble(intensity_param);						//Run the function that starts the rumble motor at a certain speed, according to our intensity parameter
							}
						}
					}
					else if (type_str == "stop") { 							//Stop message from the server which will be sent to the client when the server wants to pause the program
						pause = !pause;
					}
					else if (type_str == "quit") {							//Quit message from the server which will be sent to the client when the server wants to stop the program
						stop = true;
					}
				}
			}
			if (json_msg.find("type") != json_msg.end()) {
				std::string start_cmd = json_msg["type"].get<std::string>();
				if (start_cmd == "start") {									//Start message from the server which will be sent to the client when the server is ready to receive data
					start = true;
				}
			}
			incoming_msg.clear();
			json_msg.clear();
		}
		else {
			//m_messages.push_back("<< " + websocketpp::utility::to_hex(msg->get_payload())); //If the message is not a string or similar, we only send JSON/Strings
		}
	}

	websocketpp::connection_hdl get_hdl() const {
		return m_hdl;
	}

	int get_id() const {
		return m_id;
	}

	std::string get_status() const {
		return m_status;
	}

	void record_sent_message(std::string message) {
		//m_messages.push_back(">> " + message);	//We don't need to record messages, use if you want
	}

	friend std::ostream & operator<< (std::ostream & out, connection_metadata const & data);

private:
	int m_id;
	websocketpp::connection_hdl m_hdl;
	std::string m_status;
	std::string m_uri;
	std::string m_server;
	std::string m_error_reason;
	std::vector<std::string> m_messages;
};

std::ostream & operator<< (std::ostream & out, connection_metadata const & data) {
	out << "> URI: " << data.m_uri << "\n"
		<< "> Status: " << data.m_status << "\n"
		<< "> Remote Server: " << (data.m_server.empty() ? "None Specified" : data.m_server) << "\n"
		<< "> Error/close reason: " << (data.m_error_reason.empty() ? "N/A" : data.m_error_reason) << "\n";
	out << "> Messages Processed: (" << data.m_messages.size() << ") \n";

	std::vector<std::string>::const_iterator it;
	for (it = data.m_messages.begin(); it != data.m_messages.end(); ++it) {
		out << *it << "\n";
	}

	return out;
}

class websocket_endpoint {

public:
	websocket_endpoint() : m_next_id(0) {
		m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
		m_endpoint.clear_error_channels(websocketpp::log::elevel::all);
		m_endpoint.init_asio();
		m_endpoint.start_perpetual();
		m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);
		//m_thread.reset(new websocketpp::lib::thread(&client::run, &m_endpoint));
	}

	~websocket_endpoint() {
		m_endpoint.stop_perpetual();

		for (con_list::const_iterator it = m_connection_list.begin(); it != m_connection_list.end(); ++it) {
			if (it->second->get_status() != "Open") {
				// Only close open connections
				continue;
			}
			std::cout << "> Closing connection " << it->second->get_id() << std::endl;
			websocketpp::lib::error_code ec;
			m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
			if (ec) {
				std::cout << "> Error closing connection " << it->second->get_id() << ": "
					<< ec.message() << std::endl;
			}
		}
		m_thread->join();
	}

	int connect(std::string const & uri) {
		websocketpp::lib::error_code ec;

		client::connection_ptr con = m_endpoint.get_connection(uri, ec);
		if (ec) {
			std::cout << "> Connect initialization error: " << ec.message() << std::endl;
			return -1;
		}
		int new_id = m_next_id++;
		connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(new_id, con->get_handle(), uri);
		m_connection_list[new_id] = metadata_ptr;

		con->set_open_handler(websocketpp::lib::bind(
			&connection_metadata::on_open,
			metadata_ptr,
			&m_endpoint,
			websocketpp::lib::placeholders::_1
		));
		con->set_fail_handler(websocketpp::lib::bind(
			&connection_metadata::on_fail,
			metadata_ptr,
			&m_endpoint,
			websocketpp::lib::placeholders::_1
		));
		con->set_close_handler(websocketpp::lib::bind(
			&connection_metadata::on_close,
			metadata_ptr,
			&m_endpoint,
			websocketpp::lib::placeholders::_1
		));
		con->set_message_handler(websocketpp::lib::bind(
			&connection_metadata::on_message,
			metadata_ptr,
			websocketpp::lib::placeholders::_1,
			websocketpp::lib::placeholders::_2
		));

		m_endpoint.connect(con);

		return new_id;
	}

	void send(int id, std::string message) {
		websocketpp::lib::error_code ec;
		con_list::iterator metadata_it = m_connection_list.find(id);

		if (metadata_it == m_connection_list.end()) {
			std::cout << "> No connection found with id " << id << std::endl;
			return;
		}
		m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);

		if (ec) {
			std::cout << "> Error sending message: " << ec.message() << std::endl;
			std::cout << "> Trying again after " << TIME_BETWEEN_RECONNECT << " seconds \n";
			boost::this_thread::sleep(boost::posix_time::seconds(TIME_BETWEEN_RECONNECT)); //If a send fails, the server has most likely crashed, wait TIME_BETWEEN_RECONNECT seconds if this happens then try to reconnect
			retries++;		//Keep track of the number of retries
			return;
		}
		retries = 0;		//If we send our message successfully we set our nr of retries back to 0
	}

	void close(int id, websocketpp::close::status::value code, std::string reason) {
		websocketpp::lib::error_code ec;

		con_list::iterator metadata_it = m_connection_list.find(id);
		if (metadata_it == m_connection_list.end()) {
			std::cout << "> No connection found with id " << id << std::endl;
			return;
		}

		m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
		if (ec) {
			std::cout << "> Error initiating close: " << ec.message() << std::endl;
		}
	}

	connection_metadata::ptr get_metadata(int id) const {
		con_list::const_iterator metadata_it = m_connection_list.find(id);
		if (metadata_it == m_connection_list.end()) {
			return connection_metadata::ptr();
		}
		else {
			return metadata_it->second;
		}
	}

private:
	typedef std::map<int, connection_metadata::ptr> con_list;

	client m_endpoint;
	websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

	con_list m_connection_list;
	int m_next_id;
};

class Gamepad {
private:
	int cId;
	XINPUT_STATE state;

	float deadzoneX;
	float deadzoneY;

public:
	Gamepad() : deadzoneX(0.05f), deadzoneY(0.02f) {}
	Gamepad(float dzX, float dzY) : deadzoneX(dzX), deadzoneY(dzY) {}

	float leftStickX;
	float leftStickY;
	float rightStickX;
	float rightStickY;
	float leftTrigger;
	float rightTrigger;

	int  GetPort();
	XINPUT_GAMEPAD *GetState();
	bool CheckConnection();
	bool Refresh();
	bool IsPressed(WORD);
};

int Gamepad::GetPort() {
	return cId + 1;
}

XINPUT_GAMEPAD *Gamepad::GetState() {
	return &state.Gamepad;
}

bool Gamepad::CheckConnection() {
	int controllerId = -1;

	for (DWORD i = 0; i < XUSER_MAX_COUNT && controllerId == -1; i++) {
		XINPUT_STATE state;
		ZeroMemory(&state, sizeof(XINPUT_STATE));

		if (XInputGetState(i, &state) == ERROR_SUCCESS) {
			controllerId = i;
		}

		cId = controllerId;

		return controllerId != -1;
	}
}

// Returns false if the controller has been disconnected
bool Gamepad::Refresh() {
	if (cId == -1) {
		CheckConnection();
	}

	if (cId != -1) {
		ZeroMemory(&state, sizeof(XINPUT_STATE));
		if (XInputGetState(cId, &state) != ERROR_SUCCESS) {
			cId = -1;
			return false;
		}

		float normLX = fmaxf(-1, (float)state.Gamepad.sThumbLX / 32767);
		float normLY = fmaxf(-1, (float)state.Gamepad.sThumbLY / 32767);

		leftStickX = (abs(normLX) < deadzoneX ? 0 : (abs(normLX) - deadzoneX) * (normLX / abs(normLX)));
		leftStickY = (abs(normLY) < deadzoneY ? 0 : (abs(normLY) - deadzoneY) * (normLY / abs(normLY)));

		if (deadzoneX > 0) leftStickX *= 1 / (1 - deadzoneX);
		if (deadzoneY > 0) leftStickY *= 1 / (1 - deadzoneY);

		float normRX = fmaxf(-1, (float)state.Gamepad.sThumbRX / 32767);
		float normRY = fmaxf(-1, (float)state.Gamepad.sThumbRY / 32767);

		rightStickX = (abs(normRX) < deadzoneX ? 0 : (abs(normRX) - deadzoneX) * (normRX / abs(normRX)));
		rightStickY = (abs(normRY) < deadzoneY ? 0 : (abs(normRY) - deadzoneY) * (normRY / abs(normRY)));

		if (deadzoneX > 0) rightStickX *= 1 / (1 - deadzoneX);
		if (deadzoneY > 0) rightStickY *= 1 / (1 - deadzoneY);

		leftTrigger = (float)state.Gamepad.bLeftTrigger / 255;
		rightTrigger = (float)state.Gamepad.bRightTrigger / 255;

		return true;
	}
	return false;
}

bool Gamepad::IsPressed(WORD button) {
	return (state.Gamepad.wButtons & button) != 0;
}


int main(int argc, char *argv[]) {

	///Controller Setup///
	Gamepad gamepad;
	bool wasConnected = true;

	int poll_data = -1;
	char answer;
	while (poll_data < 0) {												//Loop untill we get a valid ip address and port number
		std::cout << "Do you want to poll the controller continuously or just when buttons are pressed? 'y' to poll and 'n' not to poll" << std::endl;
		std::cin >> answer;
		if (answer == 'y') poll_data = 1;
		else if (answer == 'n') poll_data = 0;
		else std::cout << "Invalid entry" << std::endl;
	}

	///Websocket++ Setup///
	std::string ip_address;
	int id = -1;
	websocket_endpoint endpoint;

	while (id < 0) {												//Loop untill we get a valid ip address and port number
		std::cout << "Enter IP address and port, example: ws://192.168.1.1:7651 \n";
		std::cin >> ip_address;
		id = endpoint.connect(ip_address);
	}
	boost::this_thread::sleep(boost::posix_time::seconds(2));	//Wait for connection

	///Initial JSON message template that will be sent to the Server///
	std::string device_id = "xbox_controller";

	char Json[] = R"({
		"type": "device",
		"value": {
			"device_id": "",
			"device_type": "controller",
			"name": "xbox 360",
			"values": [ {
				"name": "controller",
				"type": "array",
				"datatype": "string",
				"count": 1,
				"ID": []
			},
			{
				"name": "right stick",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			},
			{
				"name": "left stick",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			},
			{
				"name": "left trigger",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			},
			{
				"name": "right trigger",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			},
			{
				"name": "buttons",
				"type": "vec",
				"datatype": "bool",
				"count": 10,
				"flags": "per_user"
			},
			{
				"name": "dpad",
				"type": "vec",
				"datatype": "int",
				"count": 8,
				"flags": "per_user"
			}
			],
			"parameters": [ 
			{
				"name": "intensity",
				"type": "vec",
				"datatype": "int",
				"default": 0,
				"min": 0,
				"max": 100,
				"count": 1
			}
			],
			"commands": [ {
				"name": "rumble",
				"datatype": "bool"
			}
			]
		}
})";

	//Start message that is sent if we do not want to constantly poll the controller for changes

	char Json2[] = R"({
		"type": "device",
		"value": {
			"device_id": "",
			"device_type": "controller",
			"name": "xbox 360",
			"values": [ {
				"name": "controller",
				"type": "array",
				"datatype": "string",
				"count": 1,
				"ID": []
			},
			{
				"name": "right stick",
				"type": "vec",
				"vec_dimension": 2,
				"datatype": "int",
				"count": 1,
				"min": -65534,
				"max": 65534,
				"flags": "per_user"
			},
			{
				"name": "left stick",
				"type": "vec",
				"vec_dimension": 2,
				"datatype": "int",
				"count": 1,
				"min": -65534,
				"max": 65534,
				"flags": "per_user"
			},
			{
				"name": "left trigger",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			},
			{
				"name": "right trigger",
				"type": "vec",
				"datatype": "int",
				"count": 1,
				"min": 0,
				"max": 255,
				"flags": "per_user"
			},
			{
				"name": "button",
				"type": "vec",
				"datatype": "string",
				"count": 1,
				"flags": "per_user"
			}
			],
			"parameters": [ 
			{
				"name": "intensity",
				"type": "vec",
				"datatype": "int",
				"default": 0,
				"min": 0,
				"max": 100,
				"count": 1
			}
			],
			"commands": [ {
				"name": "rumble",
				"datatype": "bool"
			}
			]
		}

})";
	
	if (poll_data == 1) {
		std::string init_msg(Json);					//Create string from the above char message
		//std::cout << init_msg << std::endl;		//In case of bug in our initial message
		init_msg.insert(52, device_id);				//Insert the MAC address of the device into the initial message
		json j_complete = json::parse(init_msg);	//Parse our message to Json
		std::string start_message = j_complete.dump(); //Dump our Json code to a string which will be sent to the server / NI Mate
		endpoint.send(id, start_message);			//Send the initial message
	}
	else {
		std::string init_msg(Json2);					//Create string from the above char message
		//std::cout << init_msg << std::endl;		//In case of bug in our initial message
		init_msg.insert(52, device_id);				//Insert the MAC address of the device into the initial message
		json j_complete = json::parse(init_msg);	//Parse our message to Json
		std::string start_message = j_complete.dump(); //Dump our Json code to a string which will be sent to the server / NI Mate
		endpoint.send(id, start_message);			//Send the initial message
	}
	/*
	while (!start) {							//Wait for server to send start command
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	*/
	

	while (retries < MAX_RETRIES) {										//Close after MAX_RETRIES
		//boost::this_thread::sleep(boost::posix_time::seconds(1));			//Sleep for 1 second

		unsigned int *Button_Data;				//Variable for the button value
		char *Button_array;						//Array of buttons
		char *DPad_array;						//Array of dpad

		if (!gamepad.Refresh()) {
			if (wasConnected) {
				wasConnected = false;

				std::cout << "Please connect an Xbox 360 controller." << std::endl;
			}
		}

		else {
			if (!wasConnected) {
				wasConnected = true;
				std::cout << "Controller connected on port " << gamepad.GetPort() << std::endl;
			}

			if (poll_data == 1) {


				time_start = boost::posix_time::microsec_clock::local_time();
				time_end = boost::posix_time::microsec_clock::local_time();
				diff = time_end - time_start;

				while (diff.total_milliseconds() < WAIT_TIME) {			//Limit the data sending rate
					boost::this_thread::sleep(boost::posix_time::milliseconds(10));	//Wait for connection
					time_end = boost::posix_time::microsec_clock::local_time();
					diff = time_end - time_start;
				}

				Button_array = new char[10];
				DPad_array = new char[4];

				if (gamepad.IsPressed(XINPUT_GAMEPAD_START)) Button_array[0] = '1';
				else Button_array[0] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_BACK)) Button_array[1] = '1';
				else Button_array[1] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_LEFT_THUMB)) Button_array[2] = '1';
				else Button_array[2] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_RIGHT_THUMB)) Button_array[3] = '1';
				else Button_array[3] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_LEFT_SHOULDER)) Button_array[4] = '1';
				else Button_array[4] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_RIGHT_SHOULDER)) Button_array[5] = '1';
				else Button_array[5] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_A)) Button_array[6] = '1';
				else Button_array[6] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_B)) Button_array[7] = '1';
				else Button_array[7] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_X)) Button_array[8] = '1';
				else Button_array[8] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_Y)) Button_array[9] = '1';
				else Button_array[9] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_UP)) DPad_array[0] = '1';
				else DPad_array[0] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_DOWN)) DPad_array[1] = '1';
				else DPad_array[1] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_LEFT)) DPad_array[2] = '1';
				else DPad_array[2] = '0';

				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_RIGHT)) DPad_array[3] = '1';
				else DPad_array[3] = '0';

				json json_j;
				std::string message;

				json_j["type"] = "data";
				json_j["value"]["device_id"] = device_id;
				json_j["value"]["timestamp_ms"] = (time(0) * 1000);
				json_j["value"]["user_1"]["left stick"] = { gamepad.leftStickX, gamepad.leftStickY };
				json_j["value"]["user_1"]["right stick"] = { gamepad.rightStickX, gamepad.rightStickY };
				json_j["value"]["user_1"]["left trigger"] = gamepad.leftTrigger;
				json_j["value"]["user_1"]["right trigger"] = gamepad.rightTrigger;
				json_j["value"]["user_1"]["buttons"] = { Button_array[0] - '0', Button_array[1] - '0', Button_array[2] - '0', Button_array[3] - '0',
					Button_array[4] - '0', Button_array[5] - '0', Button_array[6] - '0', Button_array[7] - '0', Button_array[8] - '0', Button_array[9] - '0'};
				json_j["value"]["user_1"]["dpad"] = { DPad_array[0], DPad_array[1], DPad_array[2], DPad_array[3] };

				message = json_j.dump();											//Dump the Json message to a string so it can be sent
				endpoint.send(id, message);											//Send the message / data to the server
				std::cout << message << std::endl;									//In case we need to debug the message that is sent
				///Cleanup before next iteration, avoid memory leaks///
				message.clear();
				json_j.clear();

				delete[] Button_array;
				delete[] DPad_array;
			}
			else if (poll_data == 0) {
				json json_j;
				std::string message;
				bool state_changed = false;

				boost::this_thread::sleep(boost::posix_time::milliseconds(500));	//Wait for connection
				json_j["type"] = "data";
				json_j["value"]["device_id"] = device_id;
				json_j["value"]["timestamp_ms"] = (time(0) * 1000);

				if (gamepad.IsPressed(XINPUT_GAMEPAD_START)) json_j["value"]["user_1"]["button"] = "start", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_BACK)) json_j["value"]["user_1"]["button"] = "back", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_LEFT_THUMB)) json_j["value"]["user_1"]["button"] = "ls", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_RIGHT_THUMB)) json_j["value"]["user_1"]["button"] = "rs", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_LEFT_SHOULDER)) json_j["value"]["user_1"]["button"] = "lb", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_RIGHT_SHOULDER)) json_j["value"]["user_1"]["button"] = "rb", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_A)) json_j["value"]["user_1"]["button"] = "a", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_B)) json_j["value"]["user_1"]["button"] = "b", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_X)) json_j["value"]["user_1"]["button"] = "x", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_Y)) json_j["value"]["user_1"]["button"] = "y", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_UP)) json_j["value"]["user_1"]["dpad"] = "up", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_DOWN)) json_j["value"]["user_1"]["dpad"] = "down", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_LEFT)) json_j["value"]["user_1"]["dpad"] = "left", state_changed = true;
				else if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_RIGHT)) json_j["value"]["user_1"]["dpad"] = "right", state_changed = true;


				else if ( (gamepad.leftStickX != old_val_LStickX) || (gamepad.leftStickY != old_val_LStickY) ) {
					old_val_LStickX = gamepad.leftStickX, old_val_LStickY = gamepad.leftStickY;
					json_j["value"]["user_1"]["left stick"] = { gamepad.leftStickX, gamepad.leftStickY };
					state_changed = true;
				}

				else if ( (gamepad.rightStickX != old_val_RStickX) || (gamepad.rightStickY != old_val_RStickY) ) {
					old_val_RStickX = gamepad.rightStickX, old_val_RStickY = gamepad.rightStickY;
					json_j["value"]["user_1"]["left stick"] = { gamepad.rightStickX, gamepad.rightStickY };
					state_changed = true;
				}

				else if (gamepad.leftTrigger != old_val_LT) {
					old_val_LT = gamepad.leftTrigger;
					json_j["value"]["user_1"]["left trigger"] = gamepad.leftTrigger;
					state_changed = true;
				}

				else if (gamepad.rightTrigger != old_val_RT) {
					old_val_LT = gamepad.rightTrigger;
					json_j["value"]["user_1"]["right trigger"] = gamepad.rightTrigger;
					state_changed = true;
				}

				if (state_changed) {
					message = json_j.dump();								//Dump the Json message to a string so it can be sent
					std::cout << message << std::endl;						//Print out message, for debugging
					endpoint.send(id, message);								//Send the message / data to the server
				}
				
				///Cleanup before next iteration, avoid memory leaks///
				message.clear();
				json_j.clear();
			}

		}

		if (retries > 0) {													//Heartbeat message didn't go through if retries is > 0
			int close_code = websocketpp::close::status::service_restart;	//Reason why we are closing connection
			std::string reason = "Trying to re-connect";
			endpoint.close(id, close_code, reason);							//This will fail if server was closed, might be useless
			id = endpoint.connect(ip_address);								//Try to reconnect to the server, id will be old id + 1
			boost::this_thread::sleep(boost::posix_time::seconds(2)); 		//Have to wait a couple of seconds to reconnect
		}

		if (stop) {															//If we receive a stop command from the sever / NI Mate
			break;															//Break from the loop and quit program
		}
	}

	//Websocket Cleanup//
	int close_code = websocketpp::close::status::normal;
	std::string reason = "Quit program";
	endpoint.close(id, close_code, reason);									//Close the connection with the websocket server
	std::cout << "> Program has closed \n";

	return 0;
}