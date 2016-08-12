/*
* Program to use with the Xbox Controller

* The program works in the following way:
- Asks for the IP address and port to NI Mate / websocket server
- Connects to NI Mate / the server
- Waits to receive the start message that NI Mate sends when it starts to use a new device
- According to the poll parameter, the program either starts to send data continuously or then just when a button is pressed / joystick is moved
- If the program receives a stop message it will pause the sending of data and if it receives a quit message it will quit the program

* Things to note:
- Currently only one controller is supported, might add multiple controller support later
- The program starts with the polling set to false so it only sends a message if a button is pressed
*/

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
#pragma comment(lib, "XInput.lib")

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

///Parameter Variables///
int intensity_param = 0;			//The value of the vibration speed, used for the different rumble fuctions
int duration_param = 0;				//The duration of the rumble in the function rumble_timed()
int pattern_param = 0;				//The vibration pattern that we want or the number of vibrations
bool poll_param = false;

///Function declarations///
void rumble();						//Function that starts or stops the rumble motors			
void rumble_timed();				//Function that starts the rumble motors and keep them on for at set time
void rumble_pattern();				//Function that vibrates the controller a nr of times (pattern_param)

///Xbox Controller Variables///
float old_val_LT = 0;				//Old value of left trigger for checking if the state has changed
float old_val_RT = 0;				//Old value of right trigger for checking if the state has changed
float old_val_LStickX = 0;			//Old value of Left stick X axis for checking if the state has changed
float old_val_LStickY = 0;			//Old value of Left stick Y axis for checking if the state has changed
float old_val_RStickX = 0;			//Old value of right stick X axis for checking if the state has changed
float old_val_RStickY = 0;			//Old value of right stick Y axis for checking if the state has changed
int button_array_nopoll[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0}; //Array for buttons
int dpad_array_nopoll[4] = { 0, 0, 0, 0 };				//Array for Dpad
int button_hold[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	//Array for keeping track of which buttons are being held down
int dpad_hold[4] = {0 ,0 , 0, 0};						//Array for keeping track of which dpad buttons are being held down

int rumble_intensity_division[101] = { 0, 655, 1310, 1966, 2621, 3276, 3932, 4587, 5242, 5898, 6553, 7208,
7864, 8519, 9174, 9830, 10485, 11140, 11796, 12451, 13107, 13762, 14417, 15073, 15728, 16383, 17039, 17694,
18349, 19005, 19660, 20315, 20971, 21626, 22281, 22937, 23592, 24247, 24903, 25558, 26214, 26869, 27524, 28180,
28835, 29490, 30146, 30801, 31456, 32112, 32767, 33422, 34078, 34733, 35388, 36044, 36699, 37354, 38010, 38665, 39321,
39976, 40631, 41287, 41942, 42597, 43253, 43908, 44563, 45219, 45874, 46529, 47185, 47840, 48495, 49151, 49806, 50461,
51117, 51772, 52428, 53083, 53738, 54394, 55049, 55704, 56360, 57015, 57670, 58326, 58981, 59636, 60292, 60947, 61602,
62258, 62913, 63568, 64224, 64879, 65535 };

///Time varibles///
boost::posix_time::ptime time_start;		//When to start counting towards the wait time
boost::posix_time::ptime time_end;			//Used to compare/determine how long time has passed
boost::posix_time::time_duration diff;		//The time difference between time_start and time_end

///Debug///
bool debug_send = true;						//Set true to print out everything that is being sent to NI Mate / Websocket server
bool debug_receive = false;					//Set true to print out everything being received from NI Mate / Websocket server

///Classses for the websocket client part of the program///

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

	///When we receive a message it will come here, >message handler///
	void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
		if (debug_receive) std::cout << msg->get_payload().c_str() << std::endl;	//In case we need to debug the messages we receive
		if (msg->get_opcode() == websocketpp::frame::opcode::text) {

			std::string incoming_msg = msg->get_payload(); 							//Convert to string
			json json_msg = json::parse(incoming_msg);								//Parse to Json

			//Check which function is sent in the Json message
			//Extract what is needed, the duration, the pattern and the intensity
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
								if (json_tmp.find("intensity") != json_tmp.end()) {	//If we find the "intensity" parameter
									intensity_param = json_tmp["intensity"].get<int>();		//Get the intensity and save it to the intensity_param variable
									if (debug_receive) std::cout << intensity_param << std::endl;	//For debugging the intensity that is received
								}
								if (json_tmp.find("pattern") != json_tmp.end()) {
									pattern_param = json_tmp["pattern"].get<int>();					//Get the pattern of the rumble and set our pattern parameter
									if (debug_receive) std::cout << pattern_param << std::endl;		//For debugging the pattern that is received
								}
								if (json_tmp.find("duration") != json_tmp.end()) {
									duration_param = json_tmp["duration"].get<int>();				//Get the duration of the rumble and set our duration parameter
									if (debug_receive) std::cout << duration_param << std::endl;	//For debugging the duration that is received
								}
								if (json_tmp.find("poll") != json_tmp.end()) {
									poll_param = json_tmp["poll"].get<bool>();						//Get the duration of the rumble and set our duration parameter
									if (debug_receive) std::cout << duration_param << std::endl;	//For debugging the duration that is received
								}
							}
						}
					}
					else if (type_str == "command") {												//Check to see if we received a command message
						if (json_msg.find("value") != json_msg.end()) {								//Check that there is a value in our command message
							std::string cmd_val = json_msg["value"].get<std::string>();				//Save the command that we have received
							if (cmd_val == "rumble") {												//Rumble command is found, start to rumble the controller
								boost::thread t1(&rumble);											//Start the rumble in new thread so that data is still being sent even though the controller is rumbling
							}
							else if (cmd_val == "rumble_pattern") {									//Rumble pattern is chosen, the chosen rumble pattern in the rumble pattern parameter will be run
								boost::thread t2(&rumble_pattern);									//Start the rumble in new thread so that data is still being sent even though the controller is rumbling
							}
							else if (cmd_val == "rumble_timed") {									//Rumble timed command is found, rumble the motor for at cetain duration
								boost::thread t3(&rumble_timed);									//Start the rumble in new thread so that data is still being sent even though the controller is rumbling
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

///Gamepad class//

class Gamepad {
private:
	int cId;
	XINPUT_STATE state;

	float deadzoneX;
	float deadzoneY;

public:
	Gamepad() : deadzoneX(0.05f), deadzoneY(0.05f) {}
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
	XINPUT_GAMEPAD *SetState(int intensity);
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

XINPUT_GAMEPAD * Gamepad::SetState(int intensity) { //Funtion that vibrates the controller, input is 0-65535

	XINPUT_VIBRATION vibration;
	ZeroMemory(&vibration, sizeof(XINPUT_VIBRATION));
	vibration.wLeftMotorSpeed = intensity;
	vibration.wRightMotorSpeed = intensity;
	XInputSetState(cId, &vibration);
	return &state.Gamepad;
}

///Controller Setup///
Gamepad gamepad;        //Initiate the controller here

void rumble() {			//Function that turns on the rumble motor
	gamepad.SetState( rumble_intensity_division[intensity_param] );
}

void rumble_timed() {	//Function that turns on the motor for a set duration
	gamepad.SetState( rumble_intensity_division[intensity_param] );
	boost::this_thread::sleep(boost::posix_time::seconds(duration_param));
	gamepad.SetState(0);
	return;
}

void rumble_pattern() {	//Function that rumbles the controller according to a set pattern
	if (pattern_param == 1) {
		gamepad.SetState( rumble_intensity_division[intensity_param] );
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));	//Let controller rumble for time
		gamepad.SetState(0);												//Turn rumble off
		return;
	}
	else if (pattern_param > 1) {
		for (int y = 0; y < pattern_param; y++) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			gamepad.SetState( rumble_intensity_division[intensity_param] );	//Set the controller rumble 
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));//Let controller rumble for 500 ms
			gamepad.SetState(0);											//Turn rumble off
		}
		return;
	}
}

int main(int argc, char *argv[]) {

	std::string start_message;		//Initiate start message here so it can be used when we want to reconnect
	bool wasConnected = true;

	///Websocket++ Setup///
	std::string ip_address;
	int id = -1;
	websocket_endpoint endpoint;

	while (id < 0) {				//Loop untill we get a valid ip address and port number
		std::cout << "Enter IP address and port, example: ws://192.168.1.1:7651 \n";
		std::cin >> ip_address;
		id = endpoint.connect(ip_address);
	}
	boost::this_thread::sleep(boost::posix_time::seconds(2));	//Wait for the connection to establish

	///Initial JSON message template that will be sent to the Server///
	std::string device_id = "xbox_controller"; //Can be changed if necessary

	char Json[] = R"({
		"type": "device",
		"value": {
			"device_id": "",
			"device_type": "controller",
			"name": "xbox 360",
			"values": [ {
				"name": "right stick",
				"type": "vec",
				"vec_dimension": 2,
				"datatype": "float",
				"count": 1,
				"min": -1,
				"max": 1,
				"flags": "per_user"
			},
			{
				"name": "left stick",
				"type": "vec",
				"vec_dimension": 2,
				"datatype": "float",
				"count": 1,
				"min": -1,
				"max": 1,
				"flags": "per_user"
			},
			{
				"name": "left trigger",
				"type": "vec",
				"datatype": "float",
				"count": 1,
				"min": 0,
				"max": 1,
				"flags": "per_user"
			},
			{
				"name": "right trigger",
				"type": "vec",
				"datatype": "float",
				"count": 1,
				"min": 0,
				"max": 1,
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
				"count": 4,
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
			},
			{
				"name": "poll",
				"type": "vec",
				"datatype": "bool",
				"default": 0,
				"count": 1
			},
			{
				"name": "duration",
				"type": "vec",
				"datatype": "int",
				"default": 0,
				"min": 0,
				"max": 20,
				"count": 1
			},
			{
				"name": "pattern",
				"type": "vec",
				"datatype": "int",
				"default": 0,
				"min": 0,
				"max": 20,
				"count": 1
			}
			],
			"commands": [ {
				"name": "rumble",
				"datatype": "bool"
			},
			{
				"name": "rumble_timed",
				"datatype": "bool"
			},
			{
				"name": "rumble_pattern",
				"datatype": "bool"
			}
			]
		}
})";

	std::string init_msg(Json);								//Create string from the above char message
	if (debug_send) std::cout << init_msg << std::endl;		//In case of bug in our initial message
	init_msg.insert(52, device_id);							//Insert the device id of the device into the initial message
	json j_complete = json::parse(init_msg);				//Parse our message to Json
	start_message = j_complete.dump();						//Dump our Json code to a string which will be sent to the server / NI Mate
	endpoint.send(id, start_message);						//Send the initial message	
	
	/*while (!start) {											//Wait for server to send start command
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}*/
	
	

	while (retries < MAX_RETRIES) {								//Close after MAX_RETRIES, if server disconnects

		if (!gamepad.Refresh()) {								//If we are unable to refresh the controller
			if (wasConnected) {									//Check if we were connected to a controller
				wasConnected = false;
				std::cout << "Please connect an Xbox 360 controller." << std::endl;
			}
		}

		else {
			if (!wasConnected) {								//We are connected to a controller
				wasConnected = true;
				std::cout << "Controller connected on port " << gamepad.GetPort() << std::endl;
			}

			if (poll_param) {

				char *Button_array;								//Array	 buttons, 1 if being pressed, 0 if not pressed
				char *DPad_array;								//Array of dpad buttons, 1 if being pressed, 0 if not pressed

				time_start = boost::posix_time::microsec_clock::local_time();	//Start the time variable to comare with
				time_end = boost::posix_time::microsec_clock::local_time();		//Our first end value, will be compared to the start time
				diff = time_end - time_start;									//Calculate the difference in time

				while (diff.total_milliseconds() < WAIT_TIME) {	//Limit the data sending rate, default is around 100-120 messages per second
					boost::this_thread::sleep(boost::posix_time::milliseconds(1));	//Wait for connection
					time_end = boost::posix_time::microsec_clock::local_time();
					diff = time_end - time_start;
				}

				Button_array = new char[10];					//Initialise a new array for our buttons
				DPad_array = new char[4];						//Initialise a new array for our dpad buttons

				///Check our buttons if they are being pressed,if they are being pressed we set a 1 in our button array///
				///Button array => { start, back, left thumb button, right thumb button, left bumper, right bumper, a, b, x, y }///
				///Dpad array => { up, down, left, right }///

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


				///Create our json message that will be sent to NI Mate / websocket server///
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
					Button_array[4] - '0', Button_array[5] - '0', Button_array[6] - '0', Button_array[7] - '0', Button_array[8] - '0', Button_array[9] - '0' };
				json_j["value"]["user_1"]["dpad"] = { DPad_array[0] - '0', DPad_array[1] - '0', DPad_array[2] - '0', DPad_array[3] - '0' };

				message = json_j.dump();											//Dump the Json message to a string so it can be sent
				endpoint.send(id, message);											//Send the message / data to the server
				if (debug_send) std::cout << message << std::endl;					//In case we need to debug the message that is sent

				///Cleanup before next iteration, just in case, avoid memory leaks///
				message.clear();
				json_j.clear();
				delete[] Button_array;
				delete[] DPad_array;
			}
			else if (!poll_param) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));		//Limit the polling rate slightly

				/* 
					The following if / else statements work as follows:
					- Check if the button is being pressed
					- If it is being pressed we check if it is being held down or not, send a message if it was not being held down previously and setting the "button_hold[button]" variable to 1
					- Else we check if it was being pressed / held down before and if it was, we send a message that we are no longer holding the button down
					- This is done for every button, every iteration
				*/


				if (gamepad.IsPressed(XINPUT_GAMEPAD_START)) {
					if (button_hold[0] == 0) {									//Check that the button is not being held down already
						button_array_nopoll[0] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9]};
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();								//Dump the Json message to a string so it can be sent
						if (debug_send) std::cout << message << std::endl;		//Print out message, for debugging
						endpoint.send(id, message);								//Send the message / data to NI Mate / websocket server
						message.clear();										//Clear the message
						json_j.clear();											//Clear our Json message
						button_hold[0] = 1;										//Set button_hold variable to 1, to show that the button is being pressed
					}
				}
				else {
					if (button_hold[0] == 1) {
						button_array_nopoll[0] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();								//Dump the Json message to a string so it can be sent
						if (debug_send) std::cout << message << std::endl;		//Print out message, for debugging
						endpoint.send(id, message);								//Send the message / data to NI Mate / websocket server
						message.clear();										//Clear the message
						json_j.clear();											//Clear our Json message
						button_hold[0] = 0;										//Set button_hold variable to 0, to show that the button is no longer being pressed
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_BACK)) {
					if (button_hold[1] == 0) {
						button_array_nopoll[1] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[1] = 1;
					}
				}
				else {
					if (button_hold[1] == 1) {
						button_array_nopoll[1] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[1] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_LEFT_THUMB)) {
					if (button_hold[2] == 0) {
						button_array_nopoll[2] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[2] = 1;
					}
				}
				else {
					if (button_hold[2] == 1) {
						button_array_nopoll[2] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[2] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_RIGHT_THUMB)) {
					if (button_hold[3] == 0) {
						button_array_nopoll[3] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[3] = 1;
					}
				}
				else {
					if (button_hold[3] == 1) {
						button_array_nopoll[3] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[3] = 0;
					}
				}
				if ( gamepad.IsPressed(XINPUT_GAMEPAD_LEFT_SHOULDER) ) {
					if (button_hold[4] == 0) {
						button_array_nopoll[4] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[4] = 1;
					}
				}
				else {
					if (button_hold[4] == 1) {
						button_array_nopoll[4] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[4] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_RIGHT_SHOULDER)) {
					if (button_hold[5] == 0) {
						button_array_nopoll[5] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[5] = 1;
					}
				}
				else {
					if (button_hold[5] == 1) {
						button_array_nopoll[5] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[5] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_A)) {
					if (button_hold[6] == 0) {
						button_array_nopoll[6] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[6] = 1;
					}
				}
				else {
					if (button_hold[6] == 1) {
						button_array_nopoll[6] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[6] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_B)) {
					if (button_hold[7] == 0) {
						button_array_nopoll[7] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[7] = 1;
					}
				}
				else {
					if (button_hold[7] == 1) {
						button_array_nopoll[7] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[7] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_X)) {
					if (button_hold[8] == 0) {
						button_array_nopoll[8] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[8] = 1;
					}
				}
				else {
					if (button_hold[8] == 1) {
						button_array_nopoll[8] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[8] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_Y)) {
					if (button_hold[9] == 0) {
						button_array_nopoll[9] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[9] = 1;
					}
				}
				else {
					if (button_hold[9] == 1) {
						button_array_nopoll[9] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						button_hold[9] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_UP)) {
					if (dpad_hold[0] == 0) {
						dpad_array_nopoll[0] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[0] = 1;
					}
				}
				else {
					if (dpad_hold[0] == 1) {
						dpad_array_nopoll[0] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[0] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_DOWN)) {
					if (dpad_hold[1] == 0) {
						dpad_array_nopoll[1] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[1] = 1;
					}
				}
				else {
					if (dpad_hold[1] == 1) {
						dpad_array_nopoll[1] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[1] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_LEFT)) {
					if (dpad_hold[2] == 0) {
						dpad_array_nopoll[2] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000); 
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[2] = 1;
					}
				}
				else {
					if (dpad_hold[2] == 1) {
						dpad_array_nopoll[2] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[2] = 0;
					}
				}
				if (gamepad.IsPressed(XINPUT_GAMEPAD_DPAD_RIGHT)) {
					if (dpad_hold[3] == 0) {
						dpad_array_nopoll[3] = 1;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[3] = 1;
					}
				}
				else {
					if (dpad_hold[3] == 1) {
						dpad_array_nopoll[3] = 0;
						json json_j;
						std::string message;
						json_j["type"] = "data";
						json_j["value"]["device_id"] = device_id;
						json_j["value"]["timestamp_ms"] = (time(0) * 1000);
						json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
							button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
						json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
						message = json_j.dump();
						if (debug_send) std::cout << message << std::endl;
						endpoint.send(id, message);
						message.clear();
						json_j.clear();
						dpad_hold[3] = 0;
					}
				}

				///Sticks and trigger have values that change, if there is a change in value we send a new message///
				
				/*
					All following if statements have the basic idea
				*/

				if ( (gamepad.leftStickX != old_val_LStickX) || (gamepad.leftStickY != old_val_LStickY) ) {		//Check if our values have changed, send message with the new values if they have
					json json_j;
					std::string message;
					json_j["type"] = "data";
					json_j["value"]["device_id"] = device_id;
					json_j["value"]["timestamp_ms"] = (time(0) * 1000);
					json_j["value"]["user_1"]["left stick"] = { gamepad.leftStickX, gamepad.leftStickY };		//Send both X and Y values at the same time, 
					json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
						button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
					json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
					message = json_j.dump();
					if (debug_send) std::cout << message << std::endl;											//Print the data in case we need to debug
					endpoint.send(id, message);																	//Send our Json message
					message.clear();																			//Clear our message
					json_j.clear();																				//Clear our Json message
					old_val_LStickX = gamepad.leftStickX, old_val_LStickY = gamepad.leftStickY;					//The new value becomes our old value
				}

				if ( (gamepad.rightStickX != old_val_RStickX) || (gamepad.rightStickY != old_val_RStickY) ) {
					json json_j;
					std::string message;
					json_j["type"] = "data";
					json_j["value"]["device_id"] = device_id;
					json_j["value"]["timestamp_ms"] = (time(0) * 1000);
					json_j["value"]["user_1"]["right stick"] = { gamepad.rightStickX, gamepad.rightStickY };
					json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
						button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
					json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
					message = json_j.dump();
					if (debug_send) std::cout << message << std::endl;
					endpoint.send(id, message);
					message.clear();
					json_j.clear();
					old_val_RStickX = gamepad.rightStickX, old_val_RStickY = gamepad.rightStickY;
				}

				if (gamepad.leftTrigger != old_val_LT) {
					json json_j;
					std::string message;
					json_j["type"] = "data";
					json_j["value"]["device_id"] = device_id;
					json_j["value"]["timestamp_ms"] = (time(0) * 1000);
					json_j["value"]["user_1"]["left trigger"] = gamepad.leftTrigger;
					json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
						button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
					json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
					message = json_j.dump();
					if (debug_send) std::cout << message << std::endl;
					endpoint.send(id, message);
					message.clear();
					json_j.clear();
					old_val_LT = gamepad.leftTrigger;
				}

				if (gamepad.rightTrigger != old_val_RT) {
					json json_j;
					std::string message;
					json_j["type"] = "data";
					json_j["value"]["device_id"] = device_id;
					json_j["value"]["timestamp_ms"] = (time(0) * 1000);
					json_j["value"]["user_1"]["right trigger"] = gamepad.rightTrigger;
					json_j["value"]["user_1"]["buttons"] = { button_array_nopoll[0], button_array_nopoll[1], button_array_nopoll[2], button_array_nopoll[3],
						button_array_nopoll[4], button_array_nopoll[5], button_array_nopoll[6], button_array_nopoll[7], button_array_nopoll[8], button_array_nopoll[9] };
					json_j["value"]["user_1"]["dpad"] = { dpad_array_nopoll[0], dpad_array_nopoll[1], dpad_array_nopoll[2], dpad_array_nopoll[3] };
					message = json_j.dump();
					if (debug_send) std::cout << message << std::endl;
					endpoint.send(id, message);
					message.clear();
					json_j.clear();
					old_val_RT = gamepad.rightTrigger;
				}				
			}
		}
		while ((retries > 0) && (retries < MAX_RETRIES)) {					//A message didn't go through if retries is > 0
			int close_code = websocketpp::close::status::service_restart;	//Reason why we are closing connection
			std::string reason = "Trying to re-connect";
			endpoint.close(id, close_code, reason);							//This will fail if server was closed, might be useless
			id = endpoint.connect(ip_address);								//Try to reconnect to the server, id will be old id + 1
			boost::this_thread::sleep(boost::posix_time::seconds(2)); 		//Have to wait a couple of seconds to reconnect
			endpoint.send(id, start_message);
		}

		while (pause) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));//Pause the program and sleep
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