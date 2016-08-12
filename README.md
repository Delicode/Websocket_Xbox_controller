##Sending controller inputs from Xbox 360 controller to NI Mate using websockets

###Project goals
This project was created to show the possibilities of using websockets in conjunction with [NI Mate](https://ni-mate.com/), to use data from other devices, in this case an Xbox controller, for additional functionality. This documentation will focus on what is needed and how the program is used. The code is not in any way elegant, mostly due to my limited knowledge in the C++ language. Feel free to change the code to your own liking. Even though the code was made to work with [NI Mate](https://ni-mate.com/), it can be used with any websocket server as long as the server can send a "start" message back to the client so that the program knows when to start sending data. The software works on Windows. Tested on a laptop with Windows 10.

#####Devices used in my test
- Laptop with Windows 10
- PC Wireless Gaming Receiver
- Xbox 360 controller

#### Software / Libraries used
- Microsoft Visual Studios 2015
- [Boost](http://www.boost.org/)
- [Websocket++](https://github.com/zaphoyd/websocketpp)
- [Nlohmann Json](https://github.com/nlohmann/json)
- I followed [this](https://katyscode.wordpress.com/2013/08/30/xinput-tutorial-part-1-adding-gamepad-support-to-your-windows-game/) example to get the controller to work

####Initial configuration

We can start by connecting the PC Wireless receiver to our computer and see if it shows up in the device manager. For me  it showed up as a "Unknown device" and I had to install the drivers for it. To do so you need to do the following:
- Right click on the "unknown device" and choose "Update Driver Software.."
- Next choose "Browse my computer for driver software"
- After that choose "Let me pick from a list of device drivers on my computer"
- From the list choose "Xbox 360 peripheral" and after that "Xbox 360 wireless receiver for windows"
- Now the device should show up as an Xbox controller and connect to the computer

After that we will install our libraries by following the installation instructions online:

- [Websocket++](https://github.com/zaphoyd/websocketpp/wiki/Build-on-debian)
- [Boost](http://www.boost.org/doc/libs/1_61_0/more/getting_started/windows.html)
- [Nlohmann Json](https://github.com/nlohmann/json), download project, place the json.hpp file into our project folder


####Using the program
- The program will first ask for the IP address and port number of the computer running NI Mate / the websocket server.
- Input the IP address and port number according to the on-screen example and press _**"enter"**_.
- Next the program will connect to the server, send an initial message and then wait for the start message to be received from the server.
- After receiving the start message the program will, by default, wait for input from the controllers and send data to the server for every controller input.
- The program can be changed to poll the controller and send messages at a speed of about 100-120 messages per second, if you want the messages in that way instead.
- NI Mate will now have a new tab where the data can be found, as well as the different parameters that can be sent to the program and commands.
- The polling of the controller is a parameter that can be set. You can also set the instensity of the rumble motor, duration of the rumble and pattern of the rumble.
- To pause the program, you can send a _"**stop**"_ command from NI Mate or websocket server.
- To stop the program, either disconnect from the server or then you can remove the device from NI Mate which sends a _"**quit**"_ message to the Xbox controller program.
