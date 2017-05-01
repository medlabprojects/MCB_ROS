#ifndef WIZNETHARDWARE_H
#define WIZNETHARDWARE_H

#include <Ethernet.h>

class WiznetHardware {
    private:
        EthernetClient my_client;
        int my_error;

    public:
        static const int ERROR_NONE = 0;
        static const int ERROR_NOT_SETUP = -1;
        static const int ERROR_WIZNET_IP_FAIL = -2;
        static const int ERROR_CONNECT_FAIL = -3;
        IPAddress wiznet_ip;
        uint8_t wiznet_mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

    public:
        WiznetHardware() 
            : my_client(),
              wiznet_ip(192, 168, 0, 40)
        {
	        sendResetPulse();
	        my_error = ERROR_NOT_SETUP;
	        return;
        }

    void init() {
        unsigned int ros_port = 11411;      // default ros port
	    IPAddress ros_ip(192, 168, 0, 1);   // IP of server running ROS
	    Ethernet.begin(wiznet_mac, wiznet_ip); // set the MAC address and ip address of the Wiznet board.
        delay(1000); // give wiznet time to configure
	    if ( !(Ethernet.localIP() == wiznet_ip)) {
            sendResetPulse(); // attempt to reset Wiznet
            my_error = ERROR_WIZNET_IP_FAIL;
            return;
        }
        if (my_client.connect(ros_ip, ros_port) != 1) {
            my_error = ERROR_CONNECT_FAIL;
            return;
        }
        my_error = ERROR_NONE;
        return;
    }

    int read() {
        unsigned int character_read;
    
        if (my_client.available() > 0) { // there are bytes to be read
            char tmp = my_client.read();
            character_read = tmp; 
        }
        else // nothing to read, return -1
            return -1;

        return character_read;
    }

    void write(uint8_t* data, int length) {
        digitalWriteFast(7, HIGH);
        my_client.write(data, length);
        digitalWriteFast(7, LOW);
    }

    unsigned long time() {
        return millis();
    }
  
    int &error() {
        return my_error;
    }

    uint8_t status() {
	    return my_client.status();
    }

    void sendResetPulse() {
        pinMode(9, OUTPUT);
	    digitalWrite(9, LOW);    // begin reset the WIZ820io
	    pinMode(10, OUTPUT);
	    digitalWrite(10, HIGH);  // de-select WIZ820io
	    pinMode(4, OUTPUT);
	    digitalWrite(4, HIGH);   // de-select the SD Card
	    digitalWrite(9, HIGH);   // end reset pulse
    }
};

#endif
