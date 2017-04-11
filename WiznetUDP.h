#ifndef WIZNETUDP_H
#define WIZNETUDP_H

#include <Ethernet.h>

class WiznetUDP {
private:
  EthernetUDP my_client;
  byte wiznet_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  IPAddress wiznet_ip;
  unsigned int wiznet_port = 8888;
  IPAddress ros_ip;
  unsigned int ros_port = 11411; // default ros port
  
public:
  WiznetUDP() 
    : my_client()
    , wiznet_ip(192, 168, 0, 40)
    , ros_ip(192, 168, 0, 1) // default ros ip
  {     
    return;
  }

  void init() {
    Ethernet.begin(wiznet_mac, wiznet_ip); // set the MAC address and ip addres of the Wiznet board.
    delay(5000); // startup delay as a fail-safe to upload a new sketch
    my_client.begin(wiznet_port);
  }

  int read() {
    unsigned int character_read;
    
    // if there's data available, read a packet
    int packetSize = my_client.parsePacket();
    if (packetSize) {
      char *tmp;
      my_client.read(tmp, 1);
      character_read = *tmp;
    }
    else // nothing to read, return -1
      return -1;

    return character_read;
  }

  void write(uint8_t* data, int length) {
    my_client.beginPacket(ros_ip, ros_port);
    my_client.write(data, length);
    my_client.endPacket();
  }

  unsigned long time() {
    return millis();
  }

};

#endif
