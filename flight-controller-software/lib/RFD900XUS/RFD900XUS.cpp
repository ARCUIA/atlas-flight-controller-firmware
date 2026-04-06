#include "RFD900XUS.h"

// Constructor implementation
RFD900XUS::RFD900XUS(SerialBus& radio_bus) : _radio_bus(radio_bus){
}

bool RFD900XUS::transmit_to_base_station(const telemetry_packet& data) {
    _radio_bus.send_bytes((uint8_t*)&data, sizeof(data));
    return true; 
}

bool RFD900XUS::is_base_station_command_available() {
    return _radio_bus.available() > 0; // Temporary, add more to this later
}


// TODO: Add code here to handle base station commands
