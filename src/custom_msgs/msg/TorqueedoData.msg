# Custom message representing a packet sent over the torqueedo bus

#The id of the device to which the packet is adressed
int16 device_id

#The type of message the packet represents
int16 message_id

#The raw hex string of the packet body
string body
