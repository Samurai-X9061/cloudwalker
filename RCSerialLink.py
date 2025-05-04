import serial, time 

class iBus:
    def __init__(self,serPort):
        self.ser = serial.Serial(
            port = serPort,
            baudrate = 115200,
            timeout = 0.1,
            )
        self.buffer = bytearray()
        self.channels = [0]*14  # iBUS typically supports 14 channels
        try:
            if not self.ser.is_open:
                self.ser.open()
        except Exception as error:
            print ("\n\nError opening iBUS"+self.ser.port+" port.\n"+str(error)+"\n\n")

    def read_packet(self, checksum = True):
        """Read data from serial port and process when a full packet is available"""
        if self.ser.in_waiting and self.ser.is_open:
            
            # Read available bytes
            data = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(data)
            
            # Process buffer for packets
            while len(self.buffer) >= 32:  # iBUS packet is 32 bytes (2 header + 28 data bytes + 2 checksum)
                # Look for the header byte (0x20)
                if self.buffer[0] != 0x20:
                    self.buffer.pop(0)
                    continue
                
                # If we don't have enough bytes yet, wait for more
                if len(self.buffer) < 32:
                    break
                
                # Extract the packet
                packet = self.buffer[:32]
                self.buffer = bytearray()
                
                # Verify checksum
                if not checksum or self._verify_checksum(packet):
                    self._parse_channels(packet)
                    return True
                
        return False
    
    def _verify_checksum(self, packet):
        """Verify the packet checksum"""
        # For iBUS, checksum is typically the lower 16 bits of the sum of all bytes except the checksum bytes
        checksum = 0xFFFF - sum(packet[:30])
        received_checksum = packet[30] + (packet[31] << 8)
        return checksum == received_checksum
    
    def _parse_channels(self, packet):
        """Parse channel data from the packet"""
        for i in range(14):
            # Each channel is 2 bytes, little-endian
            start_idx = 2 + (i * 2)  # Start from byte 2 (after header)
            self.channels[i] = packet[start_idx] + (packet[start_idx + 1] << 8)
    
    def get_channels(self):
        """Return the current channel values"""
        while True:
            #print(self.read_packet())
            if self.read_packet():
                return self.channels
            time.sleep(0.001)
    
    def __del__(self):
        """Close the serial connection"""
        self.ser.close()