def get_box_config(serial_port, baud_rate=115200):
    """
    Get the current box configuration from Betaflight using MSP protocol
    
    Args:
        serial_port: Serial port connected to flight controller
        baud_rate: Baud rate for serial communication
        
    Returns:
        Dictionary with box names, IDs and mode ranges
    """
    import serial
    import time
    
    # MSP command constants
    MSP_BOXNAMES = 116
    MSP_BOXIDS = 119
    MSP_MODE_RANGES = 34
    
    def create_msp_message(command, data=None):
        if data is None:
            data = []
        
        # MSP V1 protocol format: $M<direction><length><command><data><crc>
        message = bytearray([36, 77, 60])  # $M
        length = len(data)
        message.append(length)
        message.append(command)
        
        # Calculate CRC (XOR of length, command and all data bytes)
        crc = length ^ command
        for byte in data:
            message.append(byte)
            crc ^= byte
            
        message.append(crc)
        return message
    
    def parse_msp_response(response, command):
        if len(response) < 6:
            return None
            
        # Check for correct format: $M>length,command,data,crc
        if response[0] != 36 or response[1] != 77 or response[2] != 62:  # $M>
            return None
            
        length = response[3]
        cmd = response[4]
        
        if cmd != command:
            return None
            
        # Extract data
        data = response[5:-1]  # Exclude CRC
        return data
    
    def read_msp_response(ser, command, timeout=1.0):
        response = bytearray()
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                response.extend(ser.read(ser.in_waiting))
                
                # Look for a complete MSP response
                for i in range(len(response)):
                    if i >= 3 and response[i-3:i] == bytearray([36, 77, 62]):  # $M>
                        data = parse_msp_response(response[i-3:], command)
                        if data is not None:
                            return data
        
        return None
    
    try:
        # Open serial connection
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Allow time for connection to stabilize
        
        # Initialize variables
        box_names = []
        box_ids = []
        mode_ranges = []
        
        # Request box names
        ser.write(create_msp_message(MSP_BOXNAMES))
        ser.flush()
        box_names_data = read_msp_response(ser, MSP_BOXNAMES)
        
        # Parse box names (semicolon separated)
        if box_names_data:
            names_str = box_names_data.decode('ascii', errors='ignore')
            box_names = names_str.split(';')
            #box_names = [name for name in box_names if name]  # Remove empty names
        
        # Request box IDs
        ser.write(create_msp_message(MSP_BOXIDS))
        ser.flush()
        box_ids_data = read_msp_response(ser, MSP_BOXIDS)
        
        # Parse box IDs
        if box_ids_data:
            box_ids = list(box_ids_data)
        
        # Request mode ranges
        ser.write(create_msp_message(MSP_MODE_RANGES))
        ser.flush()
        mode_ranges_data = read_msp_response(ser, MSP_MODE_RANGES)
        
        # Parse mode ranges
        if mode_ranges_data and len(mode_ranges_data) % 4 == 0:
            for i in range(0, len(mode_ranges_data), 4):
                mode_id = mode_ranges_data[i]
                aux_channel = mode_ranges_data[i+1]
                start_step = mode_ranges_data[i+2]
                end_step = mode_ranges_data[i+3]
                
                # Find mode name if possible
                mode_name = "UNKNOWN"
                if mode_id < len(box_ids):
                    idx = box_ids.index(mode_id)
                    if idx < len(box_names):
                        mode_name = box_names[idx]
                
                mode_ranges.append({
                    "mode_id": mode_id,
                    "mode_name": mode_name,
                    "aux_channel": aux_channel,
                    "start_step": start_step,
                    "end_step": end_step
                })
        
        # Close the serial connection
        ser.close()
        
        # Create result dictionary
        result = {
            "box_names": box_names,
            "box_ids": box_ids,
            "mode_ranges": mode_ranges
        }
        
        # Create a dictionary mapping mode names to IDs
        modes_map = {}
        for i, name in enumerate(box_names):
            if i < len(box_ids):
                modes_map[name] = box_ids[i]
                
        result["modes_map"] = modes_map
        
        return result
        
    except Exception as e:
        print(f"Error: {e}")
        return None

# Replace with your actual serial port
serial_port = "COM3"  # Linux example, use COM port for Windows

box_config = get_box_config(serial_port)
if box_config:
    print("Available flight modes:")
    for name, id in box_config["modes_map"].items():
        print(f"- {name}: ID {id}")
    
    print("\nActive mode ranges:")
    for mode in box_config["mode_ranges"]:
        print(f"- {mode['mode_name']} (ID {mode['mode_id']}): AUX{mode['aux_channel']+1}, Range {mode['start_step']}-{mode['end_step']}")