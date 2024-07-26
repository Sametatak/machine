import serial

def read_serial_data():
    try:
        # Open the serial port
        ser =  serial.Serial('/dev/ttyACM0', 9600)
        
        # Loop to continuously read data
        while True:
        
            
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
                
    except serial.SerialException as e:
        print(f"Error reading serial port: {e}")
    except KeyboardInterrupt:
        print("Serial reading stopped by user.")
    finally:
        ser.close()

# Call the function to start reading data
read_serial_data()

