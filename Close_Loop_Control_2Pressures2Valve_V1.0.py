import serial
import time
import csv
import threading
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

"""
**Multi-Pressure & Multi-Valve Controller for Tic Stepper Drivers**
**Overview**
This Python script:
- Controls **two independent Tic stepper motor drivers** for **two pinch valves**.
- Uses **ESP32** as a serial bridge to read **two pressure sensors** and communicate with the Tic controllers.
- Implements **real-time feedback control** to maintain **target pressures** for each valve.
- **Prevents over-pressurization** by holding or adjusting the valve positions dynamically.
- **Logs data** (voltage, pressure, valve positions) to a CSV file.
- **Visualizes live data** in real-time with Matplotlib.

**System Setup**
- **Pressure Sensor 1** → **GPIO4** (Controls **Valve 1 - Tic #1** via **UART0**)
- **Pressure Sensor 2** → **GPIO2** (Controls **Valve 2 - Tic #2** via **UART2**)
- **ESP32 acts as a serial bridge** to forward commands between Python and Tic controllers.

**Control Logic**
- **If pressure < target** → Open valve slightly to increase flow.
- **If pressure = target** → Maintain current valve position.
- **If pressure > target** → Hold position to prevent over-pressurization.

**Data Logging**
- Saves **timestamped** voltage, pressure, and valve positions to `pressure_data.csv`.

**Live Visualization**
- Two separate **figures** plot:
  - **Voltage vs. Time**
  - **Pressure vs. Time**
  - **Valve Position vs. Time**

**Usage**
1. **Run the script**:
2. **Press `Ctrl+C` to safely exit.**

**Future Improvements**
- Implement **PID control** for smoother regulation.
- Add **emergency shut-off** if pressure exceeds a critical threshold.

"""

class TicValveController:
    def __init__(self, port="/dev/cu.usbserial-144420", log_filename="pressure_data.csv"):
        self.esp32 = serial.Serial(port, 115200, timeout=1)
        self.target_pressure_1 = 65  # Target for Valve 1
        self.target_pressure_2 = 10  # Target for Valve 2
        self.position_closed = -850
        self.position_open = 25
        self.log_filename = log_filename
        self.running = True  #Control flag for safe termination
        self.device_number_1 = 14  # Device number for Tic #1 (T834-427108)
        self.device_number_2 = 15  # Device number for Tic #2 (T834-427095)

        with open(self.log_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Voltage 1 (V)", "Pressure 1 (mmHg)", 
                             "Voltage 2 (V)", "Pressure 2 (mmHg)", "Valve1 Position", "Valve2 Position"])

    def read_adc_voltages(self):
        """Reads two voltage values from ESP32 via serial."""
        self.esp32.flushInput()
        while True:  #Keep retrying until valid data is received
            try:
                raw_data = self.esp32.readline().decode("utf-8", errors="ignore").strip()  # Ignore non-UTF-8 bytes
            
                if not raw_data:  #Handle empty reads
                    print("Warning: Received empty ADC data. Retrying...")
                    time.sleep(0.1)
                    continue

                voltage1, voltage2 = map(float, raw_data.split(","))  #Ensure proper conversion
                return voltage1, voltage2  #Return valid voltages
            
            except ValueError:
                print(f"Warning: Invalid ADC data received: {raw_data}. Retrying...")
                time.sleep(0.1)

    def convert_voltage_to_pressure(self, voltage):
        """Converts ADC voltage to pressure (mmHg)."""
        return ((voltage + 0.0295) / 2.01) * 200

    
    def get_variables(self, device_number, offset, length):
        """Retrieves variable data from the Tic."""
        print(f"Requesting {length} bytes from offset {hex(offset)} for device {device_number}")

        self.send_command(device_number, 0xA1, offset, length)  #Properly formatted request
        result = self.esp32.read(length)  #Read binary response

        if len(result) != length:
            print(f"Warning: Expected {length} bytes, got {len(result)}. Raw data: {result.hex()}")
            return None  #Prevents processing invalid data

        return bytearray(result)  #Return byte array for safe processing

    
    def get_current_position(self, device_number):
        """Retrieves the current position of the specified Tic motor."""
        print(f"Fetching current position for device {device_number}...")

        b = self.get_variables(device_number, 0x22, 4)  #Request 4 bytes from offset 0x22

        if b is None or len(b) != 4:
            print(f"Warning: Invalid position data for device {device_number}. Returning 0.")
            return 0

        # ✅ Debugging: Print raw bytes for verification
        print(f"Raw bytes received for device {device_number}: {b.hex()}")

        position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24) 
        if position >= (1 << 31): 
            position -= (1 << 32) 
        return position

    
    def exit_safe_start(self):
        """Sends the 'Exit Safe Start' command to both Tic controllers to enable movement."""
        
        print(f"Exiting safe start mode for {self.device_number_1} (Tic #1)...")
        self.send_command(self.device_number_1, 0x83)  # Send command to Tic #1
        time.sleep(0.1)  # Small delay to ensure proper execution

        print(f"Exiting safe start mode for {self.device_number_2} (Tic #2)...")
        self.send_command(self.device_number_2, 0x83)  # Send command to Tic #2
        time.sleep(0.1)  # Small delay to ensure proper execution

        print("Both valves are now enabled for movement.")

    def set_target_position(self, device_number, target):
        """Moves the specified Tic motor to a target step position."""
        if target is None:
            print(f"Error: Attempted to set position for device {device_number} with None value. Skipping command.")
            return

        print(f"Setting Valve {device_number} position to {target}")

        # Encode 32-bit position in little-endian order
        msbs = ((target >> 7) & 1) | ((target >> 14) & 2) |((target >> 21) & 4) | ((target >> 28) & 8)
        data_bytes = [
            msbs,               # MSBs of the 4-byte value
            (target >> 0) & 0x7F,      # LSB
            (target >> 8) & 0x7F,
            (target >> 16) & 0x7F,
            (target >> 24) & 0x7F
        ]

        self.send_command(device_number, 0xE0, *data_bytes)

    def log_data(self, voltage1, pressure1, voltage2, pressure2, position1, position2):
        """Logs data for both sensors and valves."""
        timestamp = datetime.now().strftime('%Y-%m-%d %I:%M:%S')
        with open(self.log_filename, mode='a', newline='') as file:
            csv.writer(file).writerow([timestamp, voltage1, pressure1, voltage2, pressure2, position1, position2])
        print(f"Logged: {timestamp}, V1:{voltage1:.3f}, P1:{pressure1:.2f}, V2:{voltage2:.3f}, P2:{pressure2:.2f}, Valve1: {position1}, Valve2: {position2}")

    def start_sequence(self):
        """Startup sequence: Fully open → Fully closed for both valves."""
        print("Starting system initialization...")

        # Fully open Valve 1
        print(f"Opening Valve {self.device_number_1}...")
        self.set_target_position(self.device_number_1, self.position_open)
        time.sleep(1)

        # Fully open Valve 2
        print(f"Opening Valve {self.device_number_2}...")
        self.set_target_position(self.device_number_2, self.position_open)
        time.sleep(10)

        # Fully close Valve 1
        print(f"Closing Valve {self.device_number_1}...")
        self.set_target_position(self.device_number_1, self.position_closed)
        time.sleep(1)

        # Fully close Valve 2
        print(f"Closing Valve {self.device_number_2}...")
        self.set_target_position(self.device_number_2, self.position_closed)
        time.sleep(10)

        print("Startup sequence complete.")


    def terminate_sequence(self):
        """Shutdown sequence: Fully close → Fully open for both valves."""
        print("Executing termination sequence...")

        #Stop control loop
        self.running = False
        time.sleep(1)  # Allow thread to stop

        #Ensure "Exit Safe Start" before moving valves
        self.exit_safe_start()  
        time.sleep(0.1)  #Allow Tic to reset    
        
        # Fully close Valve 1
        print(f"Closing Valve {self.device_number_1}...")
        self.set_target_position(self.device_number_1, self.position_closed)
        time.sleep(0.5)

        # Fully close Valve 2
        print(f"Closing Valve {self.device_number_2}...")
        self.set_target_position(self.device_number_2, self.position_closed)
        time.sleep(10)

        # Fully open Valve 1
        print(f"Opening Valve {self.device_number_1}...")
        self.set_target_position(self.device_number_1, self.position_open)
        time.sleep(0.5)

        # Fully open Valve 2
        print(f"Opening Valve {self.device_number_2}...")
        self.set_target_position(self.device_number_2, self.position_open)
        time.sleep(10)

        print("System terminated.")


    def visualize_data(self, filename):
        """Plots real-time data for both Tic devices in separate figures."""
        plt.ion()  # Enable interactive mode

        fig1, ax1 = plt.subplots(3, 1, figsize=(8, 8), sharex=True)  # Tic #1 Plots
        fig2, ax2 = plt.subplots(3, 1, figsize=(8, 8), sharex=True)  # Tic #2 Plots

        while True:
            try:
                df = pd.read_csv(filename)

                if df.empty:
                    time.sleep(1)
                    continue

                timestamps = pd.to_datetime(df["Timestamp"])

                # Clear previous plots
                for axes in [ax1, ax2]:
                    for ax in axes:
                        ax.clear()

                # Plot for Tic #1 (Valve 1)
                ax1[0].plot(timestamps, df["Voltage 1 (V)"], label="Voltage 1 (V)", color='b')
                ax1[1].plot(timestamps, df["Pressure 1 (mmHg)"], label="Pressure 1 (mmHg)", color='g')
                ax1[2].plot(timestamps, df["Valve1 Position"], label="Valve 1 Position", color='r')

                ax1[0].set_title("Tic #1: Voltage Over Time")
                ax1[1].set_title("Tic #1: Pressure Over Time")
                ax1[2].set_title("Tic #1: Valve Position Over Time")

                # Plot for Tic #2 (Valve 2)
                ax2[0].plot(timestamps, df["Voltage 2 (V)"], label="Voltage 2 (V)", color='b')
                ax2[1].plot(timestamps, df["Pressure 2 (mmHg)"], label="Pressure 2 (mmHg)", color='g')
                ax2[2].plot(timestamps, df["Valve2 Position"], label="Valve 2 Position", color='r')

                ax2[0].set_title("Tic #2: Voltage Over Time")
                ax2[1].set_title("Tic #2: Pressure Over Time")
                ax2[2].set_title("Tic #2: Valve Position Over Time")

                # Formatting for all plots
                for axes in [ax1, ax2]:
                    for ax in axes:
                        ax.legend()
                        ax.set_xlabel("Time")
                        ax.tick_params(axis='x', rotation=45)
                        ax.grid(True)

                fig1.tight_layout()
                fig2.tight_layout()
                # plt.pause(1)

            except Exception as e:
                print(f"Error in visualization: {e}")
                time.sleep(1)

    def control_loop(self):
        """Controls both valves based on their respective pressure sensors using a feedback loop."""

        self.start_sequence()  #Ensure valves fully open & close before starting control

        print("Starting real-time dual pressure control with feedback...")

        while self.running:
            #Read voltage and convert to pressure
            voltage1, voltage2 = self.read_adc_voltages()
            pressure1 = self.convert_voltage_to_pressure(voltage1)
            pressure2 = self.convert_voltage_to_pressure(voltage2)

            #Exit safe start to prevent controller shutdown
            self.exit_safe_start()

            #Get actual valve positions after movement
            position1 = self.get_current_position(self.device_number_1)
            position2 = self.get_current_position(self.device_number_2)

            #Debugging: Print current motor positions
            print(f"Device {self.device_number_1} Actual Position: {position1}")
            print(f"Device {self.device_number_2} Actual Position: {position2}")

            #Feedback control logic - Open valve slowly if pressure is low
            if pressure1 < self.target_pressure_1:
                position1 = min(position1 + 5, self.position_open)
            elif pressure1 > self.target_pressure_1:
                pass  # Hold position

            if pressure2 < self.target_pressure_2:
                position2 = min(position2 + 5, self.position_open)
            elif pressure2 > self.target_pressure_2:
                pass  # Hold position

            #Ensure position limits are respected
            position1 = max(self.position_closed, min(position1, self.position_open))
            position2 = max(self.position_closed, min(position2, self.position_open))

            #Command Tic to move to updated positions
            self.set_target_position(self.device_number_1, position1)
            self.set_target_position(self.device_number_2, position2)

            #Get actual valve positions **after movement** for logging
            actual_position1 = self.get_current_position(self.device_number_1)
            actual_position2 = self.get_current_position(self.device_number_2)

            #Log real-time positions from the motor controller
            self.log_data(voltage1, pressure1, voltage2, pressure2, actual_position1, actual_position2)

            #Small delay to allow system response
            time.sleep(1)



if __name__ == "__main__":
    tic = TicValveController(port="/dev/cu.usbserial-144420", log_filename="pressure_data.csv")
    
    tic.exit_safe_start()

    control_thread = threading.Thread(target=tic.control_loop, daemon=True)
    control_thread.start()
    try:
        plt.ion()
        tic.visualize_data("pressure_data.csv")

    except KeyboardInterrupt:
        print("Exiting...")
        tic.terminate_sequence()
        control_thread.join()
        print("Shutdown complete.")

    # # Run Matplotlib visualization
    # plt.ion()
    # tic.visualize_data("pressure_data.csv")

    # try:
    #     tic.control_loop()
    # except KeyboardInterrupt:
    #     print("Exiting...")
    #     tic.terminate_sequence()
