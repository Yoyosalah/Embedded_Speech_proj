import serial
import time

# --- CONFIGURATION ---
# Change 'COM4' to the port you found in Device Manager
SERIAL_PORT = 'COM4' 
BAUD_RATE = 9600

def collect_data():
    try:
        # Initialize serial connection
        # timeout=1 ensures the script doesn't hang forever if no data arrives
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"--- Connected to {SERIAL_PORT} ---")
        print("Waiting for word trigger... Speak into the microphone.")
        
        # Open file in append mode
        with open("voice_templates.txt", "a") as file:
            while True:
                if ser.in_waiting > 0:
                    # Read the line sent by the ATmega32
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        print(line)
                        file.write(line + "\n")
                        
                        # Optional: If you see the last frame of a word, 
                        # add a separator for easier copy-pasting
                        if "}," in line:
                            file.flush() # Force write to disk
                
                time.sleep(0.01) # Small sleep to prevent 100% CPU usage

    except serial.SerialException as e:
        print(f"Error: Could not open {SERIAL_PORT}. Is it plugged in or used by another app?")
    except KeyboardInterrupt:
        print("\nExiting and saving data...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    collect_data()
