#include <Arduino.h>
#include <HardwareSerial.h>

#define SENSOR1_PIN 4   // ADC for Pressure Sensor 1 (Tic #1)
#define SENSOR2_PIN 2   // ADC for Pressure Sensor 2 (Tic #2)

#define TXD1 25   // UART1 TX -> Tic #1 RX
#define RXD1 26   // UART1 RX -> Tic #1 TX

#define TXD2 17   // UART2 TX -> Tic #2 RX
#define RXD2 16   // UART2 RX -> Tic #2 TX

HardwareSerial Tic1(1);  // UART1 for Tic #1
HardwareSerial Tic2(2);  // UART2 for Tic #2

const uint8_t device_number_1 = 14;  // Tic1 Device Number
const uint8_t device_number_2 = 15;  // Tic2 Device Number

void setup() {
    Serial.begin(115200);  // USB Serial (Python communication)
    Tic1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // Tic #1
    Tic2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Tic #2

    Serial.println("ESP32 Tic Serial Bridge Initialized");
}

// Writes bytes to a UART port and returns success/failure.
bool write_to_tic(HardwareSerial &tic, uint8_t *buffer, size_t size) {
    size_t written = tic.write(buffer, size);
    if (written != size) {
        Serial.println("Error: Failed to write complete command to Tic.");
        return false;
    }
    return true;
}

//Reads bytes from the Tic stepper controller
ssize_t read_from_tic(HardwareSerial &tic, uint8_t *buffer, size_t size) {
    size_t received = 0;
    unsigned long start_time = millis();
    const unsigned long timeout_ms = 100;  // Timeout for Tic response

    while (received < size) {
        if (tic.available()) {
            buffer[received++] = tic.read();
        }
        if (millis() - start_time > timeout_ms) {
            Serial.println("Error: Timeout reading from Tic.");
            return -1;
        }
    }
    return received;
}

// Processes and forwards commands from Python to the correct Tic
void process_command(uint8_t *command_packet, int command_length) {
    if (command_length < 3) return;  // Ignore incomplete commands

    uint8_t device_number = command_packet[1];  // Device number is second byte

    if (device_number == device_number_1) {
        if (write_to_tic(Tic1, command_packet, command_length)) {
            uint8_t response[16];  //Buffer for Tic response
            if (read_from_tic(Tic1, response, sizeof(response)) > 0) {
                Serial.write(response, sizeof(response));  //Forward to Python
            }
        }
    } 
    else if (device_number == device_number_2) {
        if (write_to_tic(Tic2, command_packet, command_length)) {
            uint8_t response[16];  
            if (read_from_tic(Tic2, response, sizeof(response)) > 0) {
                Serial.write(response, sizeof(response));  // ✅ Forward to Python
            }
        }
    }
}

void loop() {
    //Read both ADC sensors and send voltage values to Python
    int raw_adc1 = analogRead(SENSOR1_PIN);
    float voltage1 = (raw_adc1 / 4095.0) * 3.3;

    int raw_adc2 = analogRead(SENSOR2_PIN);
    float voltage2 = (raw_adc2 / 4095.0) * 3.3;

    Serial.print(voltage1, 3);
    Serial.print(",");
    Serial.println(voltage2, 3);

    //Read serial commands from Python
    if (Serial.available()) {
        uint8_t command_packet[16];
        int command_length = Serial.readBytes(command_packet, sizeof(command_packet));

        if (command_length > 0) {
            process_command(command_packet, command_length);
        }
    }
}
