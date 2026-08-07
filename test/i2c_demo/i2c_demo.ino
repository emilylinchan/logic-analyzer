/*
 * i2c_demo.ino - I2C traffic generator for the FPGA logic analyzer demo.
 *
 * Runs on an ESP32. Reads the Model ID register from a VL53L1X time-of-flight
 * (ToF) sensor over I2C, producing a transaction the logic analyzer can trigger
 * on and capture:
 *
 *   START | addr+W | ACK | reg | ACK | REPEATED START | addr+R | ACK | data | NACK | STOP
 */

#include <Wire.h>

// ----- Configuration -----
#define I2C_SDA  21      // Logic analyzer channel 0
#define I2C_SCL  22      // Logic analyzer channel 1
 
#define DEV_ADDR 0x29    // VL53L1X default I2C address
#define REG_ADDR 0x010F  // VL53L1X Model ID register

// ----- Setup -----
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n--- VL53L1X Model ID Reader ---");

    // Initialize I2C 100 kHz standard mode (~10 samples/bit at 1 MS/s rate on logic analyzer)
    Wire.begin(I2C_SDA, I2C_SCL); 

    // Read back the Model ID
    uint8_t modelId = readRegister8(DEV_ADDR, REG_ADDR);
    Serial.printf("Model ID Read: 0x%x\n", modelId);

    // Check against expected value (0xEA)
    if (modelId == 0xEA) {
        Serial.println("Success! VL53L1X communication verified.");
    } 
    else {
        Serial.println("Error: Unexpected Model ID or communication failure.");
    }
}

// ----- Loop -----
void loop() {
    // Leave empty
}

// ----- Helper Functions -----

// Read an 8-bit value from a 16-bit register address
uint8_t readRegister8(uint8_t devAddress, uint16_t regAddress) {

    // Write to slave to request register address data
    Wire.beginTransmission(devAddress);
    Wire.write(regAddress >> 8);         // High byte
    Wire.write(regAddress & 0xFF);       // Low byte

    // Check for slave ACK
    if (Wire.endTransmission(false) != 0) {
        Serial.println("I2C Tranmission Error!");
        return 0x00;
    }

    // Read 1 byte of data from the specified register
    Wire.requestFrom(devAddress, (uint8_t)1);

    if (Wire.available()) {
        return Wire.read();
    }
    return 0x00; // Return 0 if no data received
}
