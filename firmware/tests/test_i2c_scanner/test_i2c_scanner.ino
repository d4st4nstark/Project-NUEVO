/**
 * @file test_i2c_scanner.ino
 * @brief I2C bus scanner for debugging
 *
 * Scans I2C bus and reports all detected devices.
 * Useful for verifying PCA9685, IMU, and other I2C peripherals.
 *
 * Hardware:
 * - Arduino Mega 2560
 * - I2C devices connected to SDA (pin 20) and SCL (pin 21)
 * - Pull-up resistors (usually built into modules)
 *
 * Expected Devices:
 * - 0x40: PCA9685 PWM controller (if installed)
 * - 0x68: ICM-20948 IMU (if installed)
 * - 0x70: PCA9685 ALL_CALL address (broadcast, normal)
 */

#include <Wire.h>

namespace {

const uint32_t kScanSpeedsHz[] = {
    100000UL,
    50000UL,
    25000UL,
};

const char *describeWireError(uint8_t error) {
    switch (error) {
        case 0: return "ACK";
        case 1: return "buffer overflow";
        case 2: return "NACK on address";
        case 3: return "NACK on data";
        case 4: return "other TWI error";
        case 5: return "timeout";
        default: return "unknown";
    }
}

void printKnownDeviceName(uint8_t address) {
    switch (address) {
        case 0x40:
            Serial.print(F("(PCA9685 PWM Controller)"));
            break;
        case 0x68:
            Serial.print(F("(ICM-20948 IMU or MPU-6050)"));
            break;
        case 0x69:
            Serial.print(F("(ICM-20948 IMU alternate)"));
            break;
        case 0x70:
            Serial.print(F("(PCA9685 ALL_CALL broadcast)"));
            break;
        default:
            Serial.print(F("(Unknown device)"));
            break;
    }
}

void scanBusAtSpeed(uint32_t speedHz) {
    Wire.setClock(speedHz);
    Wire.setWireTimeout(5000, true);

    Serial.println(F("----------------------------------------"));
    Serial.print(F("Scanning at "));
    Serial.print(speedHz / 1000UL);
    Serial.println(F(" kHz"));
    Serial.println(F("----------------------------------------"));

    uint8_t deviceCount = 0;
    uint8_t errorCount = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print(F("Device found at 0x"));
            if (address < 16) Serial.print(F("0"));
            Serial.print(address, HEX);
            Serial.print(F("  "));
            printKnownDeviceName(address);
            Serial.println();
            deviceCount++;
        } else if (error != 2) {
            Serial.print(F("Error "));
            Serial.print(error);
            Serial.print(F(" ("));
            Serial.print(describeWireError(error));
            Serial.print(F(") at 0x"));
            if (address < 16) Serial.print(F("0"));
            Serial.println(address, HEX);
            errorCount++;
        }
    }

    Serial.println();
    Serial.print(F("Summary @ "));
    Serial.print(speedHz / 1000UL);
    Serial.print(F(" kHz: devices="));
    Serial.print(deviceCount);
    Serial.print(F(", errors="));
    Serial.println(errorCount);
    Serial.println();
}

void runScan() {
    for (uint8_t i = 0; i < (sizeof(kScanSpeedsHz) / sizeof(kScanSpeedsHz[0])); i++) {
        scanBusAtSpeed(kScanSpeedsHz[i]);
        delay(50);
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        ; // Wait for serial port to connect
    }

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  I2C Bus Scanner"));
    Serial.println(F("========================================"));
    Serial.println();

    Wire.begin();
    Wire.setWireTimeout(5000, true);

    Serial.println(F("Scanning I2C bus (0x01-0x7F) at multiple speeds..."));
    Serial.println(F("Notes:"));
    Serial.println(F("  - Error 2 = no ACK on address (normal if no device at that slot)"));
    Serial.println(F("  - Error 4/5 = bus-level problem worth investigating"));
    Serial.println();

    runScan();

    Serial.println();
    Serial.println(F("Scan complete. Type any key to rescan."));
    Serial.println(F("========================================"));
}

void loop() {
    if (Serial.available() > 0) {
        Serial.read();  // Clear buffer
        Serial.println(F("\nRescanning..."));
        delay(100);
        runScan();
        Serial.println(F("\nScan complete. Type any key to rescan."));
        Serial.println(F("========================================"));
    }
}
