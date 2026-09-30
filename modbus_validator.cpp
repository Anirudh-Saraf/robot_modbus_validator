#include <iostream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <string_view>

class ModbusValidator {
public:
    // Calculates the Modbus RTU CRC-16 checksum over a block of data [9]
    // Polynomial: 0xA001 (reversed 0x8005) [9]
    static std::uint16_t calculateCRC(const std::uint8_t* data, std::size_t length) {
        std::uint16_t crc = 0xFFFF; // Initialize CRC register to all 1s [9]

        for (std::size_t i = 0; i < length; ++i) {
            crc ^= static_cast<std::uint16_t>(data[i]); // XOR byte into least significant byte of CRC [9]

            for (int bit = 0; bit < 8; ++bit) { // Loop through each bit [9]
                if ((crc & 0x0001) != 0) {       // If the LSB is set [9]
                    crc = (crc >> 1) ^ 0xA001;  // Shift right and XOR with polynomial [9]
                } else {
                    crc = crc >> 1;             // Just shift right [9]
                }
            }
        }
        return crc;
    }

    // Appends the calculated 2-byte CRC to a raw payload (Modbus uses Little-Endian for CRC) [8]
    static std::vector<std::uint8_t> assembleFrame(const std::vector<std::uint8_t>& payload) {
        std::uint16_t crc = calculateCRC(payload.data(), payload.size());
        
        std::vector<std::uint8_t> complete_frame = payload;
        complete_frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));        // Low byte first [8]
        complete_frame.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF)); // High byte second [8]
        
        return complete_frame;
    }

    // Validates if an incoming Modbus RTU frame has a correct CRC [8]
    static bool validateFrame(const std::vector<std::uint8_t>& frame) {
        if (frame.size() < 3) return false; // Minimum frame: Address (1), Function (1), CRC (2)

        // Calculate CRC over everything EXCEPT the last 2 bytes (the sent CRC) [8]
        std::size_t payload_length = frame.size() - 2;
        std::uint16_t calculated_crc = calculateCRC(frame.data(), payload_length);

        // Reconstruct sent CRC (Little-Endian) [8]
        std::uint16_t sent_crc = frame[frame.size() - 2] | (static_cast<std::uint16_t>(frame[frame.size() - 1]) << 8);

        return calculated_crc == sent_crc;
    }
};

void printHexDump(std::string_view label, const std::vector<std::uint8_t>& bytes) {
    std::cout << label << ": [ ";
    for (auto b : bytes) {
        std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b) << " ";
    }
    std::cout << "]\n";
}

int main() {
    std::cout << "===  Real-Time Modbus RTU Frame Validator Started  ===\n\n";

    // Scenario: Command a robotic gripper (Slave ID: 0x02) to write holding register 0x0100 with value 0x00FF (close gripper)
    // Raw Payload: [ 02 (Slave ID) | 06 (Write Single Register) | 01 00 (Reg Addr) | 00 FF (Reg Value) ]
    std::vector<std::uint8_t> gripper_command = { 0x02, 0x06, 0x01, 0x00, 0x00, 0xFF };

    std::cout << "--- 1. Frame Assembly (Transmitter Side) ---\n";
    printHexDump("Raw Gripper Payload ", gripper_command);

    // Assemble frame by appending the CRC
    std::vector<std::uint8_t> tx_frame = ModbusValidator::assembleFrame(gripper_command);
    printHexDump("Assembled TX Frame  ", tx_frame);

    std::cout << "\n--- 2. Transmission & Reception (Receiver Side) ---\n";
    
    // Test Case A: Frame received without noise
    std::vector<std::uint8_t> rx_frame_good = tx_frame;
    std::cout << "Testing Good Frame: ";
    if (ModbusValidator::validateFrame(rx_frame_good)) {
        std::cout << " CRC Verified! Gripper command accepted and executed.\n";
    } else {
        std::cout << " CRC Error! Frame dropped.\n";
    }

    // Test Case B: Frame corrupted by electrical noise (Bit flipped in payload byte 4)
    std::vector<std::uint8_t> rx_frame_bad = tx_frame;
    rx_frame_bad[4] = 0x00; // Bit flip simulation (0xFF changed to 0x00 due to EMI)
    
    std::cout << "Testing Bad Frame:  ";
    printHexDump("Corrupted Payload   ", rx_frame_bad);
    std::cout << "               ↳ Result: ";
    if (ModbusValidator::validateFrame(rx_frame_bad)) {
        std::cout << " CRC Verified! (Warning: Dangerous, undetected corruption)\n";
    } else {
        std::cout << " CRC Error! Noise detected. Frame safely dropped before motor actuation.\n";
    }

    std::cout << "\n===  Modbus Validator Shutting Down  ===\n";
    return 0;
}
