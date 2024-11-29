#include "Wire.h"

/**
 * @brief Constructor: Initialize buffer indices and store I2C handle.
 * 
 * @param i2cHandle Pointer to the HAL I2C_HandleTypeDef structure.
 */
Wire::Wire(I2C_HandleTypeDef *i2cHandle)
    : _hi2c1(i2cHandle), _rxIndex(0), _rxLength(0), _txAddress(0) {}

/**
 * @brief Initialize the I2C bus.
 * 
 * This function is intended to be called after the HAL I2C instance is 
 * already initialized using CubeMX or HAL initialization code.
 */
void Wire::begin() {
    // If CubeMX or other initialization code is used, no specific action is needed here.
    // Additional custom initialization can be added if necessary.
}

/**
 * @brief Begin transmission with the target I2C device.
 * 
 * @param address 7-bit address of the target device.
 */
void Wire::beginTransmission(uint8_t address) {
    _txAddress = address << 1;  // Convert 7-bit address to HAL-compatible format.
}

/**
 * @brief End transmission and send data to the I2C device.
 * 
 * @return uint8_t Returns 0 on success, 1 on failure.
 */
uint8_t Wire::endTransmission() {
    // HAL expects data to transmit; if no data, send an empty frame
    return HAL_I2C_Master_Transmit(_hi2c1, _txAddress, nullptr, 0, HAL_MAX_DELAY) == HAL_OK ? 0 : 1;
}

/**
 * @brief Request data from an I2C device.
 * 
 * @param address 7-bit address of the target device.
 * @param quantity Number of bytes to request.
 * @return uint8_t Number of bytes successfully received.
 */
uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity) {
    _rxIndex = 0;
    _rxLength = 0;

    // Limit the requested quantity to the buffer size
    if (quantity > BUFFER_LENGTH) {
        quantity = BUFFER_LENGTH;
    }

    // Request data from the I2C device
    if (HAL_I2C_Master_Receive(_hi2c1, (address << 1), _rxBuffer, quantity, HAL_MAX_DELAY) == HAL_OK) {
        _rxLength = quantity;
    }
    return _rxLength;  // Return the number of bytes received
}

/**
 * @brief Write a single byte to the I2C device.
 * 
 * @param data The byte to write.
 * @return size_t Returns 1 if successful, 0 otherwise.
 */
size_t Wire::write(uint8_t data) {
    // Send a single byte
    return HAL_I2C_Master_Transmit(_hi2c1, _txAddress, &data, 1, HAL_MAX_DELAY) == HAL_OK ? 1 : 0;
}

/**
 * @brief Write multiple bytes to the I2C device.
 * 
 * @param data Pointer to the data buffer to write.
 * @param length Number of bytes to write.
 * @return size_t Number of bytes successfully written.
 */
size_t Wire::write(const uint8_t* data, size_t length) {
    // Send a block of data
    return HAL_I2C_Master_Transmit(_hi2c1, _txAddress, (uint8_t*)data, length, HAL_MAX_DELAY) == HAL_OK ? length : 0;
}

/**
 * @brief Check how many bytes are available in the receive buffer.
 * 
 * @return int Number of bytes available to read.
 */
int Wire::available() {
    return _rxLength - _rxIndex;
}

/**
 * @brief Read a single byte from the receive buffer.
 * 
 * @return int The next byte from the buffer, or -1 if no data is available.
 */
int Wire::read() {
    if (_rxIndex < _rxLength) {
        return _rxBuffer[_rxIndex++];  // Return the next byte
    }
    return -1;  // No data left to read
}
