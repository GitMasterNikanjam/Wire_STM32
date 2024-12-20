
// ######################################################################################
// Include libraries: 

#include "Wire.h"

TwoWire::TwoWire(I2C_HandleTypeDef *i2cHandle)
{
    _hi2c = i2cHandle;
    _rxIndex = 0;
    _rxLength = 0;
    _txIndex = 0;
    _txLength = 0;
    _txAddress = 0;
    _transmitting = 0;
    _timeout = HAL_MAX_DELAY;
    _timeoutFlag = false;
    _reset_with_timeout = false;
    errorMessage = "";
}

bool TwoWire::begin() 
{
    if (_hi2c == nullptr) 
    {
        errorMessage = "Error TwoWire: The I2C handle instance is null.";
        return false;
    }

    if (HAL_I2C_Init(_hi2c) != HAL_OK)
    {
        errorMessage = "Error TwoWire: The HAL_I2C_Init() is not succeeded.";
        return false;
    }
		
    return true;
}

bool TwoWire::begin(uint8_t address)
{
    if (_hi2c == nullptr) 
    {
        errorMessage = "Error TwoWire: The I2C handle instance is null.";
        return false;
    }

    _slaveAddress = address;
    _slaveRxCompleteFlag = false;
    _slaveTxCompleteFlag = false;

    _hi2c->Init.OwnAddress1 = address;
    _hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

    if (HAL_I2C_Init(_hi2c) != HAL_OK)
    {
        errorMessage = "Error TwoWire: The HAL_I2C_Init() is not succeeded.";
        return false;
    }

    return true;
}

bool TwoWire::end(void)
{
    // Disable the I2C peripheral
    __HAL_I2C_DISABLE(_hi2c);

    // Deinitialize the I2C peripheral
    if (HAL_I2C_DeInit(_hi2c) != HAL_OK)
    {
        errorMessage = "Error TwoWire: end() is not succeeded.";
        return false;
    }
    return true;
}

bool TwoWire::setClock(uint32_t clock)
{
    if(!((clock == 100000) || (clock == 400000)))
    {
       errorMessage = "Error Wire: The clock value is not correct. try 100000 or 400000.";
       return false; 
    }

    _hi2c->Init.ClockSpeed = clock;
    return true;
}

void TwoWire::beginTransmission(uint8_t address) 
{
    // indicate that we are transmitting
    _transmitting = 1;

    _txAddress = address << 1;  // Convert 7-bit address to HAL-compatible format.

    // reset tx buffer iterator vars
    _txIndex = 0;
    _txLength = 0;
}

uint8_t TwoWire::endTransmission() 
{
    HAL_StatusTypeDef ret;

    switch(_txMode)
    {
        case WIRE_MODE_BLOCK:
            // Send the data in the txBuffer
            ret = HAL_I2C_Master_Transmit(_hi2c, _txAddress, _txBuffer, _txLength, _timeout);
        break;
        case WIRE_MODE_INTERRUPT:
            if (HAL_I2C_Master_Transmit_IT(_hi2c, _txAddress, _txBuffer, _txLength) != HAL_OK)
            {
                errorMessage = "Error: I2C transmission failed.";
                return 1;
            }
        break;
        case WIRE_MODE_DMA:

        break;
    }

    // Reset the state after transmission
    _txIndex = 0;
    _txLength = 0;
    _transmitting = 0;

    // Map HAL return status to detailed codes
    if (ret == HAL_OK)
    {
        return 0; // Success
    }
    else if (ret == HAL_TIMEOUT)
    {
        if(_reset_with_timeout == true)
        {
            recovery();
        }
        _timeoutFlag = true;
        return 5; // Timeout
    }
    else if (ret == HAL_ERROR)
    {
        // Distinguish between NACK errors and other errors
        if (_hi2c->ErrorCode & HAL_I2C_ERROR_AF)
        {
            // NACK during address or data phase
            if (_txIndex == 0)
            {
                return 2; // NACK on transmit of address
            }
            else
            {
                return 3; // NACK on transmit of data
            }
        }
        else
        {
            return 4; // Other error
        }
    }

    return 0;  // Success
}

bool TwoWire::setTxMode(uint8_t mode)
{
  if((mode != WIRE_MODE_BLOCK) && (mode != WIRE_MODE_INTERRUPT) && (mode != WIRE_MODE_DMA))
  {
    return false;
  }

  _txMode = mode;

  return true;
}

bool TwoWire::setRxMode(uint8_t mode)
{
  if((mode != WIRE_MODE_BLOCK) && (mode != WIRE_MODE_INTERRUPT) && (mode != WIRE_MODE_DMA))
  {
    return false;
  }

  _rxMode = mode;

  return true;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) 
{
    _rxIndex = 0;
    _rxLength = 0;

    // Limit the requested quantity to the buffer size
    if (quantity > WIRE_BUFFER_LENGTH) 
    {
        quantity = WIRE_BUFFER_LENGTH;
    }

    switch(_rxMode)
    {
        case WIRE_MODE_BLOCK:
            // Request data from the I2C device
            if (HAL_I2C_Master_Receive(_hi2c, (address << 1), _rxBuffer, quantity, _timeout) == HAL_OK) 
            {
                _rxLength = quantity;
            }
            else
            {
                errorMessage = "Error Wire: The requestFrom() is not succeeded.";
                return false;
            }
        break;
        case WIRE_MODE_INTERRUPT:
            if (HAL_I2C_Master_Receive_IT(_hi2c, (address << 1), _rxBuffer, quantity) != HAL_OK) 
            {
                errorMessage = "Error: I2C request failed.";
                return false;
            }
        break;
        case WIRE_MODE_DMA:

        break;
    }

    return true;  
}

uint8_t TwoWire::write(uint8_t data) 
{
    if(_transmitting)
    {
        // in master transmitter mode
        // don't bother if buffer is full
        if(_txLength >= WIRE_BUFFER_LENGTH)
        {
            errorMessage = "Error TwoWire: Write() is not succeeded.";
            return 0;
        }
        // put byte in tx buffer
        _txBuffer[_txIndex] = data;
        ++_txIndex;
        // update amount in buffer   
        _txLength = _txIndex;
    }
    else
    {
        // slave mode
    }

  return 1;
}

uint8_t TwoWire::write(const uint8_t* data, size_t quantity) 
{
    if(_transmitting)
    {
        // in master transmitter mode
        for(size_t i = 0; i < quantity; ++i)
        {
            if(write(data[i]) == 0)
            {
                return i;
            }
        }
    }
    else
    {
        // slave mode
    }

    return quantity;
}

int TwoWire::available() 
{
    return _rxLength - _rxIndex;
}

int TwoWire::read() 
{
    if (_rxIndex < _rxLength) 
    {
        return _rxBuffer[_rxIndex++];  // Return the next byte
    }
    return -1;  // No data left to read
}

void TwoWire::recovery(void)
{
    HAL_I2C_DeInit(_hi2c);
    HAL_I2C_Init(_hi2c);
    _timeoutFlag = false;
}

void TwoWire::clearWireTimeoutFlag(void)
{
    _timeoutFlag = false;
}

bool TwoWire::getWireTimeoutFlag(void)
{
    return _timeoutFlag;
}

void TwoWire::setWireTimeout(uint32_t timeout, bool reset_with_timeout)
{
    _reset_with_timeout = reset_with_timeout;
    _timeout = timeout;
}

void TwoWire::clearBuffers()
{
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    memset(_txBuffer, 0, sizeof(_txBuffer));
    _rxIndex = 0;
    _txIndex = 0;
    _rxLength = 0;
    _txLength = 0;
    _transmitting = 0;

    _txCompleteFlag = 0;
    _rxCompleteFlag = 0;
    
}

/**
 * Interrupt-based Transmission Callback (to be called when transmission is completed)
 */
void TwoWire::masterTxCpltCallback(void)
{
    TwoWire *wireInstance = reinterpret_cast<TwoWire*>(_hi2c->Instance);
    wireInstance->_txCompleteFlag = 1;  // Set the transmission flag
}

/**
 * Interrupt-based Reception Callback (to be called when reception is completed)
 */
void TwoWire::masterRxCpltCallback(void)
{
    TwoWire *wireInstance = reinterpret_cast<TwoWire*>(_hi2c->Instance);
    wireInstance->_rxCompleteFlag = 1;  // Set the reception flag
}


/**
 * @brief Initialize the I2C peripheral in Slave Mode with the given address.
 * 
 * @param address The address to be assigned to the slave device.
 * @return true if initialization was successful.
 */

/**
 * @brief Request data from an I2C master as a slave.
 * 
 * @param quantity Number of bytes to request.
 * @return true if succeeded.
 */
bool TwoWire::requestFromSlave(uint8_t quantity)
{
    _slaveRxCompleteFlag = 0;

    if (HAL_I2C_Slave_Receive_IT(_hi2c, _slaveBuffer, quantity) != HAL_OK) {
        errorMessage = "Slave receive request failed.";
        return false;
    }

    // Wait for the receive operation to complete
    while (!_slaveRxCompleteFlag) {
        // Timeout or other mechanisms can be added here
    }

    return true;
}

/**
 * @brief Write a byte of data as an I2C slave.
 * 
 * @param data The byte of data to write.
 * @return true if succeeded.
 */
bool TwoWire::writeSlave(uint8_t data)
{
    _slaveTxCompleteFlag = 0;

    if (HAL_I2C_Slave_Transmit_IT(_hi2c, &data, 1) != HAL_OK) {
        errorMessage = "Slave transmit failed.";
        return false;
    }

    // Wait for transmission to complete
    while (!_slaveTxCompleteFlag) {
        // Timeout or other mechanisms can be added here
    }

    return true;
}

/**
 * @brief Write multiple bytes as an I2C slave.
 * 
 * @param data Pointer to the data to send.
 * @param length Number of bytes to send.
 * @return true if succeeded.
 */
bool TwoWire::writeSlave(const uint8_t* data, size_t length)
{
    _slaveTxCompleteFlag = 0;

    if (HAL_I2C_Slave_Transmit_IT(_hi2c, (uint8_t*)data, length) != HAL_OK) {
        errorMessage = "Slave transmit multiple bytes failed.";
        return false;
    }

    // Wait for transmission to complete
    while (!_slaveTxCompleteFlag) {
        // Timeout or other mechanisms can be added here
    }

    return true;
}

/**
 * @brief Read data from the slave buffer.
 * 
 * @return The byte of data read, or -1 if no data is available.
 */
int TwoWire::readSlave()
{
    if (_slaveRxCompleteFlag) {
        // Read the data from the buffer
        return _slaveBuffer[0]; // Example: Reading the first byte
    }

    return -1;  // No data to read
}

/**
 * Interrupt-based Transmission Callback (called when slave transmission is completed)
 */
void TwoWire::slaveTxCpltCallback(void)
{
    TwoWire *wireInstance = reinterpret_cast<TwoWire*>(_hi2c->Instance);
    wireInstance->_slaveTxCompleteFlag = 1;  // Set the transmission flag
}

/**
 * Interrupt-based Reception Callback (called when slave reception is completed)
 */
void TwoWire::slaveRxCpltCallback(void)
{
    TwoWire *wireInstance = reinterpret_cast<TwoWire*>(_hi2c->Instance);
    wireInstance->_slaveRxCompleteFlag = 1;  // Set the reception flag
}

/**
 * Interrupt-based Listen Callback (called when a slave address is matched)
 */
void TwoWire::slaveAddrCallback(void)
{
    // Handle address match
    TwoWire *wireInstance = reinterpret_cast<TwoWire*>(_hi2c->Instance);
    // Implement address match behavior (e.g., reset buffers or indicate readiness)
}
