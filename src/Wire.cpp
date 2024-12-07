
// ######################################################################################
// Include libraries: 

#include "Wire.h"

TwoWire::TwoWire(I2C_HandleTypeDef *i2cHandle)
    : _hi2c(i2cHandle), _rxIndex(0), _rxLength(0), _txIndex(0), _txLength(0), _txAddress(0), _transmitting(0) 
{
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

    #if UNDER_DEVELOP
        // Enable I2C interrupts after initialization
        enableI2CInterrupts();
    #endif
		
    return true;
}

bool TwoWire::begin(uint8_t address)
{
    if (_hi2c == nullptr) 
    {
        errorMessage = "Error TwoWire: The I2C handle instance is null.";
        return false;
    }

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
    // Send the data in the txBuffer
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_hi2c, _txAddress, _txBuffer, _txLength, _timeout);

    #if UNDER_DEVELOP
        if (HAL_I2C_Master_Transmit_IT(_hi2c, _txAddress, _txBuffer, _txLength) != HAL_OK)
        {
            snprintf(errorMessage, WIRE_ERROR_MSG_LENGTH, "Error: I2C transmission failed.");
            return 1;
        }
    #endif

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

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) 
{
    _rxIndex = 0;
    _rxLength = 0;

    // Limit the requested quantity to the buffer size
    if (quantity > WIRE_BUFFER_LENGTH) 
    {
        quantity = WIRE_BUFFER_LENGTH;
    }

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

    #if UNDER_DEVELOP
        if (HAL_I2C_Master_Receive_IT(_hi2c, (address << 1), _rxBuffer, quantity) != HAL_OK) 
        {
            snprintf(errorMessage, WIRE_ERROR_MSG_LENGTH, "Error: I2C request failed.");
            return false;
        }
    #endif
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

    #if UNDER_DEVELOP
        _txCompleteFlag = 0;
        _rxCompleteFlag = 0;
    #endif
}

#if UNDER_DEVELOP

    void TwoWire::enableI2CInterrupts()
    {
        // Enable the I2C interrupt for the corresponding I2C peripheral
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);  // Change to the correct I2C interrupt number for your device (I2C1_EV_IRQn in this case)
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);  // Enable error interrupt for I2C
    }

    void TwoWire::disableI2CInterrupts()
    {
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);  // Disable the event interrupt
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);  // Disable the error interrupt
    }

#endif


#if UNDER_DEVELOP

    /**
     * Interrupt-based Transmission Callback (to be called when transmission is completed)
     */
    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c->Instance == I2C1)  // Ensure we're handling the correct I2C instance
        {
            Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
            wireInstance->_txCompleteFlag = 1;  // Set the transmission flag
        }
    }

    /**
     * Interrupt-based Reception Callback (to be called when reception is completed)
     */
    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c->Instance == I2C1)  // Ensure we're handling the correct I2C instance
        {
            Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
            wireInstance->_rxCompleteFlag = 1;  // Set the reception flag
        }
    }

    /**
     * Interrupt-based Error Callback (to be called in case of error)
     */
    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
    {
        if (hi2c->Instance == I2C1)  // Ensure we're handling the correct I2C instance
        {
            Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
            snprintf(wireInstance->errorMessage, WIRE_ERROR_MSG_LENGTH, "I2C Error detected!");
            // Handle error (add specific error handling as needed)
        }
    }

#endif

#if UNDER_DEVELOP

/**
 * @brief Initialize the I2C peripheral in Slave Mode with the given address.
 * 
 * @param address The address to be assigned to the slave device.
 * @return true if initialization was successful.
 */
bool Wire::beginSlave(uint8_t address)
{
    _slaveAddress = address;
    _slaveRxCompleteFlag = 0;
    _slaveTxCompleteFlag = 0;

    // Configure I2C in slave mode
    _hi2c->Instance = I2C1; // Set the peripheral instance, modify if necessary
    _hi2c->Init.OwnAddress1 = _slaveAddress; // Set slave address
    _hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(_hi2c) != HAL_OK) {
        snprintf(errorMessage, WIRE_ERROR_MSG_LENGTH, "I2C Slave initialization failed.");
        return false;
    }

    // Enable interrupt for the slave
    HAL_I2C_EnableListen_IT(_hi2c); // Enable I2C interrupt for slave listen mode

    return true;
}

/**
 * @brief Request data from an I2C master as a slave.
 * 
 * @param quantity Number of bytes to request.
 * @return true if succeeded.
 */
bool Wire::requestFromSlave(uint8_t quantity)
{
    _slaveRxCompleteFlag = 0;

    if (HAL_I2C_Slave_Receive_IT(_hi2c, _slaveBuffer, quantity) != HAL_OK) {
        snprintf(errorMessage, WIRE_ERROR_MSG_LENGTH, "Slave receive request failed.");
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
bool Wire::writeSlave(uint8_t data)
{
    _slaveTxCompleteFlag = 0;

    if (HAL_I2C_Slave_Transmit_IT(_hi2c, &data, 1) != HAL_OK) {
        snprintf(errorMessage, WIRE_ERROR_MSG_LENGTH, "Slave transmit failed.");
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
bool Wire::writeSlave(const uint8_t* data, size_t length)
{
    _slaveTxCompleteFlag = 0;

    if (HAL_I2C_Slave_Transmit_IT(_hi2c, (uint8_t*)data, length) != HAL_OK) {
        snprintf(errorMessage, WIRE_ERROR_MSG_LENGTH, "Slave transmit multiple bytes failed.");
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
int Wire::readSlave()
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
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
        wireInstance->_slaveTxCompleteFlag = 1;  // Set the transmission flag
    }
}

/**
 * Interrupt-based Reception Callback (called when slave reception is completed)
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
        wireInstance->_slaveRxCompleteFlag = 1;  // Set the reception flag
    }
}

/**
 * Interrupt-based Listen Callback (called when a slave address is matched)
 */
void HAL_I2C_SlaveAddrCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        // Handle address match
        Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
        // Implement address match behavior (e.g., reset buffers or indicate readiness)
    }
}

/**
 * Interrupt-based Error Callback for the slave
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        Wire *wireInstance = reinterpret_cast<Wire*>(hi2c->Instance);
        snprintf(wireInstance->errorMessage, WIRE_ERROR_MSG_LENGTH, "I2C Error detected!");
        // Handle error (add specific error handling as needed)
    }
}

#endif