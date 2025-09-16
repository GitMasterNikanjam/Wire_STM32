
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
    _slaveAddress = 0;
    _transmitting = 0;
    _timeout = HAL_MAX_DELAY;
    _timeoutFlag = false;
    _resetWithTimeout = false;
    _txCompleteFlag = true;
    _rxCompleteFlag = true;
    _slaveRxCompleteFlag = true;
    _slaveTxCompleteFlag = true;
    errorCode = 0;

    _txMode = WIRE_MODE_BLOCK;
    _rxMode = WIRE_MODE_BLOCK;

    _SCL_PORT = nullptr;
    _SDA_PORT = nullptr;
    _SCL_PIN = GPIO_PIN_0;
    _SDA_PIN = GPIO_PIN_0;
    
}

bool TwoWire::begin() 
{
    if (_hi2c == nullptr) 
    {
        // errorMessage = "The I2C handle instance is null.";
        errorCode = 1;
        return false;
    }

    busRecoveryGPIO();

    if(recovery() == false)
    {
        // errorMessage = "The recovery() is not succeeded.";
        errorCode = 2;
        return false;
    }
		
    return true;
}

bool TwoWire::begin(uint8_t address)
{
    if (_hi2c == nullptr) 
    {
        // errorMessage = "The I2C handle instance is null.";
        errorCode = 1;
        return false;
    }

    _slaveAddress = address;

    _hi2c->Init.OwnAddress1 = address;
    _hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

    if(recovery() == false)
    {
        // errorMessage = "The recovery() is not succeeded.";
        errorCode = 2;
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
        // errorMessage = "end() is not succeeded.";
        errorCode = 1;
        return false;
    }
    return true;
}

bool TwoWire::setClock(uint32_t clock)
{
    #if defined(STM32H7)
        // Typical Fast-mode 400k @ i2c kernel clock ~ 100 MHz (adjust to your clock tree!)
        // Use CubeMX to generate exact Timing, or inject via a config hook.
        if(clock == 100000)      _hi2c->Init.Timing = 0x00C0EAFF; // example value
        else if(clock == 400000) _hi2c->Init.Timing = 0x20303E5D; // example value
        else { errorCode=1; return false; }
    #elif defined(STM32F4) || defined(STM32F1)
        if(clock!=100000 && clock!=400000){ errorCode=1; return false; }
        _hi2c->Init.ClockSpeed = clock;
    #endif
    
    return true;
}

bool TwoWire::beginTransmission(uint8_t address) 
{
    // Check if transmit channel be free
    if(_transmitting == 1)
    {
        // errorMessage = "I2C is transmitting. End transmitting before start new transmision.";
        errorCode = 1;
        return false;
    }
    // indicate that we are transmitting
    _transmitting = 1;

    _txAddress = address << 1;  // Convert 7-bit address to HAL-compatible format.

    // reset tx buffer iterator vars
    switch (_txMode)
    {
    case WIRE_MODE_BLOCK:
        _txIndex = 0;
        _txLength = 0;
        break;
    case WIRE_MODE_INTERRUPT:
        if(_txCompleteFlag == true)
        {
            _txIndex = 0;
        }
        _txLength = 0;
    break;
    case WIRE_MODE_DMA:

    break;
    default:
        break;
    }

    return true;
}

uint8_t TwoWire::endTransmission() 
{
    if(_transmitting == 0)
    {
        return 4;
    }
    
    _transmitting = 0;

    HAL_StatusTypeDef ret = HAL_ERROR;

    switch(_txMode)
    {
        case WIRE_MODE_BLOCK:
            // Send the data in the txBuffer
            ret = HAL_I2C_Master_Transmit(_hi2c, _txAddress, _txBuffer, _txLength, _timeout);
            // Reset the state after transmission
            _txIndex = 0;
            _txLength = 0;
        break;
        case WIRE_MODE_INTERRUPT:
            if(_txCompleteFlag == true)
            {
                _txCompleteFlag = false;
                ret = HAL_I2C_Master_Transmit_IT(_hi2c, _txAddress, _txBuffer, _txLength);
                _txLength = 0;
            }
            else
            {
                ret = HAL_I2C_Master_Transmit_IT(_hi2c, _txAddress, _txBuffer + _txIndex - _txLength, _txLength);
                _txLength = 0;
            }
        break;
        case WIRE_MODE_DMA:
            _txCompleteFlag = false;
            ret = HAL_I2C_Master_Transmit_DMA(_hi2c, _txAddress, _txBuffer, _txIndex);
        break;
    }

    // Map HAL return status to detailed codes
    if (ret == HAL_OK)
    {
        return 0; // Success
    }
    else if (ret == HAL_TIMEOUT)
    {
        if(_resetWithTimeout == true)
        {
            recovery();
            busRecoveryGPIO();
        }
        _timeoutFlag = true;
        return 5; // Timeout
    }
    else
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
    // Limit the requested quantity to the empty size space of buffer 
    if (quantity > (WIRE_BUFFER_LENGTH - _rxLength)) 
    {
        quantity = (WIRE_BUFFER_LENGTH - _rxLength);
    }
    
    switch(_rxMode)
    {
        case WIRE_MODE_BLOCK:
            _rxIndex = 0;
            _rxLength = 0;
            // Request data from the I2C device
            if (HAL_I2C_Master_Receive(_hi2c, (address << 1), _rxBuffer, quantity, _timeout) == HAL_OK) 
            {
                _rxLength = quantity;
            }
            else
            {
                // errorMessage = "The requestFrom() is not succeeded.";
                errorCode = 1;
                return 0;
            }
        break;
        case WIRE_MODE_INTERRUPT:
            if(_rxCompleteFlag == true)
            {
                _rxIndex = 0;
                _rxLength = 0;
                _rxCompleteFlag = false;
                if (HAL_I2C_Master_Receive_IT(_hi2c, (address << 1), _rxBuffer, quantity) == HAL_OK) 
                {
                    _rxLength = quantity;
                }
                else
                {
                    // errorMessage = "The requestFrom() is not succeeded.";
                    errorCode = 2;
                    return 0;
                }
            }
            else
            {
                _rxCompleteFlag = false;
                if (HAL_I2C_Master_Receive_IT(_hi2c, (address << 1), _rxBuffer + _rxLength, quantity) == HAL_OK) 
                {
                    _rxLength += quantity;
                }
                else
                {
                    // errorMessage = "The requestFrom() is not succeeded.";
                    errorCode = 3;
                    return 0;
                }
            }
        break;
        case WIRE_MODE_DMA:
            _rxCompleteFlag = false;
            if (HAL_I2C_Master_Receive_DMA(_hi2c, (address<<1), _rxBuffer, quantity) == HAL_OK)
            {
                _rxLength = quantity;
                // user can poll getRxCompleteFlag()
            } 
            else 
            {
                errorCode = 4;
                return 0;
            }
        break;
    }

    return quantity;  
}

uint8_t TwoWire::write(uint8_t data) 
{
    if(_transmitting)
    {
        // in master transmitter mode
        // don't bother if buffer is full
        if(_txIndex >= WIRE_BUFFER_LENGTH)
        {
            // errorMessage = "Write() is not succeeded. TXbuffer is overflow.";
            errorCode = 1;
            return 0;
        }
        // put byte in tx buffer
        _txBuffer[_txIndex] = data;
        ++_txIndex;
        // update amount size length for transmision.   
        _txLength++;
    }
    else
    {
        // errorMessage = "Master is not begin transmiting.";
        errorCode = 2;
        return 0;
    }

  return 1;
}

uint8_t TwoWire::write(const uint8_t* data, uint8_t quantity) 
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

HAL_StatusTypeDef TwoWire::writeReg8(uint8_t dev7, uint8_t reg, const uint8_t* data, size_t len){
  return HAL_I2C_Mem_Write(_hi2c, dev7<<1, reg, I2C_MEMADD_SIZE_8BIT,
                           const_cast<uint8_t*>(data), len, _timeout);
}
HAL_StatusTypeDef TwoWire::readReg8(uint8_t dev7, uint8_t reg, uint8_t* data, size_t len){
  return HAL_I2C_Mem_Read(_hi2c, dev7<<1, reg, I2C_MEMADD_SIZE_8BIT,
                          data, len, _timeout);
}
HAL_StatusTypeDef TwoWire::writeReg16(uint8_t dev7, uint16_t reg, const uint8_t* data, size_t len){
  return HAL_I2C_Mem_Write(_hi2c, dev7<<1, reg, I2C_MEMADD_SIZE_16BIT,
                           const_cast<uint8_t*>(data), len, _timeout);
}
HAL_StatusTypeDef TwoWire::readReg16(uint8_t dev7, uint16_t reg, uint8_t* data, size_t len){
  return HAL_I2C_Mem_Read(_hi2c, dev7<<1, reg, I2C_MEMADD_SIZE_16BIT,
                          data, len, _timeout);
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

bool TwoWire::recovery(void)
{   
    HAL_StatusTypeDef status = HAL_I2C_DeInit(_hi2c);
    if (status != HAL_OK) return false;

    HAL_Delay(5);

    status = HAL_I2C_Init(_hi2c);
    if (status != HAL_OK) return false;

    _timeoutFlag = false;
    return true;
}

bool TwoWire::busRecoveryGPIO(void)
{
    if( (_SCL_PORT == nullptr) || (_SDA_PORT == nullptr) )
    {
        return false;
    }

    // Disable I2C peripheral to take control of pins
    __HAL_I2C_DISABLE(_hi2c);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Reconfigure SCL and SDA as GPIO open-drain outputs
    GPIO_InitStruct.Pin = _SCL_PIN; // SCL []
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = _SDA_PIN; // SDA
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(_SDA_PORT, &GPIO_InitStruct); 

    // Ensure SCL released HIGH first (thanks to pull-up)
    HAL_GPIO_WritePin(_SCL_PORT, _SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    // 2. Toggle SCL manually and release SDA
    for (int i = 0; i < 9; i++) {

        if (HAL_GPIO_ReadPin(_SDA_PORT, _SDA_PIN) == GPIO_PIN_SET)
            break; // SDA already released

        HAL_GPIO_WritePin(_SCL_PORT, _SCL_PIN, GPIO_PIN_RESET); // SCL Low
        HAL_Delay(1);

        HAL_GPIO_WritePin(_SCL_PORT, _SCL_PIN, GPIO_PIN_SET);   // SCL High
        HAL_Delay(1); 
    }

    // Generate a STOP: SDA rising while SCL high
    // Make SDA OD output temporarily to drive LOW->HIGH
    GPIO_InitStruct.Pin   = _SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(_SDA_PORT, &GPIO_InitStruct);

    // Drive SDA low while SCL high, then release
    HAL_GPIO_WritePin(_SCL_PORT, _SCL_PIN, GPIO_PIN_SET);   // SCL High
    HAL_Delay(1);
    HAL_GPIO_WritePin(_SDA_PORT, _SDA_PIN, GPIO_PIN_RESET); // SDA Low
    HAL_Delay(1);
    HAL_GPIO_WritePin(_SDA_PORT, _SDA_PIN, GPIO_PIN_SET);   // SDA High
    HAL_Delay(1);

    // Re-init I2C peripheral
    // (Reconfigure alternate function, speed, etc. as in MX_I2C_Init)
    __HAL_I2C_ENABLE(_hi2c);

    // 3. Re-init I2C
    HAL_StatusTypeDef status = HAL_I2C_DeInit(_hi2c);
    if (status != HAL_OK) return false;

    HAL_Delay(5);

    status = HAL_I2C_Init(_hi2c);
    if (status != HAL_OK) return false;

    _timeoutFlag = false;
    return true;
}

void TwoWire::clearWireTimeoutFlag(void)
{
    _timeoutFlag = false;
}

bool TwoWire::getWireTimeoutFlag(void)
{
    return _timeoutFlag;
}

void TwoWire::setWireTimeout(uint32_t timeout, bool resetWithTimeout)
{
    _resetWithTimeout = resetWithTimeout;
    _timeout = timeout;
}

void TwoWire::_clearBuffers()
{
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    memset(_txBuffer, 0, sizeof(_txBuffer));
    _rxIndex = 0;
    _txIndex = 0;
    _rxLength = 0;
    _txLength = 0;
    _transmitting = 0;

    _txCompleteFlag = true;
    _rxCompleteFlag = true;
    _slaveRxCompleteFlag = true;
    _slaveTxCompleteFlag = true;

    _timeoutFlag = false;
}

HAL_StatusTypeDef TwoWire::ackPoll(uint16_t dev7bit, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t devAddr = dev7bit << 1; // HAL expects 8-bit address (7-bit << 1)
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        if (HAL_I2C_IsDeviceReady(_hi2c, devAddr, 1, 2) == HAL_OK) return HAL_OK;
    }
    return HAL_TIMEOUT;
}

/**
 * Interrupt-based Transmission Callback (to be called when transmission is completed)
 */
void TwoWire::masterTxCpltCallback(void)
{
    _txLength = _txIndex = 0;
    _txCompleteFlag = true;  // Set the transmission flag
}

/**
 * Interrupt-based Reception Callback (to be called when reception is completed)
 */
void TwoWire::masterRxCpltCallback(void)
{
    _rxCompleteFlag = true;  // Set the reception flag
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
        // errorMessage = "Slave receive request failed.";
        errorCode = 1;
        return false;
    }

    // Wait for the receive operation to complete
    uint32_t start = HAL_GetTick();
    while (!_slaveRxCompleteFlag) {
        // Timeout or other mechanisms can be added here
        if (HAL_GetTick() - start > _timeout) {
            // errorMessage = "Timeout in slave receive";
            errorCode = 2;
            return false;
        }
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
        // errorMessage = "Slave transmit failed.";
        errorCode = 1;
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
    if (length > WIRE_BUFFER_LENGTH) 
    {
        // errorMessage = "Slave TX length exceeds buffer size.";
        errorCode = 1;
        return false;
    }

    _slaveTxCompleteFlag = 0;

    if (HAL_I2C_Slave_Transmit_IT(_hi2c, (uint8_t*)data, length) != HAL_OK) {
        // errorMessage = "Slave transmit multiple bytes failed.";
        errorCode = 2;
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

bool TwoWire::IsDeviceReady(uint8_t address)
{
    if (HAL_I2C_IsDeviceReady(_hi2c, address << 1 , 3, 100) != HAL_OK) 
    {
        return false;
    }
    return true;
}

HAL_I2C_StateTypeDef TwoWire::GetState(void)
{
    return HAL_I2C_GetState(_hi2c);
}

void TwoWire::setGPIO(GPIO_TypeDef* SCLPort, uint16_t SCLPin, GPIO_TypeDef* SDAPort, uint16_t SDAPIn)
{
    _SCL_PORT = SCLPort;
    _SCL_PIN = SCLPin;
    _SDA_PORT = SDAPort;
    _SDA_PIN = SDAPIn;
}

/**
 * Interrupt-based Transmission Callback (called when slave transmission is completed)
 */
void TwoWire::slaveTxCpltCallback(void)
{
    _slaveTxCompleteFlag = true;  // Set the transmission flag
}

/**
 * Interrupt-based Reception Callback (called when slave reception is completed)
 */
void TwoWire::slaveRxCpltCallback(void)
{
    _slaveRxCompleteFlag = true;  // Set the reception flag
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




