#pragma once

// Define the target MCU family here
// #define STM32F4
#define STM32F1
// #define STM32H7

// ###############################################################################################
// Include libraries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

#include <string>

// ###############################################################################################
// Define Global Macros:

#define WIRE_BUFFER_LENGTH 32  // Buffer size for data transfer

#define WIRE_MODE_BLOCK         0
#define WIRE_MODE_INTERRUPT     1
#define WIRE_MODE_DMA			2

// #define UNDER_DEVELOP

// ##############################################################################################

/**
 * @class TwoWire
 */
class TwoWire 
{
public:

    /// @brief Last error occurred for the object.
    std::string errorMessage;

    /**
     * @brief Constructor.
     * @param i2cHandle is a HAL I2C_HandleTypeDef pointer.
     * @note - The i2cHandle can be defined outside of the Wire class and then used within the class.
     * 
     * @note - It is recommended that the I2C peripheral be set and initialized outside of the Wire class.
     * 
     * @note - The GPIO I2C peripheral configurations must be set outside of the Wire class.
     */
    TwoWire(I2C_HandleTypeDef *i2cHandle);

    /**
     * @brief Initialize I2C as a master.
     * @note This function initializes the Wire library and join the I2C bus as a controller or a peripheral. 
     * This function should normally be called only once.
     * @return true if succeeded.
     */
    bool begin();     

    /**
     * @brief Initialize I2C as a slave.
     * @note This function initializes the Wire library and join the I2C bus as a controller or a peripheral. 
     * This function should normally be called only once.
     * @param address: the 7-bit slave address (optional); if not specified, join the bus as a controller device.
     * @return true if succeeded.
     */
    bool begin(uint8_t address);       

    /**
     * @brief Disables the I2C bus and releases any allocated resources.
     * Stops the Wire library functions from further use.
     * @return true if succeeded.
     */
    bool end(void);

    /**
     * @brief Set Clock speed of i2c prepheral.
     * @return true if succeeded.
     * @warning Use this function before begin method.
     */
    bool setClock(uint32_t clock);

    /**
     * @brief This function begins a transmission to the I2C peripheral device with the given address. 
     * Subsequently, queue bytes for transmission with the write() function and transmit them by calling endTransmission().
     * @param address: the 7-bit address of the device to transmit to.
     *  */                 
    void beginTransmission(uint8_t address); 

    /**
     * @brief This function ends a transmission to a peripheral device that was begun by beginTransmission() 
     * and transmits the bytes that were queued by write().
     * @return 
     * 0: success.
     * 
     * 1: data too long to fit in transmit buffer.
     * 
     * 2: received NACK on transmit of address.
     * 
     * 3: received NACK on transmit of data.
     * 
     * 4: other error.
     * 
     * 5: timeout
     */
    uint8_t endTransmission();               

    /**
     * @brief Set I2C transmit mode that can be Block mode, Interrupt mode, DMA mode.
     * @param mode: Can be 0: Block mode, 1: Interrupt mode, 3: DMA mode.
     * @return true if succeeded.
     * @warning TxMode should be set before calling the begin() method.
     */
    bool setTxMode(uint8_t mode);

    /**
     * @brief Set I2C receive mode that can be Block mode, Interrupt mode, DMA mode.
     * @param mode: Can be 0: Block mode, 1: Interrupt mode, 3: DMA mode.
     * @return true if succeeded.
     * @warning RxMode should be set before calling the begin() method.
     */
    bool setRxMode(uint8_t mode);

    /**
     * @brief Request data from an I2C device.
     * @param address: the 7-bit slave address of the device to request bytes from.
     * @param quantity: the number of bytes to request.
     * @note This function is used by the controller device to request bytes from a peripheral device. 
     * The bytes may then be retrieved with the available() and read() functions. 
     * @return uint8_t: the number of bytes returned from the peripheral device.
     */
    uint8_t requestFrom(uint8_t address, uint8_t quantity); 

    /**
     * @brief Write a byte.
     * @param data: a value to send as a single byte.
     * @note This function writes data from a peripheral device in response to a request from a controller device, 
     * or queues bytes for transmission from a controller to peripheral device (in-between calls to beginTransmission() and endTransmission()).
     * @return The number of bytes written (reading this number is optional).
     */
    uint8_t write(uint8_t data);              

    /**
     * @brief Write multiple bytes
     * @param data: a value to send as a single byte.
     * @param quantity: the number of bytes to transmit.
     * @note This function writes data from a peripheral device in response to a request from a controller device, 
     * or queues bytes for transmission from a controller to peripheral device (in-between calls to beginTransmission() and endTransmission()).
     * @return The number of bytes written (reading this number is optional).
     */
    uint8_t write(const uint8_t* data, size_t quantity); 

    /**
     * @brief Check how many bytes are available to read
     * @return int Number of bytes available.
     */
    int available();               

    /**
     * @brief Read a byte
     * @return int The byte value or -1 if no data is available.
     *  */          
    int read();   

    /**
     * @brief Deinit and init i2c phrepheral for recovery i2c if nedded.
     */
    void recovery(void);   

    /**
     * @brief Sets the timeout for Wire transmissions in master mode.
     * @param timeout a timeout: timeout in milliseconds, if zero then timeout checking is disabled
     * @param reset_on_timeout: if true then Wire hardware will be automatically reset on timeout
     * @note these timeouts are almost always an indication of an underlying problem, such as misbehaving devices, noise, 
     * insufficient shielding, or other electrical problems. These timeouts will prevent your sketch from locking up, 
     * but not solve these problems. In such situations there will often (also) be data corruption which doesn’t result 
     * in a timeout or other error and remains undetected. So when a timeout happens, 
     * it is likely that some data previously read or written is also corrupted. 
     * Additional measures might be needed to more reliably detect such issues (e.g. checksums or reading back written values) 
     * and recover from them (e.g. full system reset). This timeout and such additional measures should be seen as a last line 
     * of defence, when possible the underlying cause should be fixed instead.
     */
    void setWireTimeout(uint32_t timeout, bool reset_with_timeout);

    /**
     * @brief Get wire timeout flag.
     */
    bool getWireTimeoutFlag(void);

    /**
     * @brief Clear timeout flag.
     */
    void clearWireTimeoutFlag(void);

    void masterTxCpltCallback(void);

    void masterRxCpltCallback(void);

    void slaveTxCpltCallback(void);

    void slaveRxCpltCallback(void);

    void slaveAddrCallback(void);

    /**
     * @brief Start receiving data as a slave.
     * @param quantity Number of bytes to receive.
     * @return true if succeeded.
     */
    bool requestFromSlave(uint8_t quantity); 

    /**
     * @brief Write data as a slave.
     * @param data The data byte to send.
     * @return true if succeeded.
     */
    bool writeSlave(uint8_t data);

    /**
     * @brief Write multiple bytes as a slave.
     * @param data Pointer to the data to send.
     * @param length Number of bytes to send.
     * @return true if succeeded.
     */
    bool writeSlave(const uint8_t* data, size_t length);

    /**
     * @brief Read data as a slave.
     * @return The byte of data read, or -1 if no data available.
     */
    int readSlave();

private:

    /**
     * @brief HAL I2C handle
     */
    I2C_HandleTypeDef *_hi2c;        

    /// @brief Receive buffer   
    uint8_t _rxBuffer[WIRE_BUFFER_LENGTH];   

    /// @brief Transmit buffer   
    uint8_t _txBuffer[WIRE_BUFFER_LENGTH];   

    /// @brief Current index in the receive buffer     
    uint8_t _rxIndex;   

    /// @brief Current index in the send buffer 
    uint8_t _txIndex;    

    /// @brief Length of received data                 
    uint8_t _rxLength; 

    /// @brief Length of send data  
    uint8_t _txLength;        

    /// @brief Address of the target device              
    uint8_t _txAddress;      

    /// @brief Transmission status flag
    uint8_t _transmitting;     

    uint32_t _timeout;

    bool _reset_with_timeout;

    bool _timeoutFlag;

    /**
     * @brief Slave address for I2C communication.
     */
    uint8_t _slaveAddress;

    // Add flags for slave interrupts
    volatile bool _slaveRxCompleteFlag;

    volatile bool _slaveTxCompleteFlag;

    uint8_t _slaveBuffer[WIRE_BUFFER_LENGTH]; // Slave data buffer

    // Interrupt flags
    volatile bool _txCompleteFlag; ///< TX complete flag

    volatile bool _rxCompleteFlag; ///< RX complete flag

    /**
     * @brief I2C transmit mode that can be Block mode, Interrupt mode, DMA mode.
     * @note - Can be 0: Block mode, 1: Interrupt mode, 3: DMA mode.
     * @return true if succeeded.
     */
    volatile uint8_t _txMode;

    /**
     * @brief I2C receive mode that can be Block mode, Interrupt mode, DMA mode.
     * @note - Can be 0: Block mode, 1: Interrupt mode, 3: DMA mode.
     * @return true if succeeded.
     */
    volatile uint8_t _rxMode;

    /// @brief Clears the send and receive buffers 
    void clearBuffers();         
};


