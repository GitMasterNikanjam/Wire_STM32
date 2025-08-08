#pragma once

// ###########################################################################################

/**
 * @file Wire.h
 * @brief Cross-platform I2C wrapper header for STM32 HAL.
 *
 * This file provides MCU selection and HAL includes based on user-defined macros.
 * Ensure you provide a "mcu_select.h" header alongside this file to define your target MCU.
 * Supported MCUs: STM32F1, STM32F4, STM32H7.
 */

// ##############################################################################################
// ===============================================================================
// MCU Selection Configuration
// ===============================================================================

#include "mcu_select.h"

/*
    If there is not exist mcu_select.h at beside of this header file, Create it and put this bellow following content. 
    Then select your desired MCU that want work with.
*/
// ----------------------------------------------------------------
// mcu_select.h file:

// Define the target MCU family here
// Uncomment the desired MCU family definition below:

// #define STM32F1
// #define STM32F4
// #define STM32H7

// ----------------------------------------------------------------

// ###############################################################################################
// ========================================================================
// Include libraries:
// ===============================================================================

#if defined(STM32F1)
#include "stm32f1xx_hal.h"          ///< HAL driver for STM32F1 series
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"          ///< HAL driver for STM32F4 series
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"          ///< HAL driver for STM32H7 series
#else
#error "Unsupported MCU family. Please define STM32F1, STM32F4, or STM32H7 in mcu_select.h."
#endif

#include <string>                   ///< Required for error messages and string processing

// ###############################################################################################
// ===============================================================================
// Global Macro Definitions
// ===============================================================================

#ifndef WIRE_BUFFER_LENGTH
    /**
     * @brief Buffer size used for I2C transmit and receive operations.
     */
    #define WIRE_BUFFER_LENGTH      32  
#endif

#ifndef WIRE_MODE_BLOCK
    /**
     * @brief I2C communication mode: blocking (polling) mode.
     */
    #define WIRE_MODE_BLOCK         0
#endif

#ifndef WIRE_MODE_INTERRUPT
    /**
     * @brief I2C communication mode: interrupt-based communication.
     */
    #define WIRE_MODE_INTERRUPT     1
#endif

#ifndef WIRE_MODE_DMA
    /**
     * @brief I2C communication mode: DMA-based communication.
     */
    #define WIRE_MODE_DMA		    2
#endif 

// #define UNDER_DEVELOP  ///< Uncomment to enable experimental features

// ##############################################################################################
// TwoWire class:

/**
 * @class TwoWire
 * @brief High-level I2C communication wrapper class for STM32 HAL.
 *
 * This class abstracts I2C operations including master and slave communication modes,
 * and integrates error reporting and flexible initialization.
 * 
 * @note Default TX/RX communication mode is WIRE_MODE_BLOCK.
 */
class TwoWire 
{
    public:

        /**
         * @brief Last error code recorded during I2C communication.
         */
        uint8_t errorCode;

        /**
         * @brief Constructor.
         *
         * Initializes the TwoWire instance with a given I2C handle.
         *
         * @param i2cHandle Pointer to a pre-configured HAL I2C handle.
         *
         * @note It is recommended that:
         * - The I2C peripheral (I2C1, I2C2, etc.) be initialized before calling this constructor.
         * - GPIO pins for the I2C interface be configured before use.
         */
        TwoWire(I2C_HandleTypeDef *i2cHandle);
    
        /**
         * @brief Initializes the I2C interface in master mode.
         *
         * This method connects the controller (master) to the I2C bus.
         * It should normally be called once during setup.
         *
         * @return true if initialization succeeded, false otherwise.
         */
        bool begin();     

        /**
         * @brief Initializes the I2C interface in slave mode.
         *
         * Joins the I2C bus using the given 7-bit address. If no address is specified,
         * it acts as a controller (master).
         *
         * @param address 7-bit I2C address to use in slave mode.
         * @return true if initialization succeeded, false otherwise.
         */
        bool begin(uint8_t address);       

        /**
         * @brief Disables the I2C interface and releases associated resources.
         *
         * Stops the I2C operations and resets the internal state.
         *
         * @return true if deinitialization succeeded, false otherwise.
         */
        bool end(void);

        /**
         * @brief Set Clock speed of i2c prepheral.
         * @return true if succeeded.
         * @warning Use this function before begin method.
         */
        bool setClock(uint32_t clock);

        /**
         * @brief Set I2C transmit mode that can be Block mode, Interrupt mode, DMA mode.
         * @param mode: Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         * @warning TxMode should be set before calling the begin() method.
         */
        bool setTxMode(uint8_t mode);

        /**
         * @brief Set I2C receive mode that can be Block mode, Interrupt mode, DMA mode.
         * @param mode: Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         * @warning RxMode should be set before calling the begin() method.
         */
        bool setRxMode(uint8_t mode);

        /**
         * @brief Sets the timeout for Wire transmissions in master mode.
         * @param timeout a timeout: timeout in milliseconds, if zero then timeout checking is disabled
         * @param resetWithTimeout: if true then Wire hardware will be automatically reset on timeout
         * @note these timeouts are almost always an indication of an underlying problem, such as misbehaving devices, noise, 
         * insufficient shielding, or other electrical problems. These timeouts will prevent your sketch from locking up, 
         * but not solve these problems. In such situations there will often (also) be data corruption which doesn’t result 
         * in a timeout or other error and remains undetected. So when a timeout happens, 
         * it is likely that some data previously read or written is also corrupted. 
         * Additional measures might be needed to more reliably detect such issues (e.g. checksums or reading back written values) 
         * and recover from them (e.g. full system reset). This timeout and such additional measures should be seen as a last line 
         * of defence, when possible the underlying cause should be fixed instead.
         */
        void setWireTimeout(uint32_t timeout, bool resetWithTimeout = false);

        /**
         * @brief Get wire timeout flag.
         */
        bool getWireTimeoutFlag(void);

        /**
         * @brief This function begins a transmission to the I2C peripheral device with the given address. 
         * Subsequently, queue bytes for transmission with the write() function and transmit them by calling endTransmission().
         * @param address: the 7-bit address of the device to transmit to.
         * @return true if succeeded.
         * @return False if the previous transmission has not completed yet.
         *  */                 
        bool beginTransmission(uint8_t address); 

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
        uint8_t write(const uint8_t* data, uint8_t quantity); 

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
         * @brief Request data from an I2C device.
         * @param address: the 7-bit slave address of the device to request bytes from.
         * @param quantity: the number of bytes to request.
         * @note This function is used by the controller device to request bytes from a peripheral device. 
         * The bytes may then be retrieved with the available() and read() functions. 
         * @return uint8_t: the number of bytes returned from the peripheral device.
         */
        uint8_t requestFrom(uint8_t address, uint8_t quantity); 

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
        bool recovery(void);  
        
        /**
         * @brief Recovers the I2C bus by manually toggling the SCL line and generating a STOP condition.
         *
         * This function is used when the I2C bus becomes stuck, for example, if a slave device
         * (e.g., an EEPROM) holds the SDA line low due to an incomplete transaction or error.
         * It temporarily reconfigures the SCL and SDA pins as GPIO open-drain outputs, sends
         * up to 9 clock pulses on the SCL line to release the slave, and then generates a STOP
         * condition manually. Finally, it reinitializes the I2C peripheral.
         *
         * @note This function should be called when I2C communication fails due to a bus hang.
         *       It may be combined with additional recovery logic or delays depending on the
         *       specific slave devices used.
         *
         * @retval true  Bus recovery succeeded and I2C peripheral was reinitialized.
         * @retval false Recovery failed or I2C peripheral reinitialization error occurred.
         */
        bool busRecoveryGPIO(void);

        /**
         * @brief Clear timeout flag.
         */
        void clearWireTimeoutFlag(void);

        /**
         * @brief Interrupt callback function for masterTxCpltCallback
         * @note Use this function inside global I2C masterTxCpltCallback
         */
        void masterTxCpltCallback(void);

        /**
         * @brief Interrupt callback function for masterRxCpltCallback
         * @note Use this function inside global I2C masterRxCpltCallback
         */
        void masterRxCpltCallback(void);

        /**
         * @brief Interrupt callback function for slaveTxCpltCallback
         * @note Use this function inside global I2C slaveTxCpltCallback
         */
        void slaveTxCpltCallback(void);

        /**
         * @brief Interrupt callback function for slaveRxCpltCallback
         * @note Use this function inside global I2C slaveRxCpltCallback
         */
        void slaveRxCpltCallback(void);

        /**
         * @brief Interrupt callback function for slaveAddrCallback
         * @note Use this function inside global I2C slaveAddrCallback
         */
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

        /// @brief Return interrupt txCompleteFlag
        bool getTxCompleteFlag(void) { return _txCompleteFlag;};

        /// @brief Return interrupt rxCompleteFlag
        bool getRxCompleteFlag(void) { return _rxCompleteFlag;};

        /// @brief Return interrupt slaveTxCompleteFlag
        bool getSlaveTxCompleteFlag(void) { return _slaveTxCompleteFlag;};

        /// @brief Return interrupt slaveRxCompleteFlag
        bool getSlaveRxCompleteFlag(void) { return _slaveRxCompleteFlag;};

        /**
         * @brief This function is used to check if a slave device is available on the I2C bus before initiating communication. 
         * This helps prevent HAL_BUSY errors or failed transactions.
         */
        bool IsDeviceReady(uint8_t address);

        /**
         * @brief Check I2C State.
         */
        HAL_I2C_StateTypeDef GetState(void);

        void setGPIO(GPIO_TypeDef* SCLPort, uint16_t SCLPin, GPIO_TypeDef* SDAPort, uint16_t SDAPIn);

    private:

        GPIO_TypeDef* _SCL_PORT; 
        uint16_t _SCL_PIN;

        GPIO_TypeDef* _SDA_PORT; 
        uint16_t _SDA_PIN;

        /**
         * @brief HAL I2C handle pointer
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

        /// @brief Transmission status flag
        uint8_t _transmitting;     

        /// @brief The flag that enable/disable recovery i2c if timeout occured. A value of true means i2c recovery after timeout occurred.
        bool _resetWithTimeout;

        /// @brief The timeout value for i2c communication. [ms]
        uint32_t _timeout;

        /// @brief The timeout event flag.
        bool _timeoutFlag;

        /// @brief Address of the target device              
        uint8_t _txAddress;

        /**
         * @brief Slave address for I2C communication.
         */
        uint8_t _slaveAddress;

        /// @brief The flag for TX master interrupt
        volatile bool _txCompleteFlag;

        /// @brief The flag for RX master interrupt
        volatile bool _rxCompleteFlag; 

        /// @brief The flag for RX slave interrupt
        volatile bool _slaveRxCompleteFlag;

        /// @brief The flag for TX slave interrupt
        volatile bool _slaveTxCompleteFlag;

        /// @brief Slave data buffer
        uint8_t _slaveBuffer[WIRE_BUFFER_LENGTH]; 

        /**
         * @brief I2C transmit mode that can be Block mode, Interrupt mode, DMA mode.
         * @note - Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         */
        volatile uint8_t _txMode;

        /**
         * @brief I2C receive mode that can be Block mode, Interrupt mode, DMA mode.
         * @note - Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         */
        volatile uint8_t _rxMode;

        /// @brief Clears the send and receive buffers 
        void _clearBuffers();         
};




