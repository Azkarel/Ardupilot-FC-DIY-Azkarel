/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MHZ (1000U*1000U)
#define KHZ (1000U)


#define AP_SIGNED_FIRMWARE 0

#define HAL_ENABLE_DFU_BOOT FALSE
#define CHIBIOS_BOARD_NAME "Azkarel"
// MCU type (ChibiOS define)
#define STM32H7xx_MCUCONF
#define STM32H743_MCUCONF

#define STM32H743xx

// crystal frequency
#define STM32_HSECLK 25000000U

// UART used for stdout (printf)
// SDMMC available, enable POSIX filesystem support
#define USE_POSIX

#define HAL_USE_SDC TRUE
#define STM32_SDC_USE_SDMMC1 TRUE
#define HAL_USE_SDMMC 1
#define STM32_USB_USE_OTG1                  TRUE
#define STM32_OTG2_IS_OTG1                  FALSE
#define HAL_USE_USB TRUE
#define HAL_USE_SERIAL_USB TRUE
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif
#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING 1
#endif
#ifndef STM32H7
#define STM32H7 1
#endif
#define HAL_USE_HW_RNG TRUE
#define HAL_PROCESS_STACK_SIZE 0x1C00
#define HAL_STORAGE_SIZE 32768
#define STM32_ST_USE_TIMER 5
#define ALLOW_ARM_NO_COMPASS
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#define HAL_I2C_INTERNAL_MASK 0
#define HAL_COMPASS_AUTO_ROT_DEFAULT 2
#define HAL_HEATER_GPIO_PIN 80
#define HAL_HAVE_IMU_HEATER 0
#define AP_NOTIFY_GPIO_LED_RGB_ENABLED 1
#define AP_NOTIFY_GPIO_LED_RGB_RED_PIN 0
#define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN 1
#define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN 2
#define HAL_HAVE_SAFETY_SWITCH 1
#define HAL_OS_FATFS_IO 1
#define BOARD_FLASH_SIZE 2048

// location of loaded firmware
#define FLASH_LOAD_ADDRESS 0x08020000
#define EXT_FLASH_SIZE_MB 0
#define EXT_FLASH_RESERVE_START_KB 0
#define EXT_FLASH_RESERVE_END_KB 0
#define CRT0_AREAS_NUMBER 1
#define __EXTFLASHFUNC__
#ifndef AP_CRASHDUMP_ENABLED
#define AP_CRASHDUMP_ENABLED 1
#endif

#define HAL_RAM0_START 0x20000000
// memory regions
#define HAL_MEMORY_REGIONS {(void*)0x30000000, 0x00040000, 0x08 }, {(void*)0x20000000, 0x00020000, 0x02 }, {(void*)0x24000000, 0x00080000, 0x04 }, {(void*)0x00000400, 0x0000fc00, 0x02 }, {(void*)0x30040000, 0x00008000, 0x08 }, {(void*)0x38000000, 0x00010000, 0x01 }
#define HAL_CC_MEMORY_REGIONS {0x30000000, 0x30040000, CRASH_CATCHER_BYTE }, {0x20000000, 0x20020000, CRASH_CATCHER_BYTE }, {0x24000000, 0x24080000, CRASH_CATCHER_BYTE }, {0x00000400, 0x00010000, CRASH_CATCHER_BYTE }, {0x30040000, 0x30048000, CRASH_CATCHER_BYTE }, {0x38000000, 0x38010000, CRASH_CATCHER_BYTE }
#define HAL_MEMORY_TOTAL_KB 1055

// CPU serial number (12 bytes)
#define UDID_START UID_BASE


// APJ board ID (for bootloaders)
#define APJ_BOARD_ID 666

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif
#define HAL_EXPECTED_SYSCLOCK 400000000
#define HAL_EXPECTED_STM32_SYS_CK 400000000
#define HAL_EXPECTED_STM32_QSPICLK 200000000
#define HAL_EXPECTED_STM32_SDMMC1CLK 80000000
#define HAL_EXPECTED_STM32_SPI45CLK 100000000
#define HAL_EXPECTED_STM32_FDCANCLK 80000000
#define STM32_DMA_REQUIRED TRUE


#ifndef HAL_FLASH_PROTECTION
#define HAL_FLASH_PROTECTION 0
#endif
#define HAL_SPI1_CONFIG { &SPID1, 1, STM32_SPI_SPI1_DMA_STREAMS, PAL_LINE(GPIOA,5U) }
#define HAL_SPI4_CONFIG { &SPID4, 4, STM32_SPI_SPI4_DMA_STREAMS, PAL_LINE(GPIOE,12U) }
#define HAL_SPI_BUS_LIST HAL_SPI1_CONFIG,HAL_SPI4_CONFIG


// SPI device table
#define HAL_SPI_DEVICE0  SPIDesc("mpu9250"        ,  0,  1, PAL_LINE(GPIOE,10U), SPIDEV_MODE3,   2*MHZ,   4*MHZ)
#define HAL_SPI_DEVICE1  SPIDesc("icm20948"       ,  1,  3, PAL_LINE(GPIOC,13U), SPIDEV_MODE3,   2*MHZ,   4*MHZ)
#define HAL_SPI_DEVICE2  SPIDesc("bmp280"         ,  0,  2, PAL_LINE(GPIOE,9U) , SPIDEV_MODE3,   1*MHZ,   8*MHZ)
#define HAL_SPI_DEVICE3  SPIDesc("bmp388"         ,  1,  4, PAL_LINE(GPIOE,4U) , SPIDEV_MODE3,   5*MHZ,  10*MHZ)
#define HAL_SPI_DEVICE_LIST HAL_SPI_DEVICE0,HAL_SPI_DEVICE1,HAL_SPI_DEVICE2,HAL_SPI_DEVICE3

#define HAL_WITH_SPI_MPU9250 1
#define HAL_WITH_SPI_ICM20948 1
#define HAL_WITH_SPI_BMP280 1
#define HAL_WITH_SPI_BMP388 1

// ADC config
#define ANALOG_VCC_5V_PIN 4
#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_ANALOG_PINS \
{  4,  4,  2*3.30/4096 }, /* PC4 VDD_5V_SENS */ \
{  5,  5,    3.30/4096 }, /* PB1 SPARE2_ADC1 */ \
{  9,  9,  2*3.30/4096 }, /* PB0 SPARE1_ADC1 */ \
{ 10, 10,    3.30/4096 }, /* PC0 BATT_CURRENT_SENS */ \
{ 11, 11,    3.30/4096 }, /* PC1 BATT_VOLTAGE_SENS */ \
{ 18, 18,    3.30/4096 }, /* PA4 RSSI_IN */ \


// GPIO config
#define HAL_GPIO_LINE_GPIO0 PAL_LINE(GPIOD,15U)
#define HAL_GPIO_LINE_GPIO1 PAL_LINE(GPIOD,14U)
#define HAL_GPIO_LINE_GPIO2 PAL_LINE(GPIOC,6U)
#define HAL_GPIO_LINE_GPIO50 PAL_LINE(GPIOA,8U)
#define HAL_GPIO_LINE_GPIO51 PAL_LINE(GPIOE,11U)
#define HAL_GPIO_LINE_GPIO52 PAL_LINE(GPIOE,13U)
#define HAL_GPIO_LINE_GPIO53 PAL_LINE(GPIOE,14U)
#define HAL_GPIO_LINE_GPIO54 PAL_LINE(GPIOA,15U)
#define HAL_GPIO_LINE_GPIO55 PAL_LINE(GPIOB,3U)
#define HAL_GPIO_LINE_GPIO56 PAL_LINE(GPIOA,2U)
#define HAL_GPIO_LINE_GPIO57 PAL_LINE(GPIOA,3U)
#define HAL_GPIO_LINE_GPIO77 PAL_LINE(GPIOB,14U)
#define HAL_GPIO_LINE_GPIO80 PAL_LINE(GPIOD,10U)
#define HAL_GPIO_PINS { \
{   0, true,  0, PAL_LINE(GPIOD,15U)}, /* PD15 LED_R1 OUTPUT */ \
{   1, true,  0, PAL_LINE(GPIOD,14U)}, /* PD14 LED_G1 OUTPUT */ \
{   2, true,  0, PAL_LINE(GPIOC,6U)}, /* PC6 LED_B1 OUTPUT */ \
{  50, true,  1, PAL_LINE(GPIOA,8U)}, /* PA8 TIM1_CH1 TIM1 AF1 PWM1 */ \
{  51, true,  2, PAL_LINE(GPIOE,11U)}, /* PE11 TIM1_CH2 TIM1 AF1 PWM2 */ \
{  52, true,  3, PAL_LINE(GPIOE,13U)}, /* PE13 TIM1_CH3 TIM1 AF1 PWM3 */ \
{  53, true,  4, PAL_LINE(GPIOE,14U)}, /* PE14 TIM1_CH4 TIM1 AF1 PWM4 */ \
{  54, true,  5, PAL_LINE(GPIOA,15U)}, /* PA15 TIM2_CH1 TIM2 AF1 PWM5 */ \
{  55, true,  6, PAL_LINE(GPIOB,3U)}, /* PB3 TIM2_CH2 TIM2 AF1 PWM6 */ \
{  56, true,  7, PAL_LINE(GPIOA,2U)}, /* PA2 TIM2_CH3 TIM2 AF1 PWM7 */ \
{  57, true,  8, PAL_LINE(GPIOA,3U)}, /* PA3 TIM2_CH4 TIM2 AF1 PWM8 */ \
{  77, true,  0, PAL_LINE(GPIOB,14U)}, /* PB14 TIM12_CH1 TIM12 AF2 */ \
{  80, true,  0, PAL_LINE(GPIOD,10U)}, /* PD10 HEATER_EN OUTPUT */ \
}

// full pin define list
#define HAL_GPIO_PIN_BAROMETER1_CS        PAL_LINE(GPIOE,9U)
#define HAL_GPIO_PIN_BAROMETER2_CS        PAL_LINE(GPIOE,4U)
#define HAL_GPIO_PIN_BATT_CURRENT_SENS    PAL_LINE(GPIOC,0U)
#define HAL_GPIO_PIN_BATT_VOLTAGE_SENS    PAL_LINE(GPIOC,1U)
#define HAL_GPIO_PIN_COMPASS_CS           PAL_LINE(GPIOB,5U)
#define HAL_GPIO_PIN_EXT_CS               PAL_LINE(GPIOB,4U)
#define HAL_GPIO_PIN_HEATER_EN            PAL_LINE(GPIOD,10U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,10U)
#define HAL_GPIO_PIN_I2C2_SDA             PAL_LINE(GPIOB,11U)
#define HAL_GPIO_PIN_I2C2_SCL             PAL_LINE(GPIOB,10U)
#define HAL_GPIO_PIN_I2C4_SCL             PAL_LINE(GPIOB,8U)
#define HAL_GPIO_PIN_I2C4_SDA             PAL_LINE(GPIOB,7U)
#define HAL_GPIO_PIN_I2C4_SCL             PAL_LINE(GPIOB,8U)
#define HAL_GPIO_PIN_IMU1_CS              PAL_LINE(GPIOE,10U)
#define HAL_GPIO_PIN_IMU2_CS              PAL_LINE(GPIOC,13U)
#define HAL_GPIO_PIN_JTCK_SWCLK           PAL_LINE(GPIOA,14U)
#define HAL_GPIO_PIN_JTMS_SWDIO           PAL_LINE(GPIOA,13U)
#define HAL_GPIO_PIN_LED_B1               PAL_LINE(GPIOC,6U)
#define HAL_GPIO_PIN_LED_G1               PAL_LINE(GPIOD,14U)
#define HAL_GPIO_PIN_LED_R1               PAL_LINE(GPIOD,15U)
#define HAL_GPIO_PIN_LED_SAFETY           PAL_LINE(GPIOA,1U)
#define HAL_GPIO_PIN_OTG_FS_DM            PAL_LINE(GPIOA,11U)
#define HAL_GPIO_PIN_OTG_FS_DP            PAL_LINE(GPIOA,12U)
#define HAL_GPIO_PIN_RESERVE_CS           PAL_LINE(GPIOD,7U)
#define HAL_GPIO_PIN_RSSI_IN              PAL_LINE(GPIOA,4U)
#define HAL_GPIO_PIN_SAFETY_IN            PAL_LINE(GPIOC,3U)
#define HAL_GPIO_PIN_SDMMC1_CK            PAL_LINE(GPIOC,12U)
#define HAL_GPIO_PIN_SDMMC1_CMD           PAL_LINE(GPIOD,2U)
#define HAL_GPIO_PIN_SDMMC1_D0            PAL_LINE(GPIOC,8U)
#define HAL_GPIO_PIN_SDMMC1_D1            PAL_LINE(GPIOC,9U)
#define HAL_GPIO_PIN_SDMMC1_D2            PAL_LINE(GPIOC,10U)
#define HAL_GPIO_PIN_SDMMC1_D3            PAL_LINE(GPIOC,11U)
#define HAL_GPIO_PIN_SPARE1_ADC1          PAL_LINE(GPIOB,0U)
#define HAL_GPIO_PIN_SPARE2_ADC1          PAL_LINE(GPIOB,1U)
#define HAL_GPIO_PIN_SPI1_MISO            PAL_LINE(GPIOA,6U)
#define HAL_GPIO_PIN_SPI1_MOSI            PAL_LINE(GPIOA,7U)
#define HAL_GPIO_PIN_SPI1_SCK             PAL_LINE(GPIOA,5U)
#define HAL_GPIO_PIN_SPI4_MISO            PAL_LINE(GPIOE,5U)
#define HAL_GPIO_PIN_SPI4_MOSI            PAL_LINE(GPIOE,6U)
#define HAL_GPIO_PIN_SPI4_SCK             PAL_LINE(GPIOE,12U)
#define HAL_GPIO_PIN_TIM12_CH1            PAL_LINE(GPIOB,14U)
#define HAL_GPIO_PIN_TIM1_CH1             PAL_LINE(GPIOA,8U)
#define HAL_GPIO_PIN_TIM1_CH2             PAL_LINE(GPIOE,11U)
#define HAL_GPIO_PIN_TIM1_CH3             PAL_LINE(GPIOE,13U)
#define HAL_GPIO_PIN_TIM1_CH4             PAL_LINE(GPIOE,14U)
#define HAL_GPIO_PIN_TIM2_CH1             PAL_LINE(GPIOA,15U)
#define HAL_GPIO_PIN_TIM2_CH2             PAL_LINE(GPIOB,3U)
#define HAL_GPIO_PIN_TIM2_CH3             PAL_LINE(GPIOA,2U)
#define HAL_GPIO_PIN_TIM2_CH4             PAL_LINE(GPIOA,3U)
#define HAL_GPIO_PIN_TIM8_CH2             PAL_LINE(GPIOC,7U)
#define HAL_GPIO_PIN_UART5_RX             PAL_LINE(GPIOB,12U)
#define HAL_GPIO_PIN_UART5_TX             PAL_LINE(GPIOB,13U)
#define HAL_GPIO_PIN_UART7_RX             PAL_LINE(GPIOE,7U)
#define HAL_GPIO_PIN_UART7_TX             PAL_LINE(GPIOE,8U)
#define HAL_GPIO_PIN_UART8_RX             PAL_LINE(GPIOE,0U)
#define HAL_GPIO_PIN_UART8_TX             PAL_LINE(GPIOE,1U)
#define HAL_GPIO_PIN_USART1_RX            PAL_LINE(GPIOA,10U)
#define HAL_GPIO_PIN_USART1_TX            PAL_LINE(GPIOA,9U)
#define HAL_GPIO_PIN_USART2_CTS_NSS       PAL_LINE(GPIOA,0U)
#define HAL_GPIO_PIN_USART2_RTS           PAL_LINE(GPIOD,4U)
#define HAL_GPIO_PIN_USART2_RX            PAL_LINE(GPIOD,6U)
#define HAL_GPIO_PIN_USART2_TX            PAL_LINE(GPIOD,5U)
#define HAL_GPIO_PIN_USART3_RX            PAL_LINE(GPIOD,9U)
#define HAL_GPIO_PIN_USART3_TX            PAL_LINE(GPIOD,8U)
#define HAL_GPIO_PIN_VDD_3V3_SENSORS_EN   PAL_LINE(GPIOE,15U)
#define HAL_GPIO_PIN_VDD_5V_SENS          PAL_LINE(GPIOC,4U)

#define HAL_INS_PROBE1  ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this,hal.spi->get_device("mpu9250"),ROTATION_YAW_270))
#define HAL_INS_PROBE2  ADD_BACKEND(AP_InertialSensor_Invensensev2::probe(*this,hal.spi->get_device("icm20948"),ROTATION_YAW_270))
#define INS_MAX_INSTANCES 2
#define HAL_INS_PROBE_LIST HAL_INS_PROBE1;HAL_INS_PROBE2

#define HAL_MAG_PROBE1  ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0,ROTATION_PITCH_180))
#define HAL_MAG_PROBE2  ADD_BACKEND(DRIVER_AK09916, AP_Compass_AK09916::probe_ICM20948(0,ROTATION_ROLL_180_YAW_90))
#define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1;HAL_MAG_PROBE2

#define HAL_BARO_PROBE1  ADD_BACKEND(AP_Baro_BMP280::probe(*this,hal.spi->get_device("bmp280")))
#define HAL_BARO_PROBE2  ADD_BACKEND(AP_Baro_BMP388::probe(*this,hal.spi->get_device("bmp388")))
#define HAL_BARO_PROBE_LIST HAL_BARO_PROBE1;HAL_BARO_PROBE2


#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED 1
#endif
// peripherals enabled
#define STM32_I2C_USE_I2C2                  TRUE
#define STM32_I2C_USE_I2C4                  TRUE
#define STM32_SPI_USE_SPI1                  TRUE
#define STM32_SPI_USE_SPI4                  TRUE
#ifndef STM32_SERIAL_USE_UART5 
#define STM32_SERIAL_USE_UART5  TRUE
#endif
#ifndef STM32_SERIAL_USE_UART7 
#define STM32_SERIAL_USE_UART7  TRUE
#endif
#ifndef STM32_SERIAL_USE_UART8 
#define STM32_SERIAL_USE_UART8  TRUE
#endif
#ifndef STM32_SERIAL_USE_USART1
#define STM32_SERIAL_USE_USART1 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART2
#define STM32_SERIAL_USE_USART2 TRUE
#endif
#ifndef STM32_SERIAL_USE_USART3
#define STM32_SERIAL_USE_USART3 TRUE
#endif
#define AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED 1


// auto-generated DMA mapping from dma_resolver.py
#define STM32_ADC_ADC1_DMA_STREAM      STM32_DMA_STREAM_ID(2, 1)
#define STM32_ADC_ADC1_DMA_CHAN        STM32_DMAMUX1_ADC1
#define STM32_SPI_SPI1_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 0)
#define STM32_SPI_SPI1_RX_DMA_CHAN     STM32_DMAMUX1_SPI1_RX
#define STM32_SPI_SPI1_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 1)
#define STM32_SPI_SPI1_TX_DMA_CHAN     STM32_DMAMUX1_SPI1_TX
#define STM32_SPI_SPI4_RX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 2)
#define STM32_SPI_SPI4_RX_DMA_CHAN     STM32_DMAMUX1_SPI4_RX
#define STM32_SPI_SPI4_TX_DMA_STREAM   STM32_DMA_STREAM_ID(1, 3)
#define STM32_SPI_SPI4_TX_DMA_CHAN     STM32_DMAMUX1_SPI4_TX
#define STM32_TIM_TIM1_UP_DMA_STREAM   STM32_DMA_STREAM_ID(1, 4)
#define STM32_TIM_TIM1_UP_DMA_CHAN     STM32_DMAMUX1_TIM1_UP
#define STM32_TIM_TIM2_UP_DMA_STREAM   STM32_DMA_STREAM_ID(1, 5)
#define STM32_TIM_TIM2_UP_DMA_CHAN     STM32_DMAMUX1_TIM2_UP
#define STM32_UART_UART8_RX_DMA_STREAM STM32_DMA_STREAM_ID(2, 0)
#define STM32_UART_UART8_RX_DMA_CHAN   STM32_DMAMUX1_UART8_RX
#define STM32_UART_UART8_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 2)
#define STM32_UART_UART8_TX_DMA_CHAN   STM32_DMAMUX1_UART8_TX
#define STM32_UART_USART1_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 6)
#define STM32_UART_USART1_RX_DMA_CHAN  STM32_DMAMUX1_USART1_RX
#define STM32_UART_USART1_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 4)
#define STM32_UART_USART1_TX_DMA_CHAN  STM32_DMAMUX1_USART1_TX
#define STM32_UART_USART2_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 7)
#define STM32_UART_USART2_RX_DMA_CHAN  STM32_DMAMUX1_USART2_RX
#define STM32_UART_USART2_TX_DMA_STREAM STM32_DMA_STREAM_ID(2, 3)
#define STM32_UART_USART2_TX_DMA_CHAN  STM32_DMAMUX1_USART2_TX

// Mask of DMA streams which are shared
#define SHARED_DMA_MASK 0


// generated UART DMA configuration lines
#define STM32_USART1_RX_DMA_CONFIG true, STM32_UART_USART1_RX_DMA_STREAM, STM32_DMAMUX1_USART1_RX
#define STM32_USART1_TX_DMA_CONFIG true, STM32_UART_USART1_TX_DMA_STREAM, STM32_DMAMUX1_USART1_TX
#define STM32_USART2_RX_DMA_CONFIG true, STM32_UART_USART2_RX_DMA_STREAM, STM32_DMAMUX1_USART2_RX
#define STM32_USART2_TX_DMA_CONFIG true, STM32_UART_USART2_TX_DMA_STREAM, STM32_DMAMUX1_USART2_TX
#define STM32_USART3_RX_DMA_CONFIG false, 0, 0
#define STM32_USART3_TX_DMA_CONFIG false, 0, 0
#define STM32_UART5_RX_DMA_CONFIG false, 0, 0
#define STM32_UART5_TX_DMA_CONFIG false, 0, 0
#define STM32_UART7_RX_DMA_CONFIG false, 0, 0
#define STM32_UART7_TX_DMA_CONFIG false, 0, 0
#define STM32_UART8_RX_DMA_CONFIG true, STM32_UART_UART8_RX_DMA_STREAM, STM32_DMAMUX1_UART8_RX
#define STM32_UART8_TX_DMA_CONFIG true, STM32_UART_UART8_TX_DMA_STREAM, STM32_DMAMUX1_UART8_TX


// generated SPI DMA configuration lines
#define STM32_SPI_SPI1_DMA_STREAMS STM32_SPI_SPI1_TX_DMA_STREAM, STM32_SPI_SPI1_RX_DMA_STREAM
#define STM32_SPI_SPI4_DMA_STREAMS STM32_SPI_SPI4_TX_DMA_STREAM, STM32_SPI_SPI4_RX_DMA_STREAM
#define HAL_PWM_COUNT 8
// RC input config
#define HAL_USE_EICU TRUE
#define STM32_EICU_USE_TIM8 TRUE
#define RCININT_EICU_TIMER EICUD8
#define RCININT_EICU_CHANNEL EICU_CHANNEL_2


// Alarm PWM output config
#define STM32_PWM_USE_TIM12 TRUE
#define STM32_TIM12_SUPPRESS_ISR
#define HAL_PWM_ALARM \
        { /* pwmGroup */ \
          0,  /* Timer channel */ \
          { /* PWMConfig */ \
            1000000,    /* PWM clock frequency. */ \
            1000,    /* Initial PWM period 20ms. */ \
            NULL,  /* no callback */ \
            { /* Channel Config */ \
             {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}  \
            }, \
            0, 0 \
          }, \
          &PWMD12 /* PWMDriver* */ \
        }

// PWM timer config
#define STM32_PWM_USE_TIM1 TRUE
#define STM32_TIM1_SUPPRESS_ISR
#define STM32_PWM_USE_TIM2 TRUE
#define STM32_TIM2_SUPPRESS_ISR

// PWM output config
#if defined(STM32_TIM_TIM1_UP_DMA_STREAM) && defined(STM32_TIM_TIM1_UP_DMA_CHAN)
# define HAL_PWM1_DMA_CONFIG true, STM32_TIM_TIM1_UP_DMA_STREAM, STM32_TIM_TIM1_UP_DMA_CHAN
#else
# define HAL_PWM1_DMA_CONFIG false, 0, 0
#endif
#if !defined(HAL_TIM1_UP_SHARED)
#define HAL_TIM1_UP_SHARED false
#endif
#define HAL_PWM_GROUP1 { true, \
        {0, 1, 2, 3}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD1, 1, \
          HAL_PWM1_DMA_CONFIG, \
          { 1, 1, 1, 1 }, \
          { PAL_LINE(GPIOA,8U), PAL_LINE(GPIOE,11U), PAL_LINE(GPIOE,13U), PAL_LINE(GPIOE,14U) }}
#if defined(STM32_TIM_TIM2_UP_DMA_STREAM) && defined(STM32_TIM_TIM2_UP_DMA_CHAN)
# define HAL_PWM2_DMA_CONFIG true, STM32_TIM_TIM2_UP_DMA_STREAM, STM32_TIM_TIM2_UP_DMA_CHAN
#else
# define HAL_PWM2_DMA_CONFIG false, 0, 0
#endif
#if !defined(HAL_TIM2_UP_SHARED)
#define HAL_TIM2_UP_SHARED false
#endif
#define HAL_PWM_GROUP2 { false, \
        {4, 5, 6, 7}, \
        /* Group Initial Config */ \
        { \
          1000000,  /* PWM clock frequency. */ \
          20000,   /* Initial PWM period 20ms. */ \
          NULL,     /* no callback */ \
          { \
           /* Channel Config */ \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}  \
          }, 0, 0}, &PWMD2, 2, \
          HAL_PWM2_DMA_CONFIG, \
          { 1, 1, 1, 1 }, \
          { PAL_LINE(GPIOA,15U), PAL_LINE(GPIOB,3U), PAL_LINE(GPIOA,2U), PAL_LINE(GPIOA,3U) }}
#define HAL_PWM_GROUPS HAL_PWM_GROUP1,HAL_PWM_GROUP2

#define STM32_PWM_USE_ADVANCED TRUE
// I2C configuration

#if defined(STM32_I2C_I2C2_RX_DMA_STREAM) && defined(STM32_I2C_I2C2_TX_DMA_STREAM)
#define HAL_I2C2_CONFIG { &I2CD2, 2, STM32_I2C_I2C2_RX_DMA_STREAM, STM32_I2C_I2C2_TX_DMA_STREAM, PAL_LINE(GPIOB,10U), PAL_LINE(GPIOB,11U) }
#else
#define HAL_I2C2_CONFIG { &I2CD2, 2, SHARED_DMA_NONE, SHARED_DMA_NONE, PAL_LINE(GPIOB,10U), PAL_LINE(GPIOB,11U) }
#endif

#if defined(STM32_I2C_I2C4_RX_DMA_STREAM) && defined(STM32_I2C_I2C4_TX_DMA_STREAM)
#define HAL_I2C4_CONFIG { &I2CD4, 4, STM32_I2C_I2C4_RX_DMA_STREAM, STM32_I2C_I2C4_TX_DMA_STREAM, PAL_LINE(GPIOB,8U), PAL_LINE(GPIOB,7U) }
#else
#define HAL_I2C4_CONFIG { &I2CD4, 4, SHARED_DMA_NONE, SHARED_DMA_NONE, PAL_LINE(GPIOB,8U), PAL_LINE(GPIOB,7U) }
#endif

#define HAL_I2C_DEVICE_LIST HAL_I2C2_CONFIG,HAL_I2C4_CONFIG


// UART configuration
#define HAL_HAVE_SERIAL0 1
#define HAL_HAVE_SERIAL1 1
#define HAL_HAVE_SERIAL2 1
#define HAL_HAVE_SERIAL3 1
#define HAL_HAVE_SERIAL4 1
#define HAL_HAVE_SERIAL5 1
#define HAL_HAVE_SERIAL6 1
#define HAL_NUM_SERIAL_PORTS 7
#define HAL_SERIAL0_DRIVER ChibiOS::UARTDriver serial0Driver(0)
#define HAL_SERIAL1_DRIVER ChibiOS::UARTDriver serial1Driver(1)
#define HAL_SERIAL2_DRIVER ChibiOS::UARTDriver serial2Driver(2)
#define HAL_SERIAL3_DRIVER ChibiOS::UARTDriver serial3Driver(3)
#define HAL_SERIAL4_DRIVER ChibiOS::UARTDriver serial4Driver(4)
#define HAL_SERIAL5_DRIVER ChibiOS::UARTDriver serial5Driver(5)
#define HAL_SERIAL6_DRIVER ChibiOS::UARTDriver serial6Driver(6)
#define HAL_SERIAL7_DRIVER Empty::UARTDriver serial7Driver
#define HAL_SERIAL8_DRIVER Empty::UARTDriver serial8Driver
#define HAL_SERIAL9_DRIVER Empty::UARTDriver serial9Driver
#define HAL_WITH_IO_MCU 0

#define HAL_CRASH_SERIAL_PORT USART2
#define IRQ_DISABLE_HAL_CRASH_SERIAL_PORT() nvicDisableVector(STM32_USART2_NUMBER)
#define RCC_RESET_HAL_CRASH_SERIAL_PORT() rccResetUSART2(); rccEnableUSART2(true)
#define HAL_CRASH_SERIAL_PORT_CLOCK STM32_USART2CLK
#define HAL_OTG1_CONFIG {(BaseSequentialStream*) &SDU1, 1, true, false, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, UINT8_MAX,}
#define HAL_USART1_CONFIG { (BaseSequentialStream*) &SD1, 1, false, STM32_USART1_RX_DMA_CONFIG, STM32_USART1_TX_DMA_CONFIG, PAL_LINE(GPIOA,9U), PAL_LINE(GPIOA,10U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_HAVE_RTSCTS_SERIAL2
#define HAL_USART2_CONFIG { (BaseSequentialStream*) &SD2, 2, false, STM32_USART2_RX_DMA_CONFIG, STM32_USART2_TX_DMA_CONFIG, PAL_LINE(GPIOD,5U), PAL_LINE(GPIOD,6U), PAL_LINE(GPIOD,4U), 0, -1, 0, -1, 0, 0, 7}
#define HAL_USART3_CONFIG { (BaseSequentialStream*) &SD3, 3, false, STM32_USART3_RX_DMA_CONFIG, STM32_USART3_TX_DMA_CONFIG, PAL_LINE(GPIOD,8U), PAL_LINE(GPIOD,9U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_UART5_CONFIG { (BaseSequentialStream*) &SD5, 5, false, STM32_UART5_RX_DMA_CONFIG, STM32_UART5_TX_DMA_CONFIG, PAL_LINE(GPIOB,13U), PAL_LINE(GPIOB,12U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_UART7_CONFIG { (BaseSequentialStream*) &SD7, 7, false, STM32_UART7_RX_DMA_CONFIG, STM32_UART7_TX_DMA_CONFIG, PAL_LINE(GPIOE,8U), PAL_LINE(GPIOE,7U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define HAL_UART8_CONFIG { (BaseSequentialStream*) &SD8, 8, false, STM32_UART8_RX_DMA_CONFIG, STM32_UART8_TX_DMA_CONFIG, PAL_LINE(GPIOE,1U), PAL_LINE(GPIOE,0U), 0, 0, -1, 0, -1, 0, 0, UINT8_MAX}
#define AP_FEATURE_RTSCTS 1
#define HAL_SERIAL_DEVICE_LIST HAL_OTG1_CONFIG,HAL_USART1_CONFIG,HAL_USART2_CONFIG,HAL_USART3_CONFIG,HAL_UART5_CONFIG,HAL_UART7_CONFIG,HAL_UART8_CONFIG

#define HAL_UART_NUM_SERIAL_PORTS 7
// USB configuration
#define HAL_USB_VENDOR_ID 0x1209
#define HAL_USB_PRODUCT_ID 0x5741
#define HAL_USB_STRING_MANUFACTURER "ArduPilot"
#define HAL_USB_STRING_PRODUCT "%BOARD%"
#define HAL_USB_STRING_SERIAL "%SERIAL%"


#define AP_BOOTLOADER_FLASHING_ENABLED 1

/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/* PORTA:
 PA0 USART2_CTS_NSS USART2 AF7
 PA1 LED_SAFETY OUTPUT
 PA2 TIM2_CH3 TIM2 AF1 PWM7
 PA3 TIM2_CH4 TIM2 AF1 PWM8
 PA4 RSSI_IN ADC1 ADC1_IN18
 PA5 SPI1_SCK SPI1 AF5
 PA6 SPI1_MISO SPI1 AF5
 PA7 SPI1_MOSI SPI1 AF5
 PA8 TIM1_CH1 TIM1 AF1 PWM1
 PA9 USART1_TX USART1 AF7
 PA10 USART1_RX USART1 AF7
 PA11 OTG_FS_DM OTG1 AF10
 PA12 OTG_FS_DP OTG1 AF10
 PA13 JTMS-SWDIO SWD AF0
 PA14 JTCK-SWCLK SWD AF0
 PA15 TIM2_CH1 TIM2 AF1 PWM5
*/

#define VAL_GPIOA_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_OUTPUT(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_ANALOG(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_ALTERNATE(15U))

#define VAL_GPIOA_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOA_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOA_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_PULLDOWN(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOA_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOA_AFRL    (PIN_AFIO_AF(0U, 7U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 1U) | \
                           PIN_AFIO_AF(3U, 1U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 5U) | \
                           PIN_AFIO_AF(6U, 5U) | \
                           PIN_AFIO_AF(7U, 5U))

#define VAL_GPIOA_AFRH    (PIN_AFIO_AF(8U, 1U) | \
                           PIN_AFIO_AF(9U, 7U) | \
                           PIN_AFIO_AF(10U, 7U) | \
                           PIN_AFIO_AF(11U, 10U) | \
                           PIN_AFIO_AF(12U, 10U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 1U))

/* PORTB:
 PB0 SPARE1_ADC1 ADC1 ADC1_IN9
 PB1 SPARE2_ADC1 ADC1 ADC1_IN5
 PB3 TIM2_CH2 TIM2 AF1 PWM6
 PB4 EXT_CS CS
 PB5 COMPASS_CS CS
 PB7 I2C4_SDA I2C4 AF6
 PB8 I2C4_SCL I2C4 AF6
 PB10 I2C2_SCL I2C2 AF4
 PB11 I2C2_SDA I2C2 AF4
 PB12 UART5_RX UART5 AF14
 PB13 UART5_TX UART5 AF14
 PB14 TIM12_CH1 TIM12 AF2
*/

#define VAL_GPIOB_MODER   (PIN_MODE_ANALOG(0U) | \
                           PIN_MODE_ANALOG(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_ALTERNATE(3U) | \
                           PIN_MODE_OUTPUT(4U) | \
                           PIN_MODE_OUTPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOB_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_OPENDRAIN(7U) | \
                           PIN_OTYPE_OPENDRAIN(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_OPENDRAIN(10U) | \
                           PIN_OTYPE_OPENDRAIN(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOB_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOB_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_PULLUP(4U) | \
                           PIN_PUPDR_PULLUP(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_PULLUP(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOB_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOB_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 1U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 6U))

#define VAL_GPIOB_AFRH    (PIN_AFIO_AF(8U, 6U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 4U) | \
                           PIN_AFIO_AF(11U, 4U) | \
                           PIN_AFIO_AF(12U, 14U) | \
                           PIN_AFIO_AF(13U, 14U) | \
                           PIN_AFIO_AF(14U, 2U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTC:
 PC0 BATT_CURRENT_SENS ADC1 ADC1_IN10
 PC1 BATT_VOLTAGE_SENS ADC1 ADC1_IN11
 PC3 SAFETY_IN INPUT
 PC4 VDD_5V_SENS ADC1 ADC1_IN4
 PC6 LED_B1 OUTPUT
 PC7 TIM8_CH2 TIM8 AF3
 PC8 SDMMC1_D0 SDMMC1 AF12
 PC9 SDMMC1_D1 SDMMC1 AF12
 PC10 SDMMC1_D2 SDMMC1 AF12
 PC11 SDMMC1_D3 SDMMC1 AF12
 PC12 SDMMC1_CK SDMMC1 AF12
 PC13 IMU2_CS CS
*/

#define VAL_GPIOC_MODER   (PIN_MODE_ANALOG(0U) | \
                           PIN_MODE_ANALOG(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_ANALOG(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_OUTPUT(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_ALTERNATE(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_OUTPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOC_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_OPENDRAIN(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOC_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOC_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_PULLDOWN(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_PULLUP(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_PULLUP(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOC_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_LOW(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOC_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 3U))

#define VAL_GPIOC_AFRH    (PIN_AFIO_AF(8U, 12U) | \
                           PIN_AFIO_AF(9U, 12U) | \
                           PIN_AFIO_AF(10U, 12U) | \
                           PIN_AFIO_AF(11U, 12U) | \
                           PIN_AFIO_AF(12U, 12U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTD:
 PD2 SDMMC1_CMD SDMMC1 AF12
 PD4 USART2_RTS USART2
 PD5 USART2_TX USART2 AF7
 PD6 USART2_RX USART2 AF7
 PD7 RESERVE_CS CS
 PD8 USART3_TX USART3 AF7
 PD9 USART3_RX USART3 AF7
 PD10 HEATER_EN OUTPUT
 PD14 LED_G1 OUTPUT
 PD15 LED_R1 OUTPUT
*/

#define VAL_GPIOD_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_ALTERNATE(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_OUTPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_OUTPUT(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_ALTERNATE(9U) | \
                           PIN_MODE_OUTPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_OUTPUT(14U) | \
                           PIN_MODE_OUTPUT(15U))

#define VAL_GPIOD_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_OPENDRAIN(14U) | \
                           PIN_OTYPE_OPENDRAIN(15U))

#define VAL_GPIOD_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOD_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_PULLUP(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_PULLDOWN(4U) | \
                           PIN_PUPDR_PULLUP(5U) | \
                           PIN_PUPDR_PULLUP(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_PULLUP(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOD_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_LOW(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOD_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 12U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 7U) | \
                           PIN_AFIO_AF(6U, 7U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOD_AFRH    (PIN_AFIO_AF(8U, 7U) | \
                           PIN_AFIO_AF(9U, 7U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTE:
 PE0 UART8_RX UART8 AF8
 PE1 UART8_TX UART8 AF8
 PE4 BAROMETER2_CS CS
 PE5 SPI4_MISO SPI4 AF5
 PE6 SPI4_MOSI SPI4 AF5
 PE7 UART7_RX UART7 AF7
 PE8 UART7_TX UART7 AF7
 PE9 BAROMETER1_CS CS
 PE10 IMU1_CS CS
 PE11 TIM1_CH2 TIM1 AF1 PWM2
 PE12 SPI4_SCK SPI4 AF5
 PE13 TIM1_CH3 TIM1 AF1 PWM3
 PE14 TIM1_CH4 TIM1 AF1 PWM4
 PE15 VDD_3V3_SENSORS_EN OUTPUT
*/

#define VAL_GPIOE_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ALTERNATE(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_OUTPUT(4U) | \
                           PIN_MODE_ALTERNATE(5U) | \
                           PIN_MODE_ALTERNATE(6U) | \
                           PIN_MODE_ALTERNATE(7U) | \
                           PIN_MODE_ALTERNATE(8U) | \
                           PIN_MODE_OUTPUT(9U) | \
                           PIN_MODE_OUTPUT(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_OUTPUT(15U))

#define VAL_GPIOE_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOE_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOE_PUPDR   (PIN_PUPDR_PULLUP(0U) | \
                           PIN_PUPDR_PULLUP(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_PULLUP(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_PULLUP(7U) | \
                           PIN_PUPDR_PULLUP(8U) | \
                           PIN_PUPDR_PULLUP(9U) | \
                           PIN_PUPDR_PULLUP(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOE_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_LOW(15U))

#define VAL_GPIOE_AFRL    (PIN_AFIO_AF(0U, 8U) | \
                           PIN_AFIO_AF(1U, 8U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 5U) | \
                           PIN_AFIO_AF(6U, 5U) | \
                           PIN_AFIO_AF(7U, 7U))

#define VAL_GPIOE_AFRH    (PIN_AFIO_AF(8U, 7U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 1U) | \
                           PIN_AFIO_AF(12U, 5U) | \
                           PIN_AFIO_AF(13U, 1U) | \
                           PIN_AFIO_AF(14U, 1U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTF:
*/

#define VAL_GPIOF_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOF_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOF_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOF_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOF_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOF_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOF_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTG:
*/

#define VAL_GPIOG_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOG_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOG_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOG_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOG_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOG_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOG_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTH:
*/

#define VAL_GPIOH_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOH_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOH_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOH_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOH_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOH_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOH_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTI:
*/

#define VAL_GPIOI_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOI_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOI_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOI_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOI_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOI_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOI_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTJ:
*/

#define VAL_GPIOJ_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOJ_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOJ_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOJ_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOJ_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOJ_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOJ_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))

/* PORTK:
*/

#define VAL_GPIOK_MODER   (PIN_MODE_INPUT(0U) | \
                           PIN_MODE_INPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_INPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_INPUT(11U) | \
                           PIN_MODE_INPUT(12U) | \
                           PIN_MODE_INPUT(13U) | \
                           PIN_MODE_INPUT(14U) | \
                           PIN_MODE_INPUT(15U))

#define VAL_GPIOK_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_PUSHPULL(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U))

#define VAL_GPIOK_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U))

#define VAL_GPIOK_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_FLOATING(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_FLOATING(13U) | \
                           PIN_PUPDR_FLOATING(14U) | \
                           PIN_PUPDR_FLOATING(15U))

#define VAL_GPIOK_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_HIGH(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U))

#define VAL_GPIOK_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOK_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 0U) | \
                           PIN_AFIO_AF(12U, 0U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 0U))


// normal defaults

// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for "normal" builds - typically vehicle binaries but
// also examples.

#ifndef HAL_DSHOT_ALARM_ENABLED
#define HAL_DSHOT_ALARM_ENABLED (HAL_PWM_COUNT>0)
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#endif

// a similar define is present in AP_HAL_Boards.h:
#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef AP_TERRAIN_AVAILABLE
// enable terrain only if there's an SD card available:
#define AP_TERRAIN_AVAILABLE HAL_OS_FATFS_IO
#endif

#if AP_TERRAIN_AVAILABLE
#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif
#endif  // AP_TERRAIN_AVAILABLE


// end normal defaults
