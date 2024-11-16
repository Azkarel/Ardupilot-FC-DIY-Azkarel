# Ardupilot-FC-DIY-Azkarel

DIY Flight Controller for Ardupilot based on STM32H743 developer board

## Features

 - STM32H743 microcontroller
 - Two IMUs: MPU-9250, ICM20948
 - Two Magnetometers included in IMUs: AK8963, AK09916
 - Two Barometers: BMP-280, BMP-388
 - Internal vibration isolation for IMUs
 - Internal RGB LED
 - MicroSD card slot port
 - 1 USB port
 - 4 UARTs ports
 - 2 I2C and 1 CAN ports
 - 8 PWM output ports
 - Safety switch port
 - Buzzer port
 - RC IN port
 - SBUS/DSM port

## Pinout
![AzkarelFC_Board](Azkarel_Pinout.jpg "Azkarel_Pinout")

## Connectors

**GPS1**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |   RX    | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   |   TX    | +3.3V |

**GPS2**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |   RX    | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   |   TX    | +3.3V |

**HEATER**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |HEATER_EN| +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   |   -     |  -    |

**BUZZER**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   |   5V   |  +5V  |
|  2   |   GND  |  GND  |
|  3   | BUZZER | +3.3V |

**I2C2**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   | I2C_SCL | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   | I2C_SDA | +3.3V |

**RCINT**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |  RC_INT | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   |    -    |   -   |

**PWM 1 to 8**

| Pin  |  Signal  | Volt  |
| :--: | :------: | :---: |
|  1   |  SIGNAL  | +3.3V |
|  2   |   VCC    |  +5V  |
|  3   |   GND    |  GND  |

**ADC_6V6**

| Pin  |  Signal  | Volt  |
| :--: | :------: | :---: |
|  1   |   GND    |  GND  |
|  2   |  SIGNAL  | +6.6V |

**ADC_3V3**

| Pin  |  Signal  | Volt  |
| :--: | :------: | :---: |
|  1   |   GND    |  GND  |
|  2   |  SIGNAL  | +3.3V |

**RSSI_IN**

| Pin  |  Signal  | Volt  |
| :--: | :------: | :---: |
|  1   |   GND    |  GND  |
|  2   |  SIGNAL  | +3.3V |

**SAFETY_LED**

| Pin  |    Signal     | Volt  |
| :--: | :-----------: | :---: |
|  1   |    5V_OUT     |  +5V  |
|  2   |      GND      |  GND  |
|  3   | SAFETY_SW_LED | +3.3V |

**SAFETY_SW**

| Pin  |    Signal     | Volt  |
| :--: | :-----------: | :---: |
|  1   |    5V_OUT     |  +5V  |
|  2   |      GND      |  GND  |
|  3   |   SAFETY_SW   | +3.3V |

**TELEM1**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |   RX    | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   |   TX    | +3.3V |

**TELEM2**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |   RX    | +3.3V |
|  2   |   TX    | +3.3V |
|  3   |   5V    |  +5V  |
|  4   |   GND   |  GND  |
|  5   |   RTS   | +3.3V |
|  6   |   CTS   | +3.3V |

**BATTERY_VOLTAGE**

| Pin  |     Signal      | Volt  |
| :--: | :-------------: | :---: |
|  1   |       GND       |  GND  |
|  2   | BAT_VOLTAGE_ADC | +3.3V |

**BATTERY_CURRENT**

| Pin  |     Signal      | Volt  |
| :--: | :-------------: | :---: |
|  1   |       GND       |  GND  |
|  2   | BAT_CRRENT_ADC  | +3.3V |

**I2C4**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   | I2C_SCL | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   | I2C_SDA | +3.3V |

**SBUS/DSM**

| Pin  | Signal  | Volt  |
| :--: | :-----: | :---: |
|  1   |   RX    | +3.3V |
|  2   |   5V    |  +5V  |
|  3   |   GND   |  GND  |
|  4   |   TX    | +3.3V |

**USB EX**

| Pin  | Signal | Volt  |
| :--: | :----: | :---: |
|  1   | VCC_IN |  +5V  |
|  2   |   DM   | +3.3V |
|  3   |   DP   | +3.3V |
|  4   |  GND   |  GND  |

## UART Mapping

 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(Telem1)
 - SERIAL2 -> USART2 (Telem2)
 - SERIAL3 -> USART3 (GPS1), NODMA
 - SERIAL4 -> UART5 (GPS2), NODMA
 - SERIAL5 -> UART8 (SBUS)
 - SERIAL6 -> UART7 (Debug), NODMA
 - SERIAL7 -> USB2(OTG2)

## RC Input

The remote control signal should be connected to the “RC IN” pin, in their specific connector.

This signal pin supports two types of remote control signal inputs, SBUS and PPM signals.

## PWM Output

The flight controller supports up to 8 PWM outputs, support all PWM protocols as well as DShot. All 8 PWM outputs have GND on the bottom row, 5V on the middle row and signal on the top row. 5V pins serves as feeding voltage for the FC from motor ESCs.

The 8 PWM outputs are in 2 groups:

 - PWM 1, 2, 3 and 4 in group1
 - PWM 5, 6, 7 and 8 in group2

Channels 1, 3, 5, 7 support bi-directional Dshot, channels 2, 4, 6, 8 support Dshot.

## Battery Monitoring

The flight controller has two connectors to monitorize voltage and current from battery.

## GPIOs

All 8 PWM channels can be used for GPIO functions (relays, buttons, RPM etc).

The pin numbers for these PWM channels in ArduPilot are shown below:

| PWM Channels | Pin  | PWM Channels | Pin  |
| ------------ | ---- | ------------ | ---- |
| PWM1         | 50   | PWM5         | 54   |
| PWM2         | 51   | PWM6         | 55   |
| PWM3         | 52   | PWM7         | 56   |
| PWM4         | 53   | PWM8         | 67   |

## Analog inputs

The flight controller has 5 analog inputs

 - ADC 5V Sense 
 - ADC Battery Current 
 - ADC Battery Voltage 
 - ADC 3V3 Sense
 - ADC 6V6 Sense
 - RSSI voltage monitoring

