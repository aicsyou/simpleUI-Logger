# simpleUI-Logger

A tiny, simple voltage/current logger with USB for DIYers.

This simple U/I logger is based on STM32F072 ARM-Cortex M0 MCU, which has one 12-bit ADC and crystal-less USB-PHY. It is designed to assist DIYers in testing battery powered devices.

The circuit is designed to meet the following specification:

Voltage Measurement: 

  - Range: 10V, 10mV resolution, +/-(1% + 10mV) accuracy
  
Current Measurement:

  - Range: 500mA, 5mA resolution, +/-(1% + 5mA) accuracy
  
  - Range: 5mA, 5uA resolution, +/-(1% + 5uA) accuracy
  
Operation Temperature:

  - 0 ~ 40 degC
  
The designed accuray can only be achieved with limited operation temperature and calibration. Drifting over time is possible.

Project Website:

www.sleeping-fish.cn
