BOARD_TAG		= uno
ARDUINO_DIR		= /opt/arduino/arduino
MONITOR_PORT	= /dev/ttyUSB0

ARDUINO_LIBS	= TimerOne NewPing LiquidCrystal_I2C AccelStepper Wire

include /opt/Arduino-Makefile/Arduino.mk

