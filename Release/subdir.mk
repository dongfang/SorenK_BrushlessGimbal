################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Calibration.cpp \
../Commands.cpp \
../Configuration.cpp \
../FastTask.cpp \
../I2C.cpp \
../IMU.cpp \
../MPU6050.cpp \
../Main.cpp \
../Mavlink.cpp \
../MediumTask.cpp \
../RCdecode.cpp \
../Scheduler.cpp \
../SerialCom.cpp \
../SerialCommand.cpp \
../SlowLoop.cpp \
../TuningMotion.cpp \
../UARTStream.cpp \
../Util.cpp 

OBJS += \
./Calibration.o \
./Commands.o \
./Configuration.o \
./FastTask.o \
./I2C.o \
./IMU.o \
./MPU6050.o \
./Main.o \
./Mavlink.o \
./MediumTask.o \
./RCdecode.o \
./Scheduler.o \
./SerialCom.o \
./SerialCommand.o \
./SlowLoop.o \
./TuningMotion.o \
./UARTStream.o \
./Util.o 

CPP_DEPS += \
./Calibration.d \
./Commands.d \
./Configuration.d \
./FastTask.d \
./I2C.d \
./IMU.d \
./MPU6050.d \
./Main.d \
./Mavlink.d \
./MediumTask.d \
./RCdecode.d \
./Scheduler.d \
./SerialCom.d \
./SerialCommand.d \
./SlowLoop.d \
./TuningMotion.d \
./UARTStream.d \
./Util.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -Wall -Os -fpack-struct -fshort-enums -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


