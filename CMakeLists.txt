cmake_minimum_required(VERSION 3.17)

set(SOURCES
        src/motor/BLDCMotor.cpp
        src/motor/BaseMotor.cpp
        src/motor/StepperMotor.cpp

        src/driver/BLDCDriver3PWM.cpp
        src/driver/BLDCDriver6PWM.cpp
        src/driver/StepperDriver2PWM.cpp
        src/driver/StepperDriver4PWM.cpp

        src/encoder/BaseEncoder.cpp

        src/current_sensor/BaseCurrentSensor.cpp
        src/current_sensor/InlineCurrentSensor.cpp
        src/current_sensor/LowSideCurrentSensor.cpp
)

set(COMPONENTS

)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS include REQUIRES ${COMPONENTS})
