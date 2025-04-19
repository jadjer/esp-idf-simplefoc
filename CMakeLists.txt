cmake_minimum_required(VERSION 3.30.5)

set(SOURCES
        src/CurrentSensorBase.cpp
        src/DriverBase.cpp
        src/EncoderBase.cpp
        src/LowPassFilter.cpp
        src/MotorBase.cpp
)

set(COMPONENTS
        esp_timer
        pid
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS include REQUIRES ${COMPONENTS})
