cmake_minimum_required(VERSION 3.25)
project(crc16 LANGUAGES C)

add_library(crc16_Lib crc16.c)

set_property(TARGET crc16_Lib PROPERTY CXX_STANDARD 17)
set_property(TARGET crc16_Lib PROPERTY CXX_STANDARD_REQUIRED ON)
