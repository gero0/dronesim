cmake_minimum_required(VERSION 3.25)
project(Message LANGUAGES C)

add_library(Message_Lib Src/message.c)

set_property(TARGET Message_Lib PROPERTY CXX_STANDARD 17)
set_property(TARGET Message_Lib PROPERTY CXX_STANDARD_REQUIRED ON)
