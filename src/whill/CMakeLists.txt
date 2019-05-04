cmake_minimum_required(VERSION 2.8)
add_library(lib_whill STATIC
    Packet.cpp
    PacketParser.cpp
    PacketReceiver.cpp
    whill_commands.cpp
    WHILL.cpp
)