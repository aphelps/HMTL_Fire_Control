// Minimal Socket stub for native tests.
// Provides the Socket base class with inline definitions so no Socket.cpp
// needs to be compiled or linked.
#pragma once
#include "Arduino.h"

typedef uint16_t socket_addr_t;

#define SOCKET_ADDR_INVALID  0xFFFF
#define SOCKET_ADDR_ANY      0xFFFE

class Socket {
public:
    Socket() {}
    virtual ~Socket() {}
    virtual void     setup()                                              {}
    virtual boolean  initialized()                                        { return false; }
    virtual byte*    initBuffer(byte*, uint16_t)                          { return nullptr; }
    virtual void     sendMsgTo(uint16_t, const byte*, const byte)         {}
    virtual const byte* getMsg(unsigned int *retlen)                      { *retlen = 0; return nullptr; }
    virtual const byte* getMsg(uint16_t, unsigned int *retlen)            { *retlen = 0; return nullptr; }
    virtual byte     getLength()                                          { return 0; }
    virtual void*    headerFromData(const void*)                          { return nullptr; }
    virtual socket_addr_t sourceFromData(void*)                           { return 0; }
    virtual socket_addr_t destFromData(void*)                             { return 0; }
};
