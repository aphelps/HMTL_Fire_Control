// No-op SerialCLI stub for native tests.
#pragma once
#include "Arduino.h"

class SerialCLI {
public:
    SerialCLI(const char *, void (*)(const char *)) {}
    void run() {}
};
