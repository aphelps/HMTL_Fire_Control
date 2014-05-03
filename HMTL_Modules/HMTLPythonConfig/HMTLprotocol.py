#!/usr/local/bin/python3
#
# Definitions of protocol for talking to HMTL modules via serial
#

"""HMTL Protocol definitions module"""

import struct

# Protocol commands
HMTL_CONFIG_READY  = "ready"
HMTL_CONFIG_ACK    = "ok"
HMTL_CONFIG_START  = "start"
HMTL_CONFIG_END    = "end"
HMTL_CONFIG_PRINT  = "print"

HMTL_TERMINATOR    = '\n' # Indicates end of command

#
# Config starts with the config start byte, followed by the type of object,
# followed by the encoded form of that onbject
#
CONFIG_START_FMT = '<BB'
CONFIG_START_BYTE = 0xFE

# These values must match those in HMTLTypes.h
CONFIG_TYPES = {
    "header"  : 0x0,
    "value"   : 0x1,
    "rgb"     : 0x2,
    "program" : 0x3,
    "pixels"  : 0x4,
    "mpr121"  : 0x5,
    "rs485"   : 0x6
}

# Individial object formats
HEADER_FMT = '<BBBHBBB'
HEADER_MAGIC = 0x5C


#
# Configuration validation
#

def check_required(output, value, length=None):
    if (not value in output):
        print("ERROR: '" + value + "' is required in '" + output["type"] + "' config")
        return False
    if (length and (len(output[value]) != length)):
        print("ERROR: '%s' field should have length %d in '%s' config" %
              (value, length, output["type"]))
        return False

    return True

def validate_output(output):
    """Verify that an output's configuration is valid"""
    if (not "type" in output):
        print("No 'type' field in output: " + str(output))
        return False
    if (not output["type"] in CONFIG_TYPES):
        print(output["type"] + " is not a valid HMTL type")
        return False

    if (output["type"] == "value"):
        # There should be a pin and value fields
        if (not check_required(output, "pin")): return False
        if (not check_required(output, "value")): return False
    elif (output["type"] == "rgb"):
        # There should be three pins and three values
        if (not check_required(output, "pins", 3)): return False
        if (not check_required(output, "values", 3)): return False
    elif (output["type"] == "pixels"):
        if (not check_required(output, "clockpin")): return False
        if (not check_required(output, "datapin")): return False
        if (not check_required(output, "numpixels")): return False
        if (not check_required(output, "rgbtype")): return False

#XXX: Continue here
    return True

def validate_config(data):
    """Verify that the configuration file is valid"""

    print("* Validating configuration data")

    if (not "header" in data):
        print("Input file does not contain 'header'")
        return False

    config = data["header"]
    if (not "protocol_version" in config):
        print("No protocol_version in config")
        return False
    if (not "hardware_version" in config):
        print("No hardware_version in config")
        return False
    if (not "address" in config):
        print("No address in config")
        return False
    if (not "flags" in config):
        config['flags'] = 0

    if (not "outputs" in data):
        print("Input file does not contain 'data'")
        return False

    for output in data["outputs"]:
        if (not validate_output(output)):
            return False;

    return True

#
# Configuration formatting
#

def get_config_start(type):
    packed = struct.pack(CONFIG_START_FMT,
                         CONFIG_START_BYTE,
                         CONFIG_TYPES[type])
    return packed

def get_header_struct(data):
    config = data['header']

    packed_start = get_config_start('header')

    packed = struct.pack(HEADER_FMT,
                         HEADER_MAGIC,
                         config['protocol_version'],
                         config['hardware_version'],
                         config['address'],
                         0,
                         len(data['outputs']),
                         config['flags'])
    return packed_start + packed

def get_output_struct(output):
    packed_start = get_config_start(output["type"])

    return packed_start