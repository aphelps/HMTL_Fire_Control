#!/usr/bin/python
##!/usr/local/bin/python3
#
# This reads a HMTL configuration in JSON format and sends it over the serial
# connection to a HMTL module

from optparse import OptionParser
import json
from pprint import pprint
import serial
from binascii import hexlify
import time

import portscan
import HMTLprotocol
from HMTLSerial import *


def handle_args():
    global options

    parser = OptionParser()
    parser.add_option("-f", "--file", dest="filename", 
                      help="HMTL configuration file", metavar="FILE")
    parser.add_option("-d", "--device", dest="device",
                      help="Arduino USB device")
    parser.add_option("-a", "--address", dest="address", type="int",
                      help="Set address");

    parser.add_option("-c", "--chooseport", dest="chooseport", action="store_true",
                      help="Choose port from list of available ports", default=False)


    parser.add_option("-n", "--dryrun", dest="dryrun", action="store_true",
                      help="Perform dryrun only", default=False)
    parser.add_option("-w", "--write", dest="writeconfig", action="store_true",
                      help="Write configuration", default=False)
    parser.add_option("-p", "--print", dest="printconfig", action="store_true",
                      help="Read and print the current configuration", default=False)
    parser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                      help="Verbose output", default=False)



    (options, args) = parser.parse_args()
    # print("options:" + str(options) + " args:" + str(args))

    # Required args
    if ((options.filename == None) and
        (options.printconfig == False) and
        (options.address == None)):
        parser.print_help()
        exit("Must specify mode")

    if (options.chooseport):
        options.device = portscan.choose_port()

    if ((options.dryrun == False) and (options.device == None)):
        parser.print_help()
        exit("Must specify device if not in dry-run mode");

    return (options, args)

# Send the entire configuration
def send_configuration(config_data):
    print("***** Sending configuration *****");

    if (ser.send_command(HMTLprotocol.HMTL_CONFIG_START) == False):
        print("Failed to get ack from start message")
        exit(1)

    header_struct = HMTLprotocol.get_header_struct(config_data)
    ser.send_config('header', header_struct)

    for output in config_data["outputs"]:
        output_struct = HMTLprotocol.get_output_struct(output)
        ser.send_config(output["type"], output_struct)

    if (ser.send_command(HMTLprotocol.HMTL_CONFIG_END) == False):
        print("Failed to get ack from end message")
        exit(1)

# Just send commands to read the existing configuration and set the address
def send_address(address):
    print("***** Setting address to %d *****" % (address))

    send_command(HMTLprotocol.HMTL_CONFIG_READ)
    send_command(HMTLprotocol.HMTL_CONFIG_START)
    send_config("address", HMTLprotocol.get_address_struct(address))
    send_command(HMTLprotocol.HMTL_CONFIG_END)

def main():
    global ser

    handle_args()

    config_data = None
    if (options.filename != None):
        # Parse the JSON configuration file
        json_file = open(options.filename)
        config_data = json.load(json_file)
        json_file.close()

        if (options.address != None):
            print("* Setting address to %d" % options.address)

            # Set the address in the read config
            config_data["header"]["address"] = options.address

        print("****** Config read from '" + options.filename + "':")
        pprint(config_data);

        if (not HMTLprotocol.validate_config(config_data)):
            print("ERROR: Exiting due to invalid configuration file")
            exit(1)

    if (options.dryrun == False):
        # Open the serial connection and wait for 
        ser = HMTLSerial(options.device, 
                         verbose=options.verbose, 
                         dryrun=options.dryrun)

    if (options.printconfig == True):
        # Only read and print the configuration
        ser.send_command(HMTLprotocol.HMTL_CONFIG_READ)
        options.verbose = True
        ser.send_command(HMTLprotocol.HMTL_CONFIG_PRINT)
        exit(0)

    if (config_data != None):
        # Send the configuration
        send_configuration(config_data)
    elif (options.address != None):
        # Send the address update
        ser.send_address(options.address)

    if (options.verbose):
        # Have the module output its entire configuration
        ser.send_command(HMTLprotocol.HMTL_CONFIG_PRINT)

    if (options.writeconfig == True):
        # Write out the module's config to EEPROM
        ser.send_command(HMTLprotocol.HMTL_CONFIG_WRITE)

main()
