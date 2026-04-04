#!/usr/bin/env python
#
# *********     Ping Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#

from scservo_sdk.port_handler import PortHandler
from scservo_sdk.scservo_def import COMM_SUCCESS
from scservo_sdk.sms_sts import sms_sts


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler("/dev/tty.usbmodem58FA0961181")

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Try to ping ID 5 FTServo
# Get SCServo model number
scs_model_number, scs_comm_result, scs_error = packetHandler.ping(5)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
else:
    print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (5, scs_model_number))
if scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# Close port
portHandler.closePort()
