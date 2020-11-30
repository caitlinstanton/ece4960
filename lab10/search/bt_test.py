#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import unpack, calcsize


class RobotHolder:
    def __init__(self):
        client = BleakClient(theRobot_bt.address,
                             loop=loop,
                             device=Settings["adapter"])
        # if (await client.is_connected()):
        #    print("Robot connected!")
        # srv = await client.get_services()
        # print(srv)
        client.is_connected()
        self.theRobot = Robot(client, bleak=True)
        client.start_notify(Descriptors["TX_CHAR_UUID"].value, simpleHandler)

    def setter(self, toSet):
        self.instanceVariable = toSet

    def getter(self):
        return self.theRobot


async def bluetooth_get_pose():
    print('Robot is moving')
    await asyncio.sleep(3)
    print('Robot has stopped')
    return [0, 0, 30]  # dummy data


async def bluetooth_perform_observation_loop():
    print('Executing Rotation Behavaior')
    await asyncio.sleep(3)
    print('Done with Rotation Behavior')
    return [0.3, 0.2, 0.5]  # dummy data


async def getRobot():
    devices = await discover(device=Settings["adapter"], timeout=2)
    # for d in devices:
    #    print(d.name)
    p_robot = [d for d in devices if d.name == "MyRobot"]
    if (len(p_robot) > 0):
        #    print(p_robot[0].address)
        return p_robot[0]
    else:
        return None


async def robotTest(loop):

    # Handle is the TX characteristic UUID; does not change in our simple case.
    # Robot sends "enq" every 2 seconds to keep the connection "fresh"
    # Otherwise, it's a struct of the form:
    # bytes(type + length + data)
    # This struct shouldn't be more than 99 bytes long.

    def simpleHandler(handle, value):
        global time  # This is apparently needed.
        if (value == "enq".encode()):
            pass
        else:
            fmtstring = "BB" + str(len(value) - 2) + "s"
            code, length, data = unpack(fmtstring, value)
            '''
            Python doesn't have a switch statement, nor easily compatible
            enum support. This might be the easiest way to handle commands.
            '''
            if (Settings["OutputRawData"]):
                print(
                    f"Code: {getCommandName(code)} Length: {length} Data: {data}"
                )

            # Somewhat detach console output from Bluetooth handling.
            if (code == Commands.SER_TX.value):
                theRobot.pushMessage(str(data, encoding="UTF-8"))

            # Example of unpacking a little-endian 32-bit float.
            if (code == Commands.GIVE_FLOAT.value):
                print(unpack("<f", data))

            # Example of command-response.
            if (code == Commands.PONG.value):
                print(f"Got pong: round trip {time.time() - theRobot.now}")
                if (Settings["pingLoop"]):
                    loop.create_task(theRobot.ping())
                    # theRobot.newPing = True

            # Unpack from an example stream that transmits a 2-byte and a
            # 4-byte integer as quickly as possible, both little-endian.
            if (code == Commands.BYTESTREAM_TX.value):
                print(unpack("<LiIfff",
                             data))  #unpacks 1 long, 2 chars and 2 floats

    async def checkMessages():
        while True:
            if (theRobot.availMessage()):
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)