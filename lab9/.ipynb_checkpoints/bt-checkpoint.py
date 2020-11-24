# This files contains all the Bluetooth code 
import asyncio

async def bluetooth_get_pose():
  print('Robot is moving')
  await asyncio.sleep(3)
  print('Robot has stopped')
  return [0,0,30]    # dummy data

async def bluetooth_perform_observation_loop():
  print('Executing Rotation Behavaior')
  await asyncio.sleep(3)
  print('Done with Rotation Behavior')
  return [0.3,0.2,0.5]    # dummy data