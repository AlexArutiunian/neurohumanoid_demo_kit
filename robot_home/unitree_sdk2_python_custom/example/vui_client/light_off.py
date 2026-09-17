
"""
python3 light_off.py eth0
"""

import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.vui.vui_client import VuiClient

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)

client = VuiClient()
client.SetTimeout(3.0)
client.Init()

print("switch off ret =", client.SetSwitch(0))
time.sleep(0.5)
print("brightness ret =", client.SetBrightness(0))
