# Простое управление: выставить указательный палец прямо
# Требуется: pip install plusml-rh56dftp pymodbus==3.6.9

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import POS_SET_3, POS_SET_2, POS_SET_1, POS_SET_0, POS_SET_4, POS_SET_5

import sys
import io
import time

# Принудительно установите UTF-8 кодировку для вывода
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')


STRAIGHT_POS = 0
BEND_POS = 1600
TIMESPAN = 0.2

try:
    client = RH56DFTP_TCP(host="192.168.123.211", port=6000)
    print("✅ Connected")
    
    ok = client.set(POS_SET_0, STRAIGHT_POS)
    ok = client.set(POS_SET_1, STRAIGHT_POS)
    ok = client.set(POS_SET_2, STRAIGHT_POS)
    ok = client.set(POS_SET_3, STRAIGHT_POS)
    ok = client.set(POS_SET_4, STRAIGHT_POS)
    ok = client.set(POS_SET_5, STRAIGHT_POS)
    
    time.sleep(1)

    ok = client.set(POS_SET_0, BEND_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_1, BEND_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_2, BEND_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_3, BEND_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_4, BEND_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_5, BEND_POS)
    time.sleep(TIMESPAN)
	
	
	

    ok = client.set(POS_SET_0, STRAIGHT_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_1, STRAIGHT_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_2, STRAIGHT_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_3, STRAIGHT_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_4, STRAIGHT_POS)
    time.sleep(TIMESPAN)
    ok = client.set(POS_SET_5, STRAIGHT_POS)
    time.sleep(TIMESPAN)
    #print(f"➡️  Set index finger position to {INDEX_STRAIGHT_POS}: {ok}")

    # Небольшая пауза, чтобы движение успело выполниться
    time.sleep(1)
    
    ok = client.set(POS_SET_1, BEND_POS)
    ok = client.set(POS_SET_2, BEND_POS)
    ok = client.set(POS_SET_4, BEND_POS)
    #ok = client.set(POS_SET_5, BEND_POS)
    
    #ok = client.set(POS_SET_3, BEND_POS)
    #ok = client.set(POS_SET_2, BEND_POS)

    client.close()
    print("👋 Connection closed")
except Exception as e:
    print(f"❌ Error: {e}")