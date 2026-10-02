#!/usr/bin/env python3
"""
Go2 Pro — battery & low-level health from LOW_STATE.
SAFE: read-only, no motion commands.

Run:  ./venv/bin/python 02_battery.py
"""
import asyncio

import go2
from unitree_webrtc_connect.constants import RTC_TOPIC


async def main() -> int:
    conn = await go2.connect()
    print("[*] Subscribing to LOW_STATE ...")
    box = {}

    def cb(message):
        box.setdefault("data", message["data"])

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], cb)

    for _ in range(100):  # up to ~10s
        if "data" in box:
            break
        await asyncio.sleep(0.1)

    d = box.get("data")
    if not d:
        print("[!] No LOW_STATE frame received in 10s.")
        await go2.disconnect(conn)
        return 2

    bms = d["bms_state"]
    motor_temps = [m["temperature"] for m in d["motor_state"]]
    print("Go2 Pro — Battery / Health")
    print("==========================")
    print(f"  SOC (charge)   : {bms['soc']} %")
    print(f"  Current        : {bms['current']} mA")
    print(f"  Cycles         : {bms['cycle']}")
    print(f"  Pack version   : {bms['version_high']}.{bms['version_low']}")
    print(f"  Power voltage  : {d['power_v']} V")
    print(f"  BQ / MCU NTC   : {bms['bq_ntc']} / {bms['mcu_ntc']} °C")
    print(f"  Board NTC1     : {d['temperature_ntc1']} °C")
    print(f"  Motor temps    : min {min(motor_temps)} / max {max(motor_temps)} °C  (20 motors)")
    print(f"  Foot force     : {d['foot_force']}")

    await go2.disconnect(conn)
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
