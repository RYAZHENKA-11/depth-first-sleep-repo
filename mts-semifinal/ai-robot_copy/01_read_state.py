#!/usr/bin/env python3
"""
Go2 Pro — read-only sport-mode state (mode, body height, foot force, IMU).
SAFE: sends NO motion commands → robot can be on a table.
This is also the end-to-end connectivity test.

Run:  ./venv/bin/python 01_read_state.py
First close the Unitree mobile app (it holds the single WebRTC slot).
"""
import asyncio

import go2
from unitree_webrtc_connect.constants import RTC_TOPIC

TARGET_FRAMES = 5


async def main() -> int:
    conn = await go2.connect()
    print("[*] Subscribing to LF_SPORT_MOD_STATE ...")
    got = {"n": 0}

    def cb(message):
        d = message["data"]
        got["n"] += 1
        if got["n"] <= TARGET_FRAMES:
            imu = d["imu_state"]
            rpy = [round(x, 3) for x in imu["rpy"]]
            print(
                f"  #{got['n']:>2}  mode={d['mode']}  body_h={d['body_height']:.3f}m  "
                f"foot_force={d['foot_force']}  rpy={rpy}  imu_temp={imu['temperature']}C"
            )

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], cb)

    for _ in range(100):  # up to ~10s
        if got["n"] >= TARGET_FRAMES:
            break
        await asyncio.sleep(0.1)

    ok = got["n"] > 0
    print(
        f"[+] SUCCESS — received {got['n']} frames. Python API works end-to-end. ✅"
        if ok else "[!] Connected but no state frames in 10s (unexpected)."
    )
    if ok:
        print("    note: foot_force here reads 0 unless motion mode is 'normal' "
              "(default 'mcf' zeroes it). Real foot_force is always in LOW_STATE → 02_battery.py.")
    await go2.disconnect(conn)
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
