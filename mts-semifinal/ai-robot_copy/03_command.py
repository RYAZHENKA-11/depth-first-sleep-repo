#!/usr/bin/env python3
"""
Go2 Pro — send ONE high-level command over the WebRTC API. SAFETY-GATED.

DRY RUN BY DEFAULT: without --yes it only prints what it WOULD do and exits
without touching the robot. Add --yes to actually send. Acrobatics need --force.

Posture / locomotion require the robot ON THE FLOOR with clearance around it.

Examples:
  ./venv/bin/python 03_command.py list
  ./venv/bin/python 03_command.py hello                 # dry run (prints plan)
  ./venv/bin/python 03_command.py hello --yes
  ./venv/bin/python 03_command.py stand_up --yes
  ./venv/bin/python 03_command.py stand_down --yes
  ./venv/bin/python 03_command.py move 0.3 0 0 --duration 2 --yes
  ./venv/bin/python 03_command.py move 0 0 0.5 --duration 1.5 --yes   # turn in place
  ./venv/bin/python 03_command.py stop --yes
  ./venv/bin/python 03_command.py backflip --yes --force              # high risk
"""
import asyncio
import sys

import go2

# name -> (SPORT_CMD key, needs_floor, is_danger, description)
POSE = {
    "damp":       ("Damp",          False, False, "motors soft / limp"),
    "stop":       ("StopMove",      False, False, "stop current motion"),
    "balance":    ("BalanceStand",  True,  False, "balance stand"),
    "stand_up":   ("StandUp",       True,  False, "stand up tall"),
    "stand_down": ("StandDown",     True,  False, "lie down"),
    "recovery":   ("RecoveryStand", True,  False, "recover to standing"),
    "sit":        ("Sit",           True,  False, "sit"),
    "rise_sit":   ("RiseSit",       True,  False, "rise from sit"),
    "hello":      ("Hello",         True,  False, "wave hello"),
    "stretch":    ("Stretch",       True,  False, "stretch"),
    "wiggle":     ("WiggleHips",    True,  False, "wiggle hips"),
    "heart":      ("FingerHeart",   True,  False, "finger heart"),
    # Acrobatics (flips) need AI mode + battery pre-flight → handled by 05_flip.py.
    "frontflip":  ("FrontFlip",     True,  True,  "FRONT FLIP — use 05_flip.py"),
    "backflip":   ("BackFlip",      True,  True,  "BACK FLIP — use 05_flip.py"),
}

# Safe velocity clamps for `move` (unless --force). x=fwd m/s, y=lat m/s, z=yaw rad/s
LIM = {"x": 0.6, "y": 0.5, "z": 1.0}


def usage_and_exit(code=0):
    print(__doc__)
    print("Commands:", ", ".join(["move"] + list(POSE)))
    sys.exit(code)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


async def run(cmd, args, do_it, force):
    conn = await go2.connect()
    try:
        if cmd == "move":
            x, y, z = args["x"], args["y"], args["z"]
            await go2.ensure_normal_mode(conn)
            print(f"[*] move x={x} y={y} z={z} for {args['duration']}s ...")
            t = 0.0
            try:
                while t < args["duration"]:
                    await go2.sport(conn, "Move", {"x": x, "y": y, "z": z})
                    await asyncio.sleep(0.1)
                    t += 0.1
            finally:
                await go2.sport(conn, "StopMove")  # graceful stop, always
                print("[*] StopMove sent.")
        else:
            key, needs_floor, _danger, _desc = POSE[cmd]
            if needs_floor:
                await go2.ensure_normal_mode(conn)
            print(f"[*] sending {key} ...")
            await go2.sport(conn, cmd if cmd == "Move" else POSE[cmd][0])
            await asyncio.sleep(2)
            await go2.sport(conn, "StopMove")
        print("[+] done. ✅")
    finally:
        await go2.disconnect(conn)


def main():
    argv = sys.argv[1:]
    if not argv or argv[0] in ("-h", "--help", "help"):
        usage_and_exit(0)
    if argv[0] == "list":
        print("Available commands:")
        print(f"  {'move':<12} x y z [--duration S]   drive (fwd/lat/yaw), auto-StopMove")
        for n, (k, floor, danger, desc) in POSE.items():
            tag = "  ⚠DANGER" if danger else ("  (floor)" if floor else "")
            print(f"  {n:<12} {desc}{tag}")
        sys.exit(0)

    do_it = "--yes" in argv
    force = "--force" in argv
    flags = {"--yes", "--force"}
    cmd = argv[0]

    if cmd == "move":
        toks = list(argv[1:])
        duration = 2.0
        if "--duration" in toks:
            i = toks.index("--duration")
            duration = float(toks[i + 1])
            del toks[i:i + 2]
        nums = [a for a in toks if not a.startswith("--")]
        if len(nums) < 3:
            print("[!] move needs: move <x> <y> <z> [--duration S]")
            sys.exit(1)
        x, y, z = float(nums[0]), float(nums[1]), float(nums[2])
        if not force:
            x = clamp(x, -LIM["x"], LIM["x"])
            y = clamp(y, -LIM["y"], LIM["y"])
            z = clamp(z, -LIM["z"], LIM["z"])
        args = {"x": x, "y": y, "z": z, "duration": duration}
        plan = f"move x={x} y={y} z={z} for {duration}s, then StopMove"
        danger = False
    elif cmd in POSE:
        key, floor, danger, desc = POSE[cmd]
        args = {}
        plan = f"{key} — {desc}" + ("  [needs floor + clearance]" if floor else "")
    else:
        print(f"[!] unknown command: {cmd}")
        usage_and_exit(1)

    print(f"PLAN: {plan}")
    if danger:
        print(f"[!] '{cmd}' is an acrobatic move — needs AI mode + battery pre-flight, "
              "not done here. Use the dedicated safety-checked script:")
        print(f"      ./venv/bin/python 05_flip.py {cmd} --yes --force")
        sys.exit(3)
    if not do_it:
        print("\nDRY RUN (no --yes) — nothing sent.")
        print("Safety checklist before --yes:")
        print("  • Unitree mobile app CLOSED (it holds the WebRTC slot)")
        print("  • robot on the FLOOR with >2 m clearance (for any posture/move)")
        print("  • you can hit Ctrl+C — the script sends StopMove on exit")
        sys.exit(0)

    try:
        asyncio.run(run(cmd, args, do_it, force))
    except KeyboardInterrupt:
        print("\n[!] interrupted — robot should have received StopMove.")
        sys.exit(0)


if __name__ == "__main__":
    main()
