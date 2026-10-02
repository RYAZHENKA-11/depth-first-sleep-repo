#!/usr/bin/env python3
"""Headless wrapper for `unitree-fetch-aes-key` — stubs audio, then runs the CLI.

The console script imports the package (-> webrtc_audio -> sounddevice) which
aborts on a headless node with no audio server. Control/fetch need no audio.

Usage (this robot is 国行 Go2 -> region cn, device-type Go2):
  ./venv/bin/python fetch_key.py --email you@example.com --region cn --device-type Go2
      (lists every device bound to the account, with its AES-128 key)
  ./venv/bin/python fetch_key.py --email you@example.com --region cn --device-type Go2 \
      --sn <SERIAL> -q            # print only the bare 32-hex key
Password is prompted interactively when --password is omitted (never on argv).
"""
import sys
import types

try:
    import sounddevice  # noqa: F401
except Exception:
    _sd = types.ModuleType("sounddevice")
    _sd.query_devices = lambda *a, **k: []
    _sd.OutputStream = _sd.InputStream = _sd.RawOutputStream = object
    sys.modules["sounddevice"] = _sd

from unitree_webrtc_connect._cli import main

if __name__ == "__main__":
    raise SystemExit(main())
