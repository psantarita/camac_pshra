"""
CAMAC Slot Scanner
==================
Scans all CAMAC slots (1-23) and reports which ones
have a module responding with Q=1 or X=1.

Also reads the CC-USB firmware ID and serial number.
"""

import ctypes
import struct
import time

DLL_PATH = r"C:\Windows\SysWOW64\libxxusb.dll"
SERIAL = b"CC0406"  # ← change to match your unit, 0284

dll = ctypes.WinDLL(DLL_PATH)
dll.xxusb_serial_open.restype = ctypes.c_void_p
dll.xxusb_serial_open.argtypes = [ctypes.c_char_p]
dll.xxusb_device_close.restype = ctypes.c_short
dll.xxusb_device_close.argtypes = [ctypes.c_void_p]
dll.CAMAC_read.restype = ctypes.c_short
dll.CAMAC_read.argtypes = [
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_long),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
]
dll.CAMAC_C.restype = ctypes.c_short
dll.CAMAC_C.argtypes = [ctypes.c_void_p]
dll.CAMAC_Z.restype = ctypes.c_short
dll.CAMAC_Z.argtypes = [ctypes.c_void_p]

hdev = dll.xxusb_serial_open(SERIAL)
if not hdev:
    print(f"Cannot open CC-USB with serial {SERIAL.decode()}")
    print("Check: is the crate powered on? Is the USB cable connected?")
    exit(1)

# Read CC-USB internal registers (N=25)
data = ctypes.c_long(0)
q = ctypes.c_int(0)
x = ctypes.c_int(0)

dll.CAMAC_read(hdev, 25, 0, 0, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x))
fw = data.value & 0xFFFF
print(f"CC-USB Firmware ID: 0x{fw:04X}")

dll.CAMAC_read(hdev, 25, 13, 0, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x))
sn = data.value & 0x7FF
print(f"CC-USB Serial: {sn}")

# C/Z to reset dataway cleanly
dll.CAMAC_C(hdev)
time.sleep(0.1)
dll.CAMAC_Z(hdev)
time.sleep(0.1)

print("\n" + "=" * 60)
print("CAMAC SLOT SCAN")
print("=" * 60)
print(
    f"{'Slot':>4s}  {'Q':>1s}  {'X':>1s}  {'F(0)A(0) data':>14s}  {'F(1)A(0) data':>14s}  Module?"
)
print("-" * 60)

found = []

for slot in range(1, 24):
    if slot == 25:
        continue  # skip CC-USB internal

    # F(0)A(0) = Read data register, channel 0
    d0 = ctypes.c_long(0)
    q0 = ctypes.c_int(0)
    x0 = ctypes.c_int(0)
    dll.CAMAC_read(
        hdev, slot, 0, 0, ctypes.byref(d0), ctypes.byref(q0), ctypes.byref(x0)
    )

    # F(1)A(0) = Read control/status register
    d1 = ctypes.c_long(0)
    q1 = ctypes.c_int(0)
    x1 = ctypes.c_int(0)
    dll.CAMAC_read(
        hdev, slot, 0, 1, ctypes.byref(d1), ctypes.byref(q1), ctypes.byref(x1)
    )

    has_module = x0.value or x1.value or q0.value or q1.value

    if has_module:
        # Try to identify module type
        module_hint = ""

        # Check if it's an ADC by trying F(4)A(0) sparse read
        d4 = ctypes.c_long(0)
        q4 = ctypes.c_int(0)
        x4 = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev, slot, 0, 4, ctypes.byref(d4), ctypes.byref(q4), ctypes.byref(x4)
        )

        # Check if it's a scaler by trying F(0)A(0) and seeing if Q=1
        if q0.value and d0.value == 0:
            module_hint = "← Possible scaler/counter"
        if x4.value:
            module_hint = "← Possible ADC (F4 responds)"

        # Try F(6)A(1) = Read Hit Register (Phillips 7164 specific)
        d6 = ctypes.c_long(0)
        q6 = ctypes.c_int(0)
        x6 = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev, slot, 1, 6, ctypes.byref(d6), ctypes.byref(q6), ctypes.byref(x6)
        )
        if x6.value and q6.value:
            module_hint = "← Phillips 7164 ADC (Hit Register OK)"

        # Try F(0)A(15) which TDCs often respond to
        d15 = ctypes.c_long(0)
        q15 = ctypes.c_int(0)
        x15 = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev, slot, 15, 0, ctypes.byref(d15), ctypes.byref(q15), ctypes.byref(x15)
        )

        found.append(slot)
        print(
            f"  {slot:2d}   {q0.value}  {x0.value}  0x{d0.value & 0xFFFFFF:06X}       "
            f"0x{d1.value & 0xFFFFFF:06X}       ✓ {module_hint}"
        )
    else:
        print(f"  {slot:2d}   {q0.value}  {x0.value}  {'—':>14s}  {'—':>14s}  empty")

print("-" * 60)
if found:
    print(f"\nModules found in slots: {found}")
    print(f"\nTo use in DAQ, set:")
    for s in found:
        print(
            f'  ADC_xxx = {{"slot": {s}, "channels": list(range(16)), "name": "Slot {s}"}}'
        )
else:
    print("\nNo modules found! Check:")
    print("  - Is the module fully seated in the crate?")
    print("  - Is the crate power supply on?")
    print("  - Slot numbering: count from LEFT starting at 1")

dll.xxusb_device_close(hdev)
print("\n[OK] Scanner done.")
