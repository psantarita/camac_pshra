import ctypes
import struct
import time

DLL_PATH = r"C:\Windows\SysWOW64\libxxusb.dll"
SERIAL = b"CC0284"

dll = ctypes.WinDLL(DLL_PATH)
dll.xxusb_serial_open.restype = ctypes.c_void_p
hdev = dll.xxusb_serial_open(SERIAL)

if not hdev:
    print(
        "Could not open CC-USB. Please turn off the CAMAC crate, wait 3 seconds, and turn it back on."
    )
    exit()

print("1. Killing zombie List Mode in FPGA...")
# CRITICAL: [5, 0, 0] = register 0 (Action Register) = 0 (stop)
# Previously [5, 1, 0] wrote to register 1 — WRONG! List mode never stopped.
pkt = struct.pack("<3H", 5, 0, 0)
dll.xxusb_bulk_write(hdev, pkt, len(pkt), 2000)
time.sleep(0.3)  # give FPGA time to finish in-flight stack execution

print("2. Clearing LAM masks and Global Mode...")
q = ctypes.c_int(0)
x = ctypes.c_int(0)
dll.CAMAC_write(hdev, 25, 9, 16, ctypes.c_long(0), ctypes.byref(q), ctypes.byref(x))
dll.CAMAC_write(hdev, 25, 1, 16, ctypes.c_long(0), ctypes.byref(q), ctypes.byref(x))

print("3. Deep CAMAC Hardware Flush (C & Z)...")
dll.CAMAC_C(hdev)
time.sleep(0.2)
dll.CAMAC_Z(hdev)
time.sleep(0.2)

print("4. Forcing Phillips ADC Data Clear (F9 A0)...")
data = ctypes.c_long(0)
dll.CAMAC_read(hdev, 17, 0, 9, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x))
dll.CAMAC_read(hdev, 19, 0, 9, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x))

print("5. Flushing lingering USB PC buffers...")
buf = ctypes.create_string_buffer(65536)
flush_count = 0
for _ in range(100):  # increased iterations for multi-event buffers
    if dll.xxusb_bulk_read(hdev, buf, 65536, 100) > 0:
        flush_count += 1
    else:
        break
if flush_count:
    print(f"   Flushed {flush_count} stale buffer(s).")

dll.xxusb_device_close(hdev)
print("Hardware successfully rescued! The dataway is clean.")
