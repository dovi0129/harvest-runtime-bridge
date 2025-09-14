# examples/scan_bus.py
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
DEV = "/dev/ttyUSB0"
PROTO = 2.0
BAUDS = [57600, 115200, 1000000, 2000000, 3000000]  # 흔한 값 우선

for baud in BAUDS:
    port = PortHandler(DEV)
    pkt  = PacketHandler(PROTO)
    if not port.openPort():
        print(f"[{baud}] openPort FAIL"); continue
    if not port.setBaudRate(baud):
        print(f"[{baud}] setBaudRate FAIL"); port.closePort(); continue
    data, res = pkt.broadcastPing(port)
    if res == COMM_SUCCESS and data:
        print(f"[{baud}] Detected IDs:")
        for dxl_id, (model, fw) in data.items():
            print(f"  ID={dxl_id} model={model} fw={fw}")
    else:
        print(f"[{baud}] no response (res={res})")
    port.closePort()
