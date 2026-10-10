import time
from xrobotoolkit_teleop.common.xr_client import XrClient

xr = XrClient()

print("实时打印左右扳机值 (Ctrl+C 停止)")
print(f"{'时间':>10s}  {'左扳机':>10s}  {'右扳机':>10s}")
print("-" * 35)

try:
    while True:
        left_val = xr.get_key_value_by_name("left_trigger")
        right_val = xr.get_key_value_by_name("right_trigger")
        now = time.strftime("%H:%M:%S")
        print(f"\r{now}  {left_val:10.4f}  {right_val:10.4f}", end="", flush=True)
        time.sleep(0.05)  # 20Hz
except KeyboardInterrupt:
    print("\n已停止")
