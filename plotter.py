import asyncio
import websockets
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# websocket config
uri = "ws://192.168.1.102/ws"
imu_axes = ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
history_len = 100

# data
imu_data = {axis: deque([0.0]*history_len, maxlen=history_len) for axis in imu_axes}
joystick_pos = {"x": 0, "y": 0}
data_lock = threading.Lock()

# imu plots
fig_imu, axs = plt.subplots(2, 3, figsize=(10, 8), sharex=True)
axs = axs.flatten()
lines = {}

for i, axis in enumerate(imu_axes):
    axs[i].set_title(axis)
    axs[i].grid(True)
    lines[axis], = axs[i].plot([], [], label=axis)

# joystick plot
fig_joy, ax_joy = plt.subplots(figsize=(5, 5))
point, = ax_joy.plot([], [], 'ro', markersize=8)
ax_joy.set_title("Joystick Position")
ax_joy.set_xlim(0, 4096)
ax_joy.set_ylim(0, 4096)
ax_joy.set_xlabel("joy_x")
ax_joy.set_ylabel("joy_y")
ax_joy.grid(True)

def update_plot_imu(frame):
    with data_lock:
        for axis in imu_axes:
            y = imu_data[axis]
            x = range(len(y))
            i = imu_axes.index(axis)

            lines[axis].set_data(x, y)
            axs[i].set_xlim(0, len(y))

            if y:
                ymin, ymax = min(y), max(y)
                if ymin == ymax:
                    ymin -= 0.1
                    ymax += 0.1
                margin = 0.1 * max(abs(ymin), abs(ymax), 1e-5)
                axs[i].set_ylim(ymin - margin, ymax + margin)
    return lines.values()

def update_plot_joystick(frame):
    with data_lock:
        x = joystick_pos["x"]
        y = joystick_pos["y"]
        point.set_data([x], [y])
    return point,

async def websocket_listener():
    async with websockets.connect(uri) as socket:
        async for msg in socket:
            try:
                data = json.loads(msg)
                imu = data.get("imu", {})
                joy = data.get("joystick", {})
                with data_lock:
                    for axis in imu_axes:
                        if "acc" in axis:
                            imu_data[axis].append(imu["acceleration"][axis])
                        else:
                            imu_data[axis].append(imu["gyroscope"][axis])
                    if "joy_x" in joy and "joy_y" in joy:
                        joystick_pos["x"] = joy["joy_x"]
                        joystick_pos["y"] = joy["joy_y"]
            except Exception as e:
                print("WebSocket parse error:", e)

def start_async_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(websocket_listener())

loop = asyncio.new_event_loop()
threading.Thread(target=start_async_loop, args=(loop,), daemon=True).start()

ani_imu = FuncAnimation(fig_imu, update_plot_imu, interval=10, cache_frame_data=False)
ani_joy = FuncAnimation(fig_joy, update_plot_joystick, interval=10, cache_frame_data=False)

plt.tight_layout()
plt.show()
