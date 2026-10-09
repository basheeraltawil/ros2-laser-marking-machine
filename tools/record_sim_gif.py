#!/usr/bin/env python3
"""Record the Gazebo overview camera with a status banner and save an animated GIF.

Used for the README GIFs. With the simulation running (sim.launch.py, Gazebo on):

    python3 tools/record_sim_gif.py --title "Cut every piece" --seconds 40 --out docs/images/x.gif
    # meanwhile, in another terminal:  ros2 run belt_marking_control demo_scenario 2

Needs ffmpeg. Frames are cropped to the machine, a banner shows state, counters and the
active alarm (from machine/state), and the GIF is played back `--speed` times faster,
except fault states (HOLDING, HELD, ABORTING, ABORTED), which play at real speed and are
held for at least `--fault-hold` seconds so the alarm can be read.
"""

import argparse
import os
import shutil
import subprocess
import tempfile
import time

from belt_marking_interfaces.msg import MachineState
import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

FAULT_STATES = {'HOLDING', 'HELD', 'ABORTING', 'ABORTED'}
STATE_COLORS = {'red': (60, 60, 230), 'yellow': (40, 200, 240), 'green': (80, 200, 80),
                'none': (200, 200, 200)}


def banner(frame: np.ndarray, title: str, st) -> np.ndarray:
    h, w = frame.shape[:2]
    bar = np.full((84, w, 3), 32, np.uint8)
    cv2.putText(bar, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2,
                cv2.LINE_AA)
    if st is not None:
        light = 'red' if st.light_red else 'yellow' if st.light_yellow else \
            'green' if st.light_green else 'none'
        cv2.circle(bar, (w - 24, 22), 11, STATE_COLORS[light], -1)
        text = (f'{st.state_name}   marks {st.marks_done}/{st.marks_total}   '
                f'pieces {st.pieces_cut}   rejects {st.rejects}')
        cv2.putText(bar, text, (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1,
                    cv2.LINE_AA)
        if st.active_alarms:
            a = st.active_alarms[0]
            cv2.putText(bar, f'{a.code_text}  {a.text}'[:60], (12, 76),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, frame])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--title', default='Belt marking machine (simulation)')
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/overview/image')
    ap.add_argument('--crop', default='380,50,1220,700', help='x0,y0,x1,y1 in camera pixels')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--speed', type=float, default=2.0)
    ap.add_argument('--fps', type=int, default=10, help='camera rate')
    ap.add_argument('--gif-fps', type=int, default=8)
    ap.add_argument('--fault-hold', type=float, default=2.5)
    a = ap.parse_args()
    x0, y0, x1, y1 = (int(v) for v in a.crop.split(','))

    rclpy.init()
    node = rclpy.create_node('gif_recorder')
    state = {'st': None}
    frames = []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MachineState, 'machine/state',
                             lambda m: state.__setitem__('st', m), latched)

    def on_image(msg: Image):
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_RGB2BGR)
        scale = a.width / img.shape[1]
        img = cv2.resize(img, (a.width, int(img.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        st = state['st']
        frames.append((banner(img, a.title, st), st is not None and st.state_name in FAULT_STATES,
                       st.state_name if st is not None else ''))

    node.create_subscription(Image, a.topic, on_image, qos_profile_sensor_data)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    print(f'{len(frames)} frames')

    # trim the idle tail: keep 1.5 s after the last state change
    last_change = max((i for i in range(1, len(frames)) if frames[i][2] != frames[i - 1][2]),
                      default=len(frames) - 1)
    frames = [f[:2] for f in frames[:last_change + int(1.5 * a.fps)]]
    # fault frames: repeat `speed` times (real speed) and pad each fault episode
    out, episode = [], []
    min_frames = int(a.fault_hold * a.fps * a.speed)
    for img, fault in frames + [(None, False)]:
        if fault:
            episode.append(img)
            continue
        if episode:
            seq = [f for f in episode for _ in range(int(a.speed))]
            seq += [seq[-1]] * max(0, min_frames - len(seq))
            out += seq
            episode = []
        if img is not None:
            out.append(img)
    tmp = tempfile.mkdtemp()
    for i, f in enumerate(out):
        cv2.imwrite(os.path.join(tmp, f'{i:05d}.png'), f)
    rate = a.fps * a.speed
    palette = os.path.join(tmp, 'palette.png')
    src = ['-framerate', str(rate), '-i', os.path.join(tmp, '%05d.png')]
    subprocess.run(['ffmpeg', '-v', 'error', '-y', *src, '-vf',
                    f'fps={a.gif_fps},palettegen=max_colors=96:stats_mode=diff', palette], check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', *src, '-i', palette, '-lavfi',
                    f'fps={a.gif_fps}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4', a.out],
                   check=True)
    shutil.rmtree(tmp)
    print(f'{a.out}: {os.path.getsize(a.out) / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
