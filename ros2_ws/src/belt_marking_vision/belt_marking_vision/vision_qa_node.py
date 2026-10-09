"""Vision QA node: inspect every label after the laser and publish ``quality/result``.

Modes
  camera     (Gazebo QA camera or a real camera driver publishing sensor_msgs/Image)
             For each MARK event the node predicts when the mark passes under the camera
             from the belt position (machine/state), keeps the frame where the mark is
             closest to the image centre, and inspects it.
  synthetic  (simulation without Gazebo) renders the label from the plant's ground truth
             (sim/plant_events: weak / missing marks) and inspects the rendered image.

The controller counts rejects and holds after N consecutive rejects (E-602). The node
never commands the machine.
"""

from collections import deque
from dataclasses import dataclass
import time
from typing import Optional

from belt_marking_interfaces.msg import MachineState, ProcessEvent, QualityResult
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .inspector import InspectConfig, MarkInspector, render_label


@dataclass
class PendingMark:
    job_id: str
    label: int
    belt_coord_mm: float
    best_dist: float = float('inf')
    best_image: Optional[np.ndarray] = None
    best_signed_mm: float = 0.0        # mark centre - camera centre at that frame
    t_created: float = 0.0


def mark_center_x(feed_mm: float, belt_coord_mm: float, mark_len_mm: float) -> float:
    """Conveyor x [mm] of the mark centre (same convention as the twin / RViz)."""
    return feed_mm - belt_coord_mm + mark_len_mm / 2.0


def image_to_array(msg: Image) -> np.ndarray:
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'R8G8B8': 3}.get(msg.encoding, 3)
    arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding in ('rgb8', 'R8G8B8'):
        arr = arr[:, :, ::-1]
    return arr if channels == 3 else arr[:, :, 0]


class VisionQaNode(Node):
    """Inspects every label after the laser and publishes quality/result."""

    def __init__(self):
        super().__init__('vision_qa_node')
        p = self.declare_parameter
        p('synthetic', True)
        p('image_topic', '/qa_camera/image_raw')
        p('camera_offset_mm', 32.0)
        p('mark_length_mm', 20.0)
        p('window_mm', 4.0)              # capture while the mark centre is this close
        p('rotate_deg', 90)              # rotate frames so the belt runs along image x
        p('mm_per_px', 0.1)
        p('min_contrast', 115.0)
        p('roi_across', [0.0, 1.0])      # belt band in the (rotated) image, rows
        p('detector', 'classic')         # classic (OpenCV rules) | yolo (ONNX model)
        p('yolo_model', '')              # path to the exported .onnx file
        p('yolo_conf', 0.4)
        p('downstream_sign', -1)         # +1/-1: image x direction of belt travel
        p('max_offset_mm', 2.0)          # allowed mark position error along the belt
        p('expected_text', '')           # fallback when the job has no mark_text
        p('use_sim', True)
        g = self.get_parameter
        self.synthetic = g('synthetic').value
        self.cam_x = g('camera_offset_mm').value
        self.mark_len = g('mark_length_mm').value
        self.window = g('window_mm').value
        self.downstream_sign = 1 if g('downstream_sign').value >= 0 else -1
        self.mm_per_px = g('mm_per_px').value
        self.rotate = int(g('rotate_deg').value) % 360
        self.expected = g('expected_text').value
        if g('detector').value == 'yolo':
            from .yolo_detector import YoloDetector, YoloInspector   # needs only OpenCV
            self.inspector = YoloInspector(
                YoloDetector(g('yolo_model').value, conf=g('yolo_conf').value),
                mm_per_px=g('mm_per_px').value, min_contrast=g('min_contrast').value,
                max_offset_mm=g('max_offset_mm').value)
        else:
            self.inspector = MarkInspector(InspectConfig(
                mm_per_px=g('mm_per_px').value, min_contrast=g('min_contrast').value,
                max_offset_mm=g('max_offset_mm').value,
                roi_across=tuple(float(v) for v in g('roi_across').value)))
        self.state: Optional[MachineState] = None
        self.state_t = 0.0
        self.pending = deque(maxlen=50)
        self.plant_marks = deque(maxlen=50)       # (weak, on_belt) from the simulated plant
        self.stats = {'ok': 0, 'reject': 0}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(QualityResult, 'quality/result', 50)
        self.create_subscription(MachineState, 'machine/state', self._on_state, latched)
        self.create_subscription(ProcessEvent, 'machine/events', self._on_event, 100)
        if self.synthetic:
            self.create_subscription(ProcessEvent, 'sim/plant_events', self._on_plant, 100)
        else:
            self.create_subscription(Image, g('image_topic').value, self._on_image,
                                     qos_profile_sensor_data)
        self.get_logger().info(f'vision QA ({"synthetic" if self.synthetic else "camera"}, '
                               f'detector {g("detector").value}), '
                               f'camera at {self.cam_x} mm, OCR '
                               f'{"on" if self.inspector.ocr else "off (tesseract missing)"}')

    # ------------------------------------------------------------------ inputs
    def _on_state(self, msg: MachineState):
        self.state, self.state_t = msg, time.monotonic()

    def _on_plant(self, ev: ProcessEvent):
        if ev.type == ProcessEvent.MARK and ev.station == 0:
            self.plant_marks.append(('weak' in ev.detail, 'no_belt' not in ev.detail))

    def _on_event(self, ev: ProcessEvent):
        if ev.type != ProcessEvent.MARK or ev.station != 0:
            return
        if self.synthetic:
            weak, on_belt = self.plant_marks.popleft() if self.plant_marks else (False, True)
            text = self._text(ev.job_id, ev.label_index)
            img = render_label(text, weak=weak, missing=not on_belt, seed=ev.label_index)
            self._publish(ev.job_id, ev.label_index, img, text)
        else:
            self.pending.append(PendingMark(ev.job_id, ev.label_index, ev.belt_coord_mm,
                                            t_created=time.monotonic()))

    def _feed_now(self) -> Optional[float]:
        st = self.state
        if st is None:
            return None
        return st.belt_position_mm + st.belt_speed_mm_s * (time.monotonic() - self.state_t)

    def _on_image(self, msg: Image):
        feed = self._feed_now()
        if feed is None or not self.pending:
            return
        img = None
        for pm in list(self.pending):
            dist = mark_center_x(feed, pm.belt_coord_mm, self.mark_len) - self.cam_x
            if abs(dist) <= self.window and abs(dist) < pm.best_dist:
                if img is None:
                    img = self._rotate(image_to_array(msg))
                pm.best_dist, pm.best_image, pm.best_signed_mm = abs(dist), img, dist
            if dist > self.window or time.monotonic() - pm.t_created > 120.0:
                self.pending.remove(pm)
                if pm.best_image is not None:
                    self._publish(pm.job_id, pm.label, pm.best_image,
                                  self._text(pm.job_id, pm.label), pm.best_signed_mm)

    def _rotate(self, img: np.ndarray) -> np.ndarray:
        k = {0: 0, 90: 1, 180: 2, 270: 3}.get(self.rotate, 0)
        return np.ascontiguousarray(np.rot90(img, k)) if k else img

    def _text(self, job_id: str, label: int) -> str:
        return self.expected or f'{job_id}-{label}'[-12:]

    # ----------------------------------------------------------------- output
    def _publish(self, job_id, label, img, text, mark_offset_mm: float = 0.0):
        expected = text if (self.synthetic or self.expected) else ''
        # where the mark should be in this frame: the belt moves between frames, so the
        # chosen frame is up to (speed / fps) away from the camera centre
        expected_px = img.shape[1] / 2 + self.downstream_sign * mark_offset_mm / self.mm_per_px
        r = self.inspector.inspect(img, expected_text=expected, expected_center_px=expected_px)
        msg = QualityResult(job_id=job_id, label_index=int(label), ok=r.ok,
                            score=float(r.score), text_read=r.text, reason=r.reason,
                            offset_mm=float(r.offset_mm))
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.stats['ok' if r.ok else 'reject'] += 1
        text = (f'label {label} {"OK" if r.ok else "rejected: " + r.reason} '
                f'(contrast {r.contrast}, offset {r.offset_mm} mm, score {r.score})')
        # separate call sites: rclpy forbids changing the severity of one call site
        if r.ok:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = VisionQaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
