"""Kinematic digital twin in Gazebo Fortress (see docs/ARCHITECTURE.md D4).

* animated joints (knife, laser heads, beams) follow ``joint_states`` through
  JointPositionController topics (``/belt_sim/<joint>``, bridged to Gazebo);
* every laser mark reported by the simulated plant (``sim/plant_events``) is spawned as a
  thin decal model on the belt and moved with the belt position (``SetEntityPose``);
* every cut spawns the cut piece (belt colour, with its marks as visuals) as a dynamic
  body; when the ejector runs (``PIECE_OUT``) the piece is dropped onto the chute and
  slides into the output bin.

Belt positions come from the plant model (stepper position), not from Gazebo contacts,
so the twin is exact and repeatable.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List

from belt_marking_interfaces.msg import IoStatus, ProcessEvent
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

BELT_Z = 0.30


@dataclass
class Decal:
    name: str
    station_x_mm: float        # conveyor position where it was made
    pos_at_mark_mm: float      # belt position when it was made
    weak: bool
    spawned: bool = False
    last_x: float = float('nan')


@dataclass
class Piece:
    name: str
    marks: List[Decal] = field(default_factory=list)
    ejected: bool = False


def box_sdf(name, sx, sy, sz, rgba, static=True, mass=0.02, extra_visuals='') -> str:
    r, g, b, a = rgba
    inertial = '' if static else (
        f'<inertial><mass>{mass}</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy>'
        f'<izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    collision = '' if static else (
        f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        f'<surface><friction><ode><mu>0.3</mu></ode></friction></surface></collision>')
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}">'
            f'<static>{"true" if static else "false"}</static><link name="link">{inertial}'
            f'{collision}<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box>'
            f'</geometry><material><ambient>{r} {g} {b} {a}</ambient>'
            f'<diffuse>{r} {g} {b} {a}</diffuse><emissive>{r * 0.3} {g * 0.3} {b * 0.3} 1'
            f'</emissive></material></visual>{extra_visuals}</link></model></sdf>')


class GzTwinNode(Node):

    def __init__(self):
        super().__init__('gz_twin_node')
        p = self.declare_parameter
        p('world', 'belt_marking')
        p('station_offsets_mm', [0.0])
        p('knife_offset_mm', 56.0)
        p('mark_length_mm', 20.0)
        p('belt_width_mm', 25.0)
        p('conveyor_end_m', 0.45)
        p('chute_drop_x_m', 0.53)
        p('max_pieces_in_bin', 15)
        p('rate_hz', 30.0)                  # decal pose updates (lag = speed / rate)
        g = self.get_parameter
        world = g('world').value
        self.stations = [float(v) for v in g('station_offsets_mm').value]
        self.knife = g('knife_offset_mm').value
        self.mark_len = g('mark_length_mm').value
        self.width = g('belt_width_mm').value / 1000.0
        self.x_end = g('conveyor_end_m').value
        self.drop_x = g('chute_drop_x_m').value
        self.max_bin = g('max_pieces_in_bin').value

        self.cli_spawn = self.create_client(SpawnEntity, f'/world/{world}/create')
        self.cli_pose = self.create_client(SetEntityPose, f'/world/{world}/set_pose')
        self.cli_del = self.create_client(DeleteEntity, f'/world/{world}/remove')
        self.joint_pubs: Dict[str, object] = {}
        self.position_mm = 0.0
        self.decals: List[Decal] = []
        self.pieces_on_table: List[Piece] = []
        self.pieces_in_bin: List[Piece] = []
        self.n_marks = 0
        self.n_pieces = 0
        self.last_cut_pos = None
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self.create_subscription(IoStatus, 'hw/io_status', self._on_io, qos_profile_sensor_data)
        self.create_subscription(ProcessEvent, 'sim/plant_events', self._on_plant, 100)
        self.create_timer(1.0 / g('rate_hz').value, self._update)
        self.get_logger().info(f'Gazebo twin for world "{world}"')

    # ---------------------------------------------------------------- joints
    def _on_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if not (name.startswith('knife') or name.startswith('laser_')):
                continue
            pub = self.joint_pubs.get(name)
            if pub is None:
                pub = self.create_publisher(Float64, f'/belt_sim/{name}', 10)
                self.joint_pubs[name] = pub
            pub.publish(Float64(data=float(pos)))

    def _on_io(self, msg: IoStatus):
        self.position_mm = msg.position_mm

    # ---------------------------------------------------------------- plant
    def _on_plant(self, ev: ProcessEvent):
        if ev.type == ProcessEvent.MARK and 'no_belt' not in ev.detail:
            self.n_marks += 1
            x = self.stations[ev.station] if ev.station < len(self.stations) else 0.0
            self.decals.append(Decal(f'mark_{self.n_marks}', x, ev.feed_mm, 'weak' in ev.detail))
        elif ev.type == ProcessEvent.CUT:
            self._cut(ev.feed_mm)
        elif ev.type == ProcessEvent.PIECE_OUT:
            self._eject()

    def _decal_x_mm(self, d: Decal) -> float:
        return d.station_x_mm + (self.position_mm - d.pos_at_mark_mm)

    def _cut(self, feed_mm: float):
        """Everything downstream of the knife (up to the previous cut) becomes a piece."""
        self.n_pieces += 1
        piece = Piece(f'piece_{self.n_pieces}')
        prev = self.last_cut_pos
        self.last_cut_pos = feed_mm
        downstream_mm = (self.x_end * 1000.0) - self.knife
        # the first cut frees everything downstream of the knife (leading waste included)
        length_mm = (feed_mm - prev) if prev is not None else downstream_mm
        length_mm = max(5.0, min(length_mm, downstream_mm))
        start_x = self.knife / 1000.0
        center_x = start_x + length_mm / 2000.0
        visuals = ''
        for d in list(self.decals):
            x = self._decal_x_mm(d) / 1000.0
            if start_x <= x <= start_x + length_mm / 1000.0 + 0.001:
                piece.marks.append(d)
                self.decals.remove(d)
                self._delete(d.name)
                rel = x + self.mark_len / 2000.0 - center_x
                c = '0.45 0.45 0.45 1' if d.weak else '0.95 0.92 0.75 1'
                visuals += (f'<visual name="{d.name}"><pose>{rel} 0 0.0011 0 0 0</pose>'
                            f'<geometry><box><size>{self.mark_len / 1000.0} '
                            f'{self.width * 0.7} 0.0004</size></box></geometry><material>'
                            f'<ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>')
        sdf = box_sdf(piece.name, length_mm / 1000.0, self.width, 0.002, (0.1, 0.1, 0.1, 1),
                      static=False, extra_visuals=visuals)
        self._spawn(piece.name, sdf, center_x, BELT_Z + 0.004)
        self.pieces_on_table.append(piece)

    def _eject(self):
        if not self.pieces_on_table:
            return
        piece = self.pieces_on_table.pop(0)
        self._set_pose(piece.name, self.drop_x, BELT_Z - 0.03, pitch=0.75)
        self.pieces_in_bin.append(piece)
        while len(self.pieces_in_bin) > self.max_bin:
            self._delete(self.pieces_in_bin.pop(0).name)

    # ---------------------------------------------------------------- loop
    def _update(self):
        for d in list(self.decals):
            x = self._decal_x_mm(d) / 1000.0
            if x > self.x_end + 0.02:
                self._delete(d.name)
                self.decals.remove(d)
                continue
            if not d.spawned:
                c = (0.45, 0.45, 0.45, 1) if d.weak else (0.95, 0.92, 0.75, 1)
                sdf = box_sdf(d.name, self.mark_len / 1000.0, self.width * 0.7, 0.0004, c)
                self._spawn(d.name, sdf, x + self.mark_len / 2000.0, BELT_Z + 0.0003)
                d.spawned, d.last_x = True, x
            elif abs(x - d.last_x) > 0.0002:
                self._set_pose(d.name, x + self.mark_len / 2000.0, BELT_Z + 0.0003)
                d.last_x = x

    # ---------------------------------------------------------------- gz calls
    def _spawn(self, name, sdf, x, z):
        if not self.cli_spawn.service_is_ready():
            return
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory(name=name, sdf=sdf, allow_renaming=False)
        req.entity_factory.pose = self._pose(x, 0.0, z)
        self.cli_spawn.call_async(req)

    def _set_pose(self, name, x, z, pitch=0.0):
        if not self.cli_pose.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        req.pose = self._pose(x, 0.0, z, pitch)
        self.cli_pose.call_async(req)

    def _delete(self, name):
        if not self.cli_del.service_is_ready():
            return
        req = DeleteEntity.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        self.cli_del.call_async(req)

    @staticmethod
    def _pose(x, y, z, pitch=0.0) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.y = math.sin(pitch / 2.0)
        p.orientation.w = math.cos(pitch / 2.0)
        return p


def main(args=None):
    rclpy.init(args=args)
    node = GzTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
