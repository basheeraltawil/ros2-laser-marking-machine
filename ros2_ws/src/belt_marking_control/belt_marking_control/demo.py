"""Live scenario demo: drives a RUNNING simulation step by step, with narration.

Terminal 1:  ros2 launch belt_marking_bringup sim.launch.py          (Gazebo + RViz + UI)
Terminal 2:  ros2 run belt_marking_control demo_scenario 8           (scenario number)

The demo uses only the public interfaces (RunJob action, machine/command, recipes,
sim/inject_fault), so everything it does can also be done by hand in the UI. Each step
prints what to watch. The demo ends with PASS/FAIL checks.

The same scenarios run headless and accelerated in CI: ``run_scenarios`` (scenarios.py).
"""

import argparse
import sys
import time
from typing import Callable, List

from belt_marking_interfaces.action import RunJob
from belt_marking_interfaces.msg import IoStatus, JobSpec, LaserStation, MachineState
from belt_marking_interfaces.srv import InjectFault, MachineCommand, SaveRecipe, SetMode
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)

RESET, HOLD, UNHOLD, STOP, ABORT, CLEAR = range(6)


def spec(job_id, quantity, pitch=60.0, cut=JobSpec.CUT_EVERY, every_n=1, laser_s=1.0,
         width=25.0, mark=30.0, lead=10.0, done=JobSpec.LASER_DONE_CONFIG):
    """Build a JobSpec with sensible defaults for the demos."""
    return JobSpec(job_id=job_id, quantity=quantity, pitch_mm=pitch, mark_length_mm=mark,
                   lead_mm=lead, cut_mode=cut, cut_every_n=every_n, laser_time_s=laser_s,
                   settle_s=0.1, feed_speed_mm_s=30.0, belt_width_mm=width,
                   laser_done_mode=done)


class Demo:
    """Drives a running simulation through a scenario via public ROS interfaces."""

    def __init__(self):
        self.node = rclpy.create_node('scenario_demo')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state: MachineState = None
        self.node.create_subscription(MachineState, 'machine/state',
                                      lambda m: setattr(self, 'state', m), qos)
        self.io = None
        self.node.create_subscription(IoStatus, 'hw/io_status',
                                      lambda m: setattr(self, 'io', m),
                                      qos_profile_sensor_data)
        self.cmd = self.node.create_client(MachineCommand, 'machine/command')
        self.mode = self.node.create_client(SetMode, 'machine/set_mode')
        self.fault_cli = self.node.create_client(InjectFault, 'sim/inject_fault')
        self.save = self.node.create_client(SaveRecipe, 'recipes/save')
        self.action = ActionClient(self.node, RunJob, 'machine/run_job')
        self.result_future = None
        self.checks: List[tuple] = []

    # ------------------------------------------------------------------ helpers
    def say(self, text: str, watch: str = ''):
        print(f'\n▶ {text}')
        if watch:
            print(f'  👀 {watch}')

    def spin_until(self, pred: Callable[[], bool], timeout: float) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if pred():
                return True
        return False

    def call(self, client, req):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{client.srv_name} not available - is the simulation running?')
        fut = client.call_async(req)
        self.spin_until(fut.done, 10.0)
        return fut.result()

    def command(self, code, option=''):
        res = self.call(self.cmd, MachineCommand.Request(command=code, option=option,
                                                         user='demo'))
        print(f'  command -> {res.message}')
        return res.accepted

    def fault(self, name, enable=True, value=0.0, station=0):
        res = self.call(self.fault_cli, InjectFault.Request(fault=name, enable=enable,
                                                            value=value, station=station))
        print(f'  fault injection: {res.message}')

    def wait_state(self, name: str, timeout: float = 60.0) -> bool:
        ok = self.spin_until(lambda: self.state is not None and self.state.state_name == name,
                             timeout)
        print(f'  state = {self.state.state_name if self.state else "?"}')
        return ok

    def ready(self):
        self.say('Bring the machine to IDLE (RESET, or CLEAR + RESET after an abort)')
        if not self.spin_until(lambda: self.state is not None, 20.0):
            raise RuntimeError('no machine/state - start: ros2 launch belt_marking_bringup '
                               'sim.launch.py')
        self.fault('clear_all')
        if self.state.state_name == 'ABORTED':
            self.spin_until(lambda: False, 0.5)
            self.command(CLEAR)
            self.wait_state('STOPPED', 10)
        if self.state.mode_name not in ('AUTO', 'SIMULATION'):
            self.call(self.mode, SetMode.Request(mode=MachineState.MODE_AUTO, user='demo'))
        if self.state.state_name in ('EXECUTE', 'HELD', 'HOLDING'):
            self.command(STOP)
            self.wait_state('STOPPED', 20)
        if self.state.state_name != 'IDLE':
            self.command(RESET)
            self.wait_state('IDLE', 10)

    def start(self, job: JobSpec):
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('machine/run_job not available')
        fut = self.action.send_goal_async(RunJob.Goal(spec=job, user='demo'))
        self.spin_until(fut.done, 10.0)
        handle = fut.result()
        print(f'  job {job.job_id}: {"accepted" if handle.accepted else "REJECTED"}')
        self.job_id = job.job_id
        self.result_future = handle.get_result_async() if handle.accepted else None
        return handle.accepted

    def wait_done(self, timeout=600.0):
        if self.result_future is None:
            return None
        self.spin_until(self.result_future.done, timeout)
        if not self.result_future.done():
            return None
        r = self.result_future.result().result
        print(f'  result: {r.message}, marks {r.marks_done}, pieces {r.pieces_cut}, '
              f'rejects {r.rejects}, {r.duration_s:.1f} s')
        return r

    def this_job(self) -> bool:
        """machine/state already describes the job started last (not the previous one)."""
        return self.state is not None and self.state.job_id == getattr(self, 'job_id', None)

    def wait_marks(self, n, timeout=300.0):
        return self.spin_until(lambda: self.this_job() and self.state.marks_done >= n, timeout)

    def alarm_active(self, code):
        return any(a.code == code for a in self.state.active_alarms)

    def check(self, text, ok):
        self.checks.append((text, bool(ok)))
        print(f'  {"✔" if ok else "✘"} {text}')

    # ---------------------------------------------------------------- scenarios
    def s1(self):
        self.say('Continuous marking: 10 labels, no cut',
                 'Gazebo: a light mark appears under the laser every 60 mm and travels '
                 'towards the bin; the knife never moves. UI: marks counter rises.')
        self.start(spec('DEMO-1', 10, cut=JobSpec.CUT_NONE))
        r = self.wait_done()
        self.check('job COMPLETE with 10 marks, 0 pieces', r and r.success and
                   r.marks_done == 10 and r.pieces_cut == 0)

    def s2(self):
        self.say('Batch: 8 pieces, cut every piece, fixed laser time (timed mode)',
                 'Gazebo: mark -> feed 46 mm -> knife crosses the belt -> the piece drops '
                 'into the bin. Light tower green.')
        self.start(spec('DEMO-2', 8, pitch=40.0, mark=20.0, lead=5.0,
                        done=JobSpec.LASER_DONE_TIMED, laser_s=2.5))
        r = self.wait_done()
        self.check('8 pieces cut', r and r.success and r.pieces_cut == 8)

    def s3(self):
        self.say('Sets of labels: 12 labels, cut every 4',
                 'Gazebo: three pieces of 4 labels each drop into the bin.')
        self.start(spec('DEMO-3', 12, pitch=40.0, mark=20.0, lead=5.0,
                        cut=JobSpec.CUT_EVERY_N, every_n=4))
        r = self.wait_done()
        self.check('3 pieces of 4 labels', r and r.success and r.pieces_cut == 3)

    def s4(self):
        self.say('Two laser stations (start multi_laser_sim.launch.py for this one)',
                 'Gazebo: both laser heads raster on every stop; station 1 marks the label '
                 '150 mm behind station 0, 0.3 s later.')
        job = spec('DEMO-4', 6)
        job.stations = [LaserStation(enabled=True, offset_mm=0.0, delay_s=0.0),
                        LaserStation(enabled=True, offset_mm=150.0, delay_s=0.3)]
        if not self.start(job):
            print('  (needs the 2-station launch: ros2 launch belt_marking_bringup '
                  'multi_laser_sim.launch.py)')
            self.check('two-station job accepted', False)
            return
        r = self.wait_done()
        self.check('6 labels marked by both stations', r and r.success and r.marks_done == 6)

    def s5(self):
        self.say('Recipe change: save two belt types, run both',
                 'UI Recipes: "belt-20mm" and "belt-50mm" appear. Gazebo: guides stay, '
                 'mark spacing changes 40 -> 80 mm.')
        for name, width, pitch in (('belt-20mm', 20.0, 40.0), ('belt-50mm', 50.0, 80.0)):
            s = spec(name, 4, pitch=pitch, width=width, mark=20.0, lead=5.0)
            self.call(self.save, SaveRecipe.Request(name=name, spec=s, overwrite=True,
                                                    user='demo'))
            self.start(s)
            r = self.wait_done()
            self.check(f'{name}: complete', r and r.success)
            self.ready()
        self.start(spec('TOO-WIDE', 2, width=150.0))
        r = self.wait_done(10)
        self.check('150 mm belt refused by validation',
                   r is not None and not r.success and 'belt width' in r.message)

    def s6(self):
        self.say('Laser does not finish: the done signal never comes',
                 'After label 2 the laser stays busy. After 2·t + 2 s: E-201, yellow light, '
                 'state HELD, belt still.')
        self.start(spec('DEMO-6', 6, laser_s=1.0))
        self.wait_marks(2)
        self.fault('laser_late', value=30.0)
        self.check('E-201 and HELD', self.wait_state('HELD', 30) and self.alarm_active(201))
        self.say('Operator: fix the laser, then RESUME -> REJECT (count the label as reject)',
                 'UI: RESUME asks RETRY or REJECT.')
        self.fault('clear_all')
        print('  waiting until the laser controller is idle again ...')
        self.spin_until(lambda: self.io is not None and not any(self.io.laser_busy), 60.0)
        self.command(UNHOLD, 'reject')
        r = self.wait_done()
        self.check('job completes, 1 reject, no label lost',
                   r and r.success and r.rejects == 1 and r.marks_done == 6)

    def s7(self):
        self.say('Knife sticks before the end position',
                 'Gazebo: knife stops half way, returns. UI: E-301, HELD.')
        self.start(spec('DEMO-7', 5))
        self.spin_until(lambda: self.this_job() and self.state.pieces_cut >= 2, 120)
        self.fault('knife_stuck_extend')
        self.check('E-301 and HELD', self.wait_state('HELD', 30) and self.alarm_active(301))
        self.say('Operator: free the knife, RESUME')
        self.fault('clear_all')
        self.command(UNHOLD)
        r = self.wait_done()
        self.check('all 5 pieces cut', r and r.success and r.pieces_cut == 5)

    def s8(self):
        self.say('Belt runs out mid-batch',
                 'UI: E-401 "No belt at the fork sensor", HELD; counters keep their value.')
        self.start(spec('DEMO-8', 10))
        self.wait_marks(3)
        self.fault('belt_runout', value=20.0)
        self.check('E-401 and HELD', self.wait_state('HELD', 60) and self.alarm_active(401))
        held = self.state.marks_done
        self.say('Operator: splice a new belt (clear fault), RESUME')
        self.fault('belt_runout', enable=False)
        self.spin_until(lambda: False, 0.5)
        self.command(UNHOLD)
        r = self.wait_done()
        self.check(f'count kept ({held} at hold) and all 10 done',
                   r and r.success and r.marks_done == 10)

    def s9(self):
        self.say('USB link to the Arduino lost while moving',
                 'UI: E-501, red light, ABORTED. Firmware model: belt stops within 0.5 s.')
        self.start(spec('DEMO-9', 10))
        self.wait_marks(2)
        self.fault('link_loss')
        self.check('E-501 and ABORTED', self.wait_state('ABORTED', 10) and
                   self.alarm_active(501))
        self.say('Operator: reconnect, CLEAR, RESET')
        self.fault('link_loss', enable=False)
        self.spin_until(lambda: False, 1.0)
        self.command(CLEAR)
        self.wait_state('STOPPED', 10)
        self.command(RESET)
        self.check('back to IDLE', self.wait_state('IDLE', 10))

    def s10(self):
        self.say('E-stop during a cut',
                 'Gazebo: everything stops. UI: E-101, red light, ABORTED.')
        self.start(spec('DEMO-10', 5))
        self.spin_until(lambda: self.this_job() and self.state.phase == 'CUT_EXTEND', 120)
        self.fault('estop')
        self.check('E-101 and ABORTED', self.wait_state('ABORTED', 5) and
                   self.alarm_active(101))
        self.check('CLEAR refused while pressed', not self.command(CLEAR))
        self.say('Operator: release the E-stop, CLEAR, RESET (knife retracts)')
        self.fault('estop', enable=False)
        self.spin_until(lambda: False, 0.5)
        self.command(CLEAR)
        self.wait_state('STOPPED', 10)
        self.command(RESET)
        self.check('IDLE again', self.wait_state('IDLE', 10))

    def s11(self):
        self.say('Vision QA: the laser starts making weak marks (vision:=true)',
                 'Gazebo: marks turn grey. UI: rejects counter rises; after 3 in a row E-602, '
                 'HELD.')
        self.start(spec('DEMO-11', 12))
        self.wait_marks(3)
        self.fault('laser_weak_mark')
        self.check('E-602 after 3 rejects', self.wait_state('HELD', 120) and
                   self.alarm_active(602))
        self.say('Operator: clean the lens, RESUME')
        self.fault('clear_all')
        self.command(UNHOLD)
        r = self.wait_done()
        self.check('job completes with rejects counted', r and r.success and r.rejects >= 3)

    def s12(self):
        self.say('Soak test runs 8 simulated hours: use the accelerated headless runner',
                 'ros2 run belt_marking_control run_scenarios 12 --hours 8   (~25 s)')
        self.check('see run_scenarios', True)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Run a scenario live on the simulation.')
    ap.add_argument('number', type=int, choices=range(1, 13))
    args = ap.parse_args([a for a in (argv or sys.argv[1:]) if not a.startswith('--ros')])
    rclpy.init()
    demo = Demo()
    try:
        if args.number != 12:
            demo.ready()
        getattr(demo, f's{args.number}')()
    except RuntimeError as exc:
        print(f'\n✘ {exc}')
        return 2
    finally:
        demo.node.destroy_node()
        rclpy.try_shutdown()
    ok = all(c for _, c in demo.checks)
    print(f'\n{"PASS" if ok else "FAIL"}: scenario {args.number} '
          f'({sum(c for _, c in demo.checks)}/{len(demo.checks)} checks)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
