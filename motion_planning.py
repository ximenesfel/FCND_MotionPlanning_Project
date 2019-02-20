import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import h5py

from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone import Drone


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.index = 0

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)


    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):

        self.previous_position = self.target_position

        if len(self.waypoints):

            print("[INFO] Waypoint transition ...")
            self.target_position = self.waypoints.pop(0)
            print('[INFO] Target position: ', self.target_position)

            if len(self.waypoints):

                # Obaining the next two points from waypoint

                if self.index == 0:
                    self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
                    self.index += 1
                    self.flight_state = States.WAYPOINT

                else:

                    waypointActual = self.previous_position
                    waypointNext = self.target_position


                    self.heading = np.arctan2((waypointNext[1] - waypointActual[1]),
                                              (waypointNext[0] - waypointActual[0]))


                    self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                                      self.heading)

                    self.index += 1

                    self.flight_state = States.WAYPOINT

            else:
                waypointActual = self.previous_position
                waypointNext = self.target_position
                self.heading = np.arctan2((waypointNext[1] - waypointActual[1]), (waypointNext[0] - waypointActual[0]))

                self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                                  self.heading)

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):

        self.flight_state = States.PLANNING

        # Read hdf5 file
        f = h5py.File('waypoints.hdf5', 'r')
        a_group_key = list(f.keys())[0]

        # Get the waypoint from hdf5 file
        waypoints = list(f[a_group_key][()])

        # Convert to a data that simulator understand
        waypoints = [ [int(p[0]), int(p[1]), int(p[2]), int(p[3])] for p in waypoints]

        # Set self.waypoints
        self.waypoints = waypoints

        # Send waypoint to simulator
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
