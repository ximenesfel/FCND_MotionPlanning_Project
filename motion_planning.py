import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from planning_utils import a_star, heuristic, create_grid, create_grid_and_edges, closest_point, a_star_graph, heuristic_graph
from myDrone import MyDrone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global
from planning_utils import prune_path
import networkx as nx
import numpy.linalg as LA


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(MyDrone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

    def update_ne_plot(self):
        ne = np.array([self.local_position[1], self.local_position[0]]).reshape(1, -1)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_d_plot(self):
        d = np.array([self.local_position[2]])
        # update timestep
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')

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
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

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
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        METHOD = "grid_search"

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        # Read the latitude and longitude information from csv file
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)
        
        # TODO: set home position to (lon0, lat0, 0)

        # Processing data from colliders.csv to obtain values of
        # latitude and longitude
        lat = row1[0].strip('lat0')
        long = row1[1].strip(' lon0')

        # As the output from csv is a string, a convertion to float is necesssary
        lat0 = float(lat)
        long0 = float(long)


        # Setting the home position with the values of csv file.
        self.set_home_position(long0, lat0, 0.0)

        # TODO: retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)
 
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))


        # Read all the values from obstacles in csv file;
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)


        
        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        north_offset = -316
        east_offset = -445

        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)

        # TODO: convert start position to current position rather than map center

        north_start = int(local_position[0])
        east_start = int(local_position[1])

        grid_start = ((north_start + -north_offset), (east_start + -east_offset))
        
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)



        # TODO: adapt to set goal as latitude / longitude position and convert
        latitude_goal = 37.79503409
        longitude_goal = -122.39635021


        goal_global = [longitude_goal, latitude_goal, 0]
        goal_local = global_to_local(goal_global, self.global_home)

        north_goal = int(goal_local[0])
        east_goal = int(goal_local[1])


        grid_goal = ((north_goal + -north_offset), (east_goal + -east_offset))

        ########    Run A* to find a path from start to goal  ###############################

        ### So we have the grid representation and the


        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)

        if METHOD == "grid_search":

            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            path, _ = a_star(grid, heuristic, grid_start, grid_goal)

            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            pruned_path = prune_path(path)

            # Convert path to waypoints
            waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        elif METHOD == "graph_search":

            grid, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

            G = nx.Graph()
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                dist = LA.norm(np.array(p2) - np.array(p1))
                G.add_edge(p1, p2, weight=dist)

            start_ne_g = closest_point(G, grid_start)
            goal_ne_g = closest_point(G, grid_goal)

            path, cost = a_star_graph(G, heuristic_graph, start_ne_g, goal_ne_g)

            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            pruned_path = (prune_path(path))


            # Convert path to waypoints
            waypoints = [[int(p[0]) + north_offset , int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]


        # # TODO: prune path to minimize number of waypoints
        # # TODO (if you're feeling ambitious): Try a different approach altogether!
        # pruned_path = prune_path(path)
        #
        # # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        # Set self.waypoints
        self.waypoints = waypoints


        # TODO: send waypoints to sim (this is just for visualization of waypoints)
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
