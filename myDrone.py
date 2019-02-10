from udacidrone import Drone

import numpy as np
import visdom


class MyDrone(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array([self.local_position[1], self.local_position[0]]).reshape(1, -1)
        self.ne_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)",
            xlabel='North',
            ylabel='East'
        ))

        # Plot D
        d = np.array([self.local_position[2]])
        self.t = 0
        self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
            title="Altitude (meters)",
            xlabel='Timestep',
            ylabel='Up'
        ))
