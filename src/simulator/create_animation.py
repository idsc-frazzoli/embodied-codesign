# import time
from decimal import Decimal

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.animation import FuncAnimation, FFMpegWriter


class UpdateDist(object):
    def __init__(self, ax1, ax2, vstates_list, belief_list, object_list, list_of_ds):
        self.belief_list = belief_list
        self.vstates_list = vstates_list
        self.object_list = object_list
        self.line1, = ax1.plot([], [], 'k-',color="green")
        self.line2, = ax2.plot([], [], 'k-', color="green")
        self.x = list_of_ds
        self.ax1 = ax1
        self.vline, = ax1.plot([], [], linestyle='--', color='red')

        self.ax1.set_xlim(0, 50)
        self.ax1.set_ylim(0, 1)
        self.ax1.set(xlabel='distance [m]', ylabel='Belief')
        self.ax1.grid(True)

        self.ax2 = ax2

        self.car_length = 5
        self.car_width = 2
        self.car = plt.Polygon(np.array(np.array([[0 ,0.5], [-60, 0.75], [-60, 0.25]])))

        # Set up plot parameters
        self.ax2.set_xlim(0, 500)
        self.ax2.set_xlim(0, 500)
        self.ax2.set_ylim(0, 1)
        self.ax2.set(xlabel='state [m]', ylabel='Belief')
        self.ax2.grid(True)
        for obj in object_list[0]:
            self.ax2.axvline(obj, linestyle='--', color='red')

        self.title = ax2.text(1, 0.8, "Speed : ", color="blue")

        plt.tight_layout()


    def init(self):
        self.line1.set_data([], [])
        self.line2.set_data([],[])
        self.vline.set_data([],[])
        self.ax2.add_patch(self.car)
        return self.line1, self.car, self.vline, self.line2

    def __call__(self, i):
        y = self.belief_list[i].po
        objects = self.object_list[i]
        self.line1.set_data(self.x, y)
        for d in objects:
            if float(d) <= 50.0:
                self.vline.set_data([d, d], [0, 1])
        x = self.vstates_list[i].x
        x_list = [ds+x for ds in self.x]
        self.line2.set_data(x_list, y)
        self.car.set_xy(np.array([[0+x, 0.5], [-60+x, 0.75], [-60+x, 0.25]]))
        v = self.vstates_list[i].v * Decimal(str(3.6))
        self.title.set_text("Speed: " + str(v) + " km/h")
        return self.line1, self.car, self.vline, self.title, self.line2


def create_animation(vstates_list, belief_list, object_list, list_of_ds) -> None:
    fig, (ax1, ax2) = plt.subplots(2,1)
    ud = UpdateDist(ax1, ax2, vstates_list, belief_list, object_list, list_of_ds)
    frames = len(vstates_list)
    print("Creating Animation.")
    anim = FuncAnimation(fig, ud, frames=np.arange(frames), init_func=ud.init,
                         interval=20, blit=True)
    plt.show()
    # timestr = time.strftime("%Y%m%d-%H%M%S")
    # rcParams['animation.convert_path'] = r'/usr/bin/ffmpeg'
    # writervideo = FFMpegWriter(fps=20)
    # anim.save("data/output/" + timestr +"animation.mp4", writer=writervideo)
