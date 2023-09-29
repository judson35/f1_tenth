import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import os
import yaml

class BagReader:
    def __init__(self):
        scan_data = []

        scan_cutoff_range = np.pi # 3.14159
        scan_angle_increment = 4.7 / 1080
        scan_angle_min = -4.7/2
        start_i = int((-scan_cutoff_range/2 - scan_angle_min)/scan_angle_increment)
        end_i = int((scan_cutoff_range/2 - scan_angle_min)/scan_angle_increment)
        self.x_axis = np.array([scan_angle_min + scan_angle_increment*i for i in range(end_i - start_i)])

        reject_threshold = 3.0
        disparity_threshold = 0.9
        gap_threshold = 3.0
        mean_window = 20
        disparity_bubble = 10

        # create reader instance and open for reading
        with Reader('/home/judson35/f1_tenth/sim_ws/bag_plotting/rosbag2_2023_09_28-16_47_46') as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/scan':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    ranges = np.array(msg.ranges[start_i:end_i])
                    ranges = np.clip(ranges,0, reject_threshold)
                    scan_data.append(ranges)

        self.scan_data = np.array(scan_data)
        print(np.sum(self.scan_data,axis=1))
        self.i = 1000
    
    def update_plot(self, frame):
        length = len(reader.scan_data[0])
        x = np.linspace(0,length, length)
        y = self.scan_data[self.i*4]

        scat.set_offsets(np.stack([self.x_axis, y]).T)
        self.i += 1
        return scat

if __name__ == "__main__":

    reader = BagReader()


    fig, ax = plt.subplots()

    length = len(reader.scan_data[0])
    scat = ax.scatter(reader.x_axis, reader.scan_data[0], label=f'Lidar Data')
    ax.set(xlabel='Angle i', ylabel='Z [m]')
    ax.legend()


    ani = animation.FuncAnimation(fig=fig, func=reader.update_plot, frames=7550, interval=60)
    plt.show()
