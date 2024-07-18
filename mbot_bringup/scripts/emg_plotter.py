#!/usr/bin/env python

import rospy
from mbot_msgs.msg import EMG
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from PyQt5 import QtCore


class EMGPlotter:
    def __init__(self):
        rospy.init_node('emg_plotter', anonymous=True)
        rospy.Subscriber('/mbot/emg', EMG, self.emg_callback)

        self.ch1_value = 0.0
        self.ch2_value = 0.0

         # Set the plot style
        plt.style.use("dark_background")

        for param in ['text.color', 'axes.labelcolor', 'xtick.color', 'ytick.color']:
            plt.rcParams[param] = '0.9'  # very light grey

        for param in ['figure.facecolor', 'axes.facecolor', 'savefig.facecolor']:
            plt.rcParams[param] = '#212946'  # bluish dark grey

        self.colors = [
            '#08F7FE',  # teal/cyan
            '#FE53BB',  # pink
            '#F5D300',  # yellow
            '#00ff41',  # matrix green
        ]

        # Hide the toolbar
        plt.rcParams['toolbar'] = 'None'

        # Initialize plot attributes
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('motion_notify_event', 
self.hide_cursor)
        self.ax.tick_params(axis='x', labelsize=12)  # Font size for x-axis tick labels
        self.ax.tick_params(axis='y', labelsize=12)
        self.ax.set_ylim(0, 3.5)
        self.ax.set_ylabel('Voltage [V]', fontsize=16)
        self.ax.set_title('EMG Sensor Voltage', fontsize=20)
        self.title = self.ax.text(0.5, 0.95, "", bbox={'facecolor':'w', 'alpha':0.1, 'pad':5},
                transform=self.ax.transAxes, ha="center")
        self.bars = self.ax.bar(['ch1', 'ch2'], [self.ch1_value, self.ch2_value], color=self.colors[:2])

        # Set grid color
        self.ax.grid(color='#2A3459')

        # Use animation for live updates
        self.ani = animation.FuncAnimation(self.fig, self.animate, init_func=self.init, interval=50, blit=True)
        
        # Set plot window to fullscreen
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()

        plt.show()

    def hide_cursor(self, event):
        self.fig.canvas.setCursor(QtCore.Qt.BlankCursor)


    def emg_callback(self, msg):
        self.ch1_value = msg.ch1
        self.ch2_value = msg.ch2       

    def init(self):
        for bar in self.bars:
            bar.set_height(0)
        return self.bars

    def animate(self, i):
        self.bars[0].set_height(self.ch1_value)
        self.bars[1].set_height(self.ch2_value)
        param_name = '/peripheral_connected'
        if rospy.has_param(param_name):
            connected = rospy.get_param(param_name)
            title = 'connected' if connected else 'disconnected'
            self.title.set_text(title)
        return self.bars + (self.title,)

    def spin(self):
        rospy.spin()  # Keep the node running


if __name__ == '__main__':
    plotter = EMGPlotter()
    plotter.spin()
