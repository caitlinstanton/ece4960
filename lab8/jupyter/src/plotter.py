#!/usr/bin/env python3

# @package plot_tool
#	This tool is offers a ROS service that draws 2D parametric plots
#	The tool currently supports geometry_msgs::Pose and nav_msgs::Path
#	The plotting is done using pyqtgraph v0.9.8
import rospy
import pyqtgraph as pg
from pyqtgraph import PlotWidget, plot, TextItem
from pyqtgraph.Qt import QtGui, QtCore
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point
from lab8.msg import PlotPoint
from lab8.srv import ProbDist, ProbDistRequest, ProbDistResponse
from lab8.srv import MapInit, MapInitRequest, MapInitResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget, QPushButton, QGroupBox,
                             QHBoxLayout, QVBoxLayout, QApplication)
from configparser import ConfigParser

import math
import os
import sys
import time
import numpy as np
from os.path import expanduser


ODOM = 0
GT = 1
BEL = 2
MAP = 3
DIST = 4

TRAJ_PLOT_TYPES = [ODOM, GT, BEL]
TRAJ_PLOT_NAMES = ["Odometry", "Ground Truth", "Belief"]
COLORS = ['g', 'r', 'b', 'c', 'm', 'y', 'w']
SYMBOLS = ['o', 's', 't', 'd', '+']

# TODO: Change names to prior and posterior trajectories
# TODO: MAP (button, plot)

# This is a global graph object, we need it global so your service handlers can access the graph
graph_obj = None

# Class CustomGraphicsWindow
#	We use this class to override the closeEvent function of the base pg.GraphicsWindow
#	This is used to reopen the window, and redraw all the content, whenever the graph window is closed.


class CustomGraphicsWindow(pg.GraphicsWindow):
    # Function closeEvent
    #	This overrides the closeEvent function in the base class. This function is invoked automatically by QTGui when the window closes.
    #	@param ev This is the event object (i.e. window close).
    def closeEvent(self, ev):
        print ("Exiting gracefully...")
        rospy.signal_shutdown("User Requested")

        # recreate the graph window
        # graph_obj.win = CustomGraphicsWindow()
        # graph_obj.win.setWindowTitle('Plot Tool')
        # graph_obj.graph = graph_obj.win.addPlot()
        # graph_obj.graph.showGrid(x=True, y=True)
        # # iterate through the current plots, and readd them to the graph GUI
        # for s in graph_obj.plot_tracker:
        #     for p in s:
        #         graph_obj.graph.addItem(p)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            print ("Exiting gracefully...")
            rospy.signal_shutdown("User Requested")
        if event.key() == Qt.Key_R:
            graph_obj.graph_drawer.reset_all_plots()


# Draws the plot
class GraphDrawer():
    def __init__(self, graph, symbol, symbol_size, map_line_size):
        self.graph = graph
        self.symbol = symbol
        self.symbol_size = symbol_size
        self.map_line_size = map_line_size

        self.x_data = [[], [], []]
        self.y_data = [[], [], []]
        self.REFRESH_PLOT_FLAG = [True, True, True]
        self.last_x = [100.0, 100.0, 100.0]
        self.last_y = [100.0, 100.0, 100.0]

        self.init_plot_data(ODOM)
        self.init_plot_data(GT)
        self.init_plot_data(BEL)
        self.init_plot_data(MAP)

        self.plots = []
        self.plots.append(self.graph.plot([], [], pen=pg.mkPen(color=(80,150,236), width=2), symbolPen=(80,150,236),
                                          symbolBrush=(80,150,236), symbolSize=self.symbol_size,
                                          symbol=self.symbol))
        self.plots.append(self.graph.plot([], [], pen=pg.mkPen(color=(0, 255, 0), width=2), symbolPen='g',
                                          symbolBrush='g', symbolSize=self.symbol_size,
                                          symbol=self.symbol))
        self.plots.append(self.graph.plot([], [], pen=pg.mkPen(color=(255, 255, 0), width=2), symbolPen='y',
                                          symbolBrush='y', symbolSize=self.symbol_size,
                                          symbol=self.symbol))

        self.init_img()

    def init_img(self):
        self.img = pg.ImageItem(border='c')
        self.img_data = np.zeros((20, 20)).astype(np.uint16)
        self.img.setImage(self.img_data)

        self.img.setZValue(-100)  # make sure image is behind other data
        self.img.setRect(pg.QtCore.QRectF(-2, -2, 4, 4))
        # img.setLevels([0, 10])  # make image appear a bit darker
        self.img.setOpacity(1)

        self.img.hide()
        
        self.graph.addItem(self.img)

    def init_plot_data(self, plot_type):
        if(plot_type == MAP):
            self.map_plots = []
            self.map_x_datasets = []
            self.map_y_datasets = []
            self.REFRESH_MAP_FLAG = False
        else:
            self.x_data[plot_type] = []
            self.y_data[plot_type] = []

            self.last_x[plot_type] = 100.0
            self.last_y[plot_type] = 100.0

            self.REFRESH_PLOT_FLAG[plot_type] = True

    def draw_square(self, cx, cy, size):
        half_size = size/2.0
        x = [cx - half_size, cx - half_size, cx +
             half_size, cx + half_size, cx - half_size]
        y = [cy - half_size, cy + half_size, cy +
             half_size, cy - half_size, cy - half_size]

        plot = self.graph.plot(x, y, pen='g')

        self.map.append(plot)

    def reset_all_plots(self):
        print ("Resetting trajectory plots...")
        for plot_type in TRAJ_PLOT_TYPES:
            self.plots[plot_type].clear()
            self.init_plot_data(plot_type)

        self.img_data = np.zeros((20, 20)).astype(np.uint16)
        self.img.setImage(self.img_data)

    def hide_plot(self, plot_type):
        if(plot_type == MAP):
            print ("Hide Map")
            for map_plot in self.map_plots:
                map_plot.hide()
        elif(plot_type == DIST):
            print ("Hide Distribution")
            self.img.hide()
        else:
            print ("Hide " + TRAJ_PLOT_NAMES[plot_type])
            self.plots[plot_type].hide()

    def show_plot(self, plot_type):
        if(plot_type == MAP):
            print ("Show Map")
            for map_plot in self.map_plots:
                map_plot.show()
        elif(plot_type == DIST):
            print ("Show Distribution")
            self.img.show()
        else:
            print ("Show " + TRAJ_PLOT_NAMES[plot_type])
            self.plots[plot_type].show()

    def run_loop(self):
        # Main run loop while ROS is still going
        while not rospy.is_shutdown():

            if(self.REFRESH_PLOT_FLAG[ODOM] == True):
                self.plots[ODOM].setData(self.x_data[ODOM], self.y_data[ODOM])
                self.REFRESH_PLOT_FLAG[ODOM] = False

            if(self.REFRESH_PLOT_FLAG[GT] == True):
                self.plots[GT].setData(self.x_data[GT], self.y_data[GT])
                self.REFRESH_PLOT_FLAG[GT] = False

            if(self.REFRESH_PLOT_FLAG[BEL] == True):
                self.plots[BEL].setData(self.x_data[BEL], self.y_data[BEL])
                self.REFRESH_PLOT_FLAG[BEL] = False

            if(self.REFRESH_MAP_FLAG == True):
                for i in range(0, len(self.map_x_datasets)):
                    self.map_plots.append(self.graph.plot(self.map_x_datasets[i],
                                                          self.map_y_datasets[i],
                                                          pen='w'))
                self.REFRESH_MAP_FLAG = False

            # Process QT events so graph can still be manipulated when there are no plot requests
            QtGui.QApplication.instance().processEvents()


# Draws the layout
class GraphWindow:
    BUTTON_RESET = "Reset (r)"
    BUTTON_ODOM = "Odom"
    BUTTON_GROUND_TRUTH = "Ground Truth"
    BUTTON_BELIEF = "Belief"
    BUTTON_MAP = "Map"
    BUTTON_DIST = "Dist."
    BUTTON_QUIT = "Quit (q)"
    BUTTON_POINTS = "Plotted Points = 0"

    def __init__(self, symbol, symbol_size, map_line_size):
        # Spawn the graph window
        self.win = CustomGraphicsWindow()
        self.win.setWindowTitle('Trajectory Plotter')
        # self.win.setWindowFlag(Qt.FramelessWindowHint)
        # self.win.setWindowFlags(QtCore.Qt.WindowMinimizeButtonHint)
        # self.win.setWindowFlags(QtCore.Qt.WindowCloseButtonHint | QtCore.Qt.WindowMinimizeButtonHint)
        # self.win.setWindowFlags(self.win.windowFlags() | QtCore.Qt.CustomizeWindowHint)
        # self.win.setWindowFlags(self.win.windowFlags() & ~QtCore.Qt.WindowCloseButtonHint)

        # self.graph = self.win.addPlot()
        self.graph = pg.PlotItem()
        self.win.addItem(self.graph)

        self.graph.showGrid(x=True, y=True)
        self.graph.setXRange(-12, 12)
        self.graph.setYRange(-12, 12)
        self.graph.setAspectLocked(lock=True, ratio=1)

        bottom_row_layout = self.win.addLayout(row=2, col=0)
        button_layout = bottom_row_layout.addLayout(row=1, col=8)

        self.graph_drawer = GraphDrawer(
            self.graph, symbol, symbol_size, map_line_size)

        self.buttons_set = {}
        self.plot_count_item = self.add_static_button(
            self.BUTTON_POINTS, button_layout, 1, 1)

        self.add_button(self.BUTTON_RESET, button_layout,
                        self.graph_drawer.reset_all_plots, False, 1, 2)
        self.add_button(self.BUTTON_ODOM, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_ODOM, ODOM), True, 1, 3)
        self.add_button(self.BUTTON_GROUND_TRUTH, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_GROUND_TRUTH, GT), True, 1, 4)
        self.add_button(self.BUTTON_BELIEF, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_BELIEF, BEL), True, 1, 5)
        self.add_button(self.BUTTON_MAP, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_MAP, MAP), True, 1, 6)
        self.add_button(self.BUTTON_DIST, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_DIST, DIST), True, 1, 7)
        self.add_button(self.BUTTON_QUIT, button_layout,
                        self.close_app, False, 1, 8)

        label = pg.LabelItem(justify='right')
        self.win.addItem(label)

        # This needs to be called to process QT events (render the window)
        QtGui.QApplication.instance().processEvents()

    # Function run_loop
    #	This is the main loop of this ROS Node. It continuously processes incoming QT events (draw requests).
    #	It replaces the standard ROS spin function. We can only do this in Python, since rospy's spin is just a sleep.
    def run_loop(self):
        self.graph_drawer.run_loop()

    def add_button(self, name, parent_layout, button_callback, checkable, row, col):

        proxy = QtGui.QGraphicsProxyWidget()
        button = QtGui.QPushButton(name)
        proxy.setWidget(button)

        if(checkable == True):
            button.setCheckable(checkable)
            button.toggle()
            button.setChecked(True)

        parent_layout.addItem(proxy, row=row, col=col)

        button.clicked.connect(button_callback)
        self.buttons_set[name] = button

    def add_static_button(self, name, parent_layout, row, col):

        proxy = QtGui.QGraphicsProxyWidget()
        button = QtGui.QPushButton(name)
        proxy.setWidget(button)

        button.setEnabled(False)
        button.setStyleSheet("QPushButton { background-color: black }")

        parent_layout.addItem(proxy, row=row, col=col)

        return proxy.widget()

    def hide_show_callback(self, name, plot_type):
        if(self.buttons_set[name].isChecked()):
            self.graph_drawer.show_plot(plot_type)
        else:
            self.graph_drawer.hide_plot(plot_type)

    def close_app(self):
        rospy.signal_shutdown("User requested")


# Receives ROS messages and plots trajectories
class Traj_Plotter():
    def __init__(self, pos_tolerance, map_line_size):
        self.POS_TOLERANCE = pos_tolerance
        self.map_line_size = map_line_size

    def traj_update(self, data):
        if((data.plot_type >= 0) and (data.plot_type < 3)):
            if(math.hypot(graph_obj.graph_drawer.last_x[data.plot_type]-data.x,
                          graph_obj.graph_drawer.last_y[data.plot_type]-data.y) >= self.POS_TOLERANCE):
                graph_obj.graph_drawer.last_x[data.plot_type] = data.x
                graph_obj.graph_drawer.last_y[data.plot_type] = data.y

                graph_obj.graph_drawer.x_data[data.plot_type].append(data.x)
                graph_obj.graph_drawer.y_data[data.plot_type].append(data.y)
                graph_obj.graph_drawer.REFRESH_PLOT_FLAG[data.plot_type] = True

        graph_obj.plot_count_item.setText("Plotted Points = "
                                          + str(sum(len(graph_obj.graph_drawer.x_data[p]) for p in TRAJ_PLOT_TYPES)))
 
    def handle_map_init(self, req):
        if((len(req.sx) == len(req.ex)) and (len(req.sx) == len(req.sy)) and (len(req.sx) == len(req.ey))):
            for i in range(0, len(req.sx)):
                graph_obj.graph_drawer.map_x_datasets.append(
                    [req.sx[i], req.ex[i]])
                graph_obj.graph_drawer.map_y_datasets.append(
                    [req.sy[i], req.ey[i]])

                graph_obj.graph_drawer.REFRESH_MAP_FLAG = True
            
            return "SUCCESS"

        else:
            print ("MAP INIT ERROR: Start and End points are not matched.")
        
            return "MAP INIT ERROR: Start and End points of lines do not matched."

    def handle_prob_dist(self, req):
        global graph_obj

        p = np.array(req.data).reshape(20, 20)
        graph_obj.graph_drawer.img.setImage(p)

        QtGui.QApplication.instance().processEvents()

        graph_obj.graph_drawer.graph.update()

        return "SUCCESS"

    def handle_plot_reset(self, req):
        global graph_obj

        graph_obj.graph_drawer.reset_all_plots()

        QtGui.QApplication.instance().processEvents()

        return EmptyResponse()

    def handle_map_reset(self, req):
        for plot in graph_obj.graph_drawer.map_plots:
            plot.clear()

        graph_obj.graph_drawer.map_plots = []

        graph_obj.graph_drawer.map_x_datasets = []
        graph_obj.graph_drawer.map_y_datasets = []

        QtGui.QApplication.instance().processEvents()

        graph_obj.graph_drawer.graph.update()

        return EmptyResponse()


    # Function node_setup
    #	This function sets up the ROS Node and registers the ROS services it provides
    def node_setup(self, symbol, symbol_size, map_line_size):
        # Create a global Graph_Drawer instance for everything to use/draw on
        # This is necessary because QTCore and QTGui does not like to be controlled from multiple threads
        # However, rospy spawns different threads to handle service requests and callbacks
        # To avoid rendering issues, we are using the queue system in Graph_Drawer for plot requests
        global graph_obj
        graph_obj = GraphWindow(symbol, symbol_size, map_line_size)
        # Start the ROS Node and register the services offered
        rospy.init_node('plot_path')

        # Subscribers
        rospy.Subscriber("/traj_points", PlotPoint,
                         self.traj_update, queue_size=300)

        # Services
        service_prob_dist = rospy.Service('plot_prob_dist',
                                          ProbDist,
                                          self.handle_prob_dist)
        service_reset_plot = rospy.Service('plot_reset',
                                          Empty,
                                          self.handle_plot_reset)
        service_reset_map = rospy.Service('map_reset',
                                          Empty,
                                          self.handle_map_reset)
        service_init_map = rospy.Service('map_init',
                                          MapInit,
                                          self.handle_map_init)

        # Kick off the main run loop in the Graph_Drawer instance
        graph_obj.run_loop()


if __name__ == '__main__':

    print("Python version: ", sys.version)

    symbol = 'o'
    symbol_size = 5
    position_tolerance = 0.03
    map_line_size = 1

    config_parser = ConfigParser()
    config_parser.read(expanduser("~") + "/catkin_ws/src/lab8/scripts/plotter_config.ini")

    config_symbol_size = config_parser.getint('PLOTTERCONFIG', 'symbol_size')
    config_position_tolerance = config_parser.getfloat(
        'PLOTTERCONFIG', 'position_tolerance')

    if ((config_symbol_size > 0) and (config_symbol_size <= 6)):
        symbol_size = config_symbol_size

    position_tolerance = config_position_tolerance

    print ("Parameters used:")
    print ("Symbol               : ", symbol)
    print ("Symbol size          : ", symbol_size)
    print ("Position Tolerance   : ", position_tolerance)
    print ("------------------\n")

    traj_plotter = Traj_Plotter(position_tolerance, map_line_size)
    traj_plotter.node_setup(
        symbol=symbol, symbol_size=symbol_size, map_line_size=map_line_size)
