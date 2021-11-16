#!/usr/bin/env python

import os
import rospy
import rospkg
import rosparam
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMenu, QInputDialog, QHBoxLayout, QPushButton, QSlider, QLabel
from python_qt_binding.QtCore import Qt, QTimer, QSize
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem

from controller_manager_msgs.utils import ControllerLister

from sensor_msgs.msg import JointState

class ControlWidget(QWidget):
    def __init__(self, parent=None):
        super(ControlWidget, self).__init__(parent)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFixedHeight(30)
        self.lower_limit = QLabel()
        self.upper_limit = QLabel()
        lay = QHBoxLayout(self)
        lay.addWidget(self.lower_limit, alignment=Qt.AlignRight)
        lay.addWidget(self.slider)
        lay.addWidget(self.upper_limit, alignment=Qt.AlignLeft)
        lay.setContentsMargins(30, 0, 0, 0)

class VelocityControl(Plugin):
    def __init__(self, context):
        super(VelocityControl, self).__init__(context)
        self.setObjectName('VelocityControl')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        self.pkg_dir = rospkg.RosPack().get_path('rqt_velocity_control')
        ui_file = os.path.join(self.pkg_dir, 'resource', 'velocity_control.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('VelocityControlUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.lower_limits = rospy.get_param('/joint_velocity_controller/lower_limit')
        self.upper_limits = rospy.get_param('/joint_velocity_controller/upper_limit')
        self.max_velocity = rospy.get_param('/joint_velocity_controller/max_velocity')

        control_view = self._widget.control_view
        control_model = QStandardItemModel(control_view)
        control_view.setModel(control_model)
        for idx in range(len(self.lower_limits)):
            item = QStandardItem('J{}'.format(idx + 1))
            item.setSizeHint(QSize(0, 30))
            control_model.appendRow(item)
            widget = ControlWidget(parent=self._widget)
            # Slider returns only int, so scale it up
            widget.slider.setMinimum(-self.max_velocity * 100)
            widget.slider.setMaximum(self.max_velocity * 100)
            widget.slider.valueChanged.connect((lambda joint: lambda value: self._on_slider(joint, value))(idx))
            widget.slider.sliderReleased.connect((lambda joint: lambda: self._on_slider(joint, 0))(idx))
            widget.lower_limit.setText(str(-self.max_velocity))
            widget.upper_limit.setText(str(self.max_velocity))
            control_view.setIndexWidget(item.index(), widget)

        self.pub_goal = rospy.Publisher('/joint_velocity_controller/goal', JointState, queue_size=1)

        self._controller_lister = ControllerLister('/controller_manager')
        # Timer for running controller updates
        self._update_ctrl_list_timer = QTimer(self)
        self._update_ctrl_list_timer.setInterval(1000.0)
        self._update_ctrl_list_timer.timeout.connect(self._update_controllers)
        self._update_ctrl_list_timer.start()

    def shutdown_plugin(self):
        self._update_ctrl_list_timer.stop()

    def _on_slider(self, joint, value):
        sender = self.sender()
        # Prevent mouse wheel and bar clicking
        if not sender.isSliderDown():
            value = 0
            sender.setValue(value)
        goal = JointState()
        goal.velocity = [0] * (len(self.lower_limits))
        # Slider returns only int, so scale it
        goal.velocity[joint] = value / 100.0
        self.pub_goal.publish(goal)

    def _update_controllers(self):
        controllers = [controller for controller in self._controller_lister() \
                       if controller.name == 'joint_velocity_controller' and controller.state == 'running']
        running = len(controllers) > 0
        self._widget.setEnabled(running)
        self._widget.controller_missing.setVisible(not running)
