#!/usr/bin/env python

import os
import rospy
import rospkg
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem

from sensor_msgs.msg import JointState


class JointStates(Plugin):
    class Signals(QObject):
        pos = Signal(JointState)

    def __init__(self, context):
        super(JointStates, self).__init__(context)
        self.setObjectName('JointStates')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        self.pkg_dir = rospkg.RosPack().get_path('moma_joint_states_display_gui')
        ui_file = os.path.join(self.pkg_dir, 'resource', 'joint_states.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('JointStatesUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.controller_name = rospy.get_param('/moma_joint_states_display_gui/controller_name')
        # To avoid redundancy, fetch all parameters that the controllers already have directly from them
        self.joint_names = rospy.get_param('/{}/joint_names'.format(self.controller_name))
        self.lower_limits = rospy.get_param('/{}/lower_limit'.format(self.controller_name))
        self.upper_limits = rospy.get_param('/{}/upper_limit'.format(self.controller_name))
        self.safety_margin = rospy.get_param('/{}/safety_margin'.format(self.controller_name))
        self.joints = dict(zip(self.joint_names, zip(self.lower_limits, self.upper_limits)))

        self._state_widgets = dict()
        states_table = self._widget.states_table
        states_table.setHorizontalHeaderLabels(['Joint', 'Lower Limit', 'Position', 'Upper Limit'])
        states_table.setRowCount(len(self.joint_names))
        for idx, (name, lower_limit, upper_limit) in enumerate(zip(self.joint_names, self.lower_limits, self.upper_limits)):
            states_table.setItem(idx, 0, QTableWidgetItem(name))
            states_table.setItem(idx, 1, QTableWidgetItem(self._format(lower_limit)))
            self._state_widgets[name] = QTableWidgetItem()
            states_table.setItem(idx, 2, self._state_widgets[name])
            states_table.setItem(idx, 3, QTableWidgetItem(self._format(upper_limit)))

        self.signals = JointStates.Signals()
        self.signals.pos.connect(self._on_joint_state)
        self.sub_pos = rospy.Subscriber('/joint_states', JointState, lambda msg: self.signals.pos.emit(msg), queue_size=1)

    def shutdown_plugin(self):
        self.sub_pos.unregister()

    def _format(self, number):
        return "{:.2f}".format(number)

    def _on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self._state_widgets:
                self._state_widgets[name].setText(self._format(position))

                margin = min(position - self.joints[name][0], self.joints[name][1] - position)
                # Fading to yellow if between safety_margin and twice safety_margin
                color_brightness_blue = max(min((margin - self.safety_margin)/self.safety_margin, 1) * 255, 0)
                color_brightness_green = 255 if color_brightness_blue > 0 else 0
                # Blinking red if below safety_margin
                if margin <= self.safety_margin:
                    color_brightness_blue = 127 * (((time.time() * 10) % 20) > 10)
                    color_brightness_green = color_brightness_blue
                self._state_widgets[name].setBackground(QColor(255, color_brightness_green, color_brightness_blue))
                #self._state_widgets[name].setBackground(Qt.white)
