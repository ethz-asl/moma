#!/usr/bin/env python

import os
import rospy
import rospkg
import rosparam
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QPushButton
from python_qt_binding.QtCore import Qt, QTimer, QSize
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem

from controller_manager_msgs.utils import ControllerLister

from sensor_msgs.msg import JointState

class CustomWidget(QWidget):
    def __init__(self, parent=None):
        super(CustomWidget, self).__init__(parent)
        self.button = QPushButton('  Go to Position  ')
        self.button.setFixedHeight(30)
        lay = QHBoxLayout(self)
        lay.addWidget(self.button, alignment=Qt.AlignRight)
        lay.setContentsMargins(0, 0, 0, 0)

class PositionSetpoint(Plugin):
    def __init__(self, context):
        super(PositionSetpoint, self).__init__(context)
        self.setObjectName('PositionSetpoint')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        pkg_dir = rospkg.RosPack().get_path('rqt_position_setpoint')
        ui_file = os.path.join(pkg_dir, 'resource', 'position_setpoint.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('PositionSetpointUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.presets = rosparam.load_file(os.path.join(pkg_dir, 'config', 'presets.yaml'))[0][0]

        preset_view = self._widget.preset_view
        model = QStandardItemModel(preset_view)
        preset_view.setModel(model)
        for name, preset in self.presets.items():
            item = QStandardItem(name)
            item.setSizeHint(QSize(0, 30))
            model.appendRow(item)
            widget = CustomWidget(parent=self._widget)
            widget.button.clicked.connect((lambda pr: lambda: self._on_preset(pr))(preset))
            preset_view.setIndexWidget(item.index(), widget)

        self.pub_goal = rospy.Publisher('/joint_space_controller/goal', JointState, queue_size=1)

        self._controller_lister = ControllerLister('/controller_manager')
        # Timer for running controller updates
        self._update_ctrl_list_timer = QTimer(self)
        self._update_ctrl_list_timer.setInterval(1000.0)
        self._update_ctrl_list_timer.timeout.connect(self._update_controllers)
        self._update_ctrl_list_timer.start()

    def shutdown_plugin(self):
        pass

    def _on_preset(self, preset):
        goal = JointState()
        for name, position in sorted(preset['joint_positions'].items()):
            #goal.name.append(name)
            goal.position.append(position)
        self.pub_goal.publish(goal)

    def _update_controllers(self):
        controllers = [controller for controller in self._controller_lister() \
                       if controller.name == 'joint_space_controller' and controller.state == 'running']
        self._widget.controller_missing.setVisible(len(controllers) < 1)
