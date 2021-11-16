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

class PresetWidget(QWidget):
    def __init__(self, parent=None):
        super(PresetWidget, self).__init__(parent)
        self.button = QPushButton('  Go to Position  ')
        self.button.setFixedHeight(30)
        lay = QHBoxLayout(self)
        lay.addWidget(self.button, alignment=Qt.AlignRight)
        lay.setContentsMargins(0, 0, 0, 0)

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

class PositionControl(Plugin):
    def __init__(self, context):
        super(PositionControl, self).__init__(context)
        self.setObjectName('PositionControl')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        self.pkg_dir = rospkg.RosPack().get_path('rqt_position_control')
        self.presets_file = os.path.join(self.pkg_dir, 'config', 'presets.yaml')
        ui_file = os.path.join(self.pkg_dir, 'resource', 'position_control.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('PositionControlUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.lower_limits = rospy.get_param('/joint_space_controller/lower_limit')
        self.upper_limits = rospy.get_param('/joint_space_controller/upper_limit')

        self.presets = {}
        self._widget.preset_view.setContextMenuPolicy(Qt.CustomContextMenu)
        self._widget.preset_view.customContextMenuRequested.connect(self._on_preset_menu)
        self._load_presets()
        self._widget.reload.clicked.connect(self._load_presets)

        self._dragging = False
        control_view = self._widget.control_view
        control_model = QStandardItemModel(control_view)
        control_view.setModel(control_model)
        #for idx, joint_name in enumerate(sorted(self.presets.values()[0]['joint_positions'].keys())):
        for idx in range(len(self.lower_limits)):
            item = QStandardItem('J{}'.format(idx + 1))
            item.setSizeHint(QSize(0, 30))
            control_model.appendRow(item)
            widget = ControlWidget(parent=self._widget)
            widget.slider.setMinimum(self.lower_limits[idx] * 100)
            widget.slider.setMaximum(self.upper_limits[idx] * 100)
            widget.slider.sliderMoved.connect(self._on_slider)
            widget.lower_limit.setText(str(self.lower_limits[idx]))
            widget.upper_limit.setText(str(self.upper_limits[idx]))
            control_view.setIndexWidget(item.index(), widget)
        self._widget.send_all.clicked.connect(self._on_command)
        self._widget.reset.clicked.connect(self._on_reset)
        self._widget.add_preset.clicked.connect(self._on_add_preset)

        self.pub_goal = rospy.Publisher('/joint_space_controller/goal', JointState, queue_size=1)
        self.sub_pos = rospy.Subscriber('/joint_states', JointState, self._on_joint_state, queue_size=1)

        self._controller_lister = ControllerLister('/controller_manager')
        # Timer for running controller updates
        self._update_ctrl_list_timer = QTimer(self)
        self._update_ctrl_list_timer.setInterval(1000.0)
        self._update_ctrl_list_timer.timeout.connect(self._update_controllers)
        self._update_ctrl_list_timer.start()

    def shutdown_plugin(self):
        self.sub_pos.unregister()
        self._update_ctrl_list_timer.stop()

    def _load_presets(self):
        self.presets = rosparam.load_file(self.presets_file)[0][0]

        preset_view = self._widget.preset_view
        preset_model = QStandardItemModel(preset_view)
        preset_view.setModel(preset_model)
        for name, preset in sorted(self.presets.items()):
            item = QStandardItem(name)
            item.setSizeHint(QSize(0, 30))
            preset_model.appendRow(item)
            widget = PresetWidget(parent=self._widget)
            widget.button.clicked.connect((lambda pr: lambda: self._on_preset(pr))(preset))
            preset_view.setIndexWidget(item.index(), widget)

    def _store_presets(self):
        with open(self.presets_file, 'w') as outfile:
            yaml.dump(self.presets, outfile, default_flow_style=False)
        self._load_presets()

    def _all_sliders(function):
        def wrapper(self, *args, **kwargs):
            control_view = self._widget.control_view
            model = control_view.model()
            for i in range(model.rowCount()):
                item = model.item(i)
                widget = control_view.indexWidget(item.index())
                function(self, i, widget, *args, **kwargs)
        return wrapper

    def _set_dragging(self, dragging):
        self._dragging = dragging
        self._widget.reset.setEnabled(dragging)

    def _on_preset(self, preset):
        goal = JointState()
        for name, position in sorted(preset['joint_positions'].items()):
            #goal.name.append(name)
            goal.position.append(position)
        self.pub_goal.publish(goal)
        self._set_dragging(False)

    def _on_preset_menu(self, pos):
        selectionModel = self._widget.preset_view.selectionModel()
        rows = selectionModel.selectedRows()
        menu = QMenu(self._widget.preset_view)

        if len(rows) > 0:
            action_delete = menu.addAction('Delete')
            action = menu.exec_(self._widget.preset_view.mapToGlobal(pos))

            if action == action_delete:
                for row in rows:
                    self.presets.pop(row.data())
                self._store_presets()

    def _on_add_preset(self):
        text, ok = QInputDialog.getText(self._widget, 'Add Preset', 'Enter Preset Name:')
        if ok:
            preset = JointState()
            self._get_goal_positions(preset)
            self.presets[str(text)] = {
                'joint_positions': {idx: position for idx, position in enumerate(preset.position)}
            }
            self._store_presets()

    def _on_slider(self):
        self._set_dragging(True)

    def _on_reset(self):
        self._set_dragging(False)

    def _on_command(self):
        goal = JointState()
        self._get_goal_positions(goal)
        self.pub_goal.publish(goal)
        self._set_dragging(False)

    @_all_sliders
    def _get_goal_positions(self, i, widget, goal):
        goal.position.append(widget.slider.value() / 100.0)

    @_all_sliders
    def _on_joint_state(self, i, widget, data):
        if not self._dragging:
            # Hack to bypass finger joint which is returned at index 0
            widget.slider.setValue(data.position[i + 1] * 100.0)

    def _update_controllers(self):
        controllers = [controller for controller in self._controller_lister() \
                       if controller.name == 'joint_space_controller' and controller.state == 'running']
        running = len(controllers) > 0
        self._widget.setEnabled(running)
        self._widget.controller_missing.setVisible(not running)
