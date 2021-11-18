import os
import rospy
import rospkg
from std_srvs.srv import Trigger
from ur_msgs.srv import SetSpeedSliderFraction
from controller_manager_msgs.srv import ListControllers, SwitchController

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from iris_sami.srv import NoArguments

class UR10ePlugin(Plugin):

    def __init__(self, context):
        super(UR10ePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('UR10e Plugin')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('iris_ur10e'), 'resource', 'UR10ePlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('UR10e Plugin UI')

        # Gripper services
        self._widget.grip.clicked[bool].connect(self.gripper_button_clicked)
        self._widget.release.clicked[bool].connect(self.release_button_clicked)
        # Controller switcher
        self._widget.switch_controller.clicked[bool].connect(self.switch_controller_button_clicked)
        # Resend robot program
        self._widget.resend_program.clicked[bool].connect(self.resend_program_button_clicked)
        # Zero FT sensor
        self._widget.zero_ft_sensor.clicked[bool].connect(self.zero_ft_sensor_button_clicked)
        # Set Speed Slider
        self._widget.set_speed_slider.clicked[bool].connect(self.set_speed_slider_button_clicked)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Add widget to the user interface
        self._widget.setWindowTitle('UR10e Plugin')
        context.add_widget(self._widget)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass


    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


    def gripper_button_clicked(self):
        # Grip service caller
        rospy.wait_for_service('iris_sami/grip')
        try:
            gripServ = rospy.ServiceProxy('iris_sami/grip', NoArguments)
            resp = gripServ().feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e

        # self._widget.com_response.setPlainText(resp)


    def release_button_clicked(self):
        # Release service caller
        rospy.wait_for_service('iris_sami/release')
        try:
            releaseServ = rospy.ServiceProxy('iris_sami/release', NoArguments)
            resp = releaseServ().feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e

        # self._widget.com_response.setPlainText(resp)


    def switch_controller_button_clicked(self):
        controllers = ['joint_group_vel_controller', 'scaled_pos_joint_traj_controller']
        status = ''

        # Get active controller
        active_controller = ''
        rospy.wait_for_service('controller_manager/list_controllers')
        try:
            listServ = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
            resp = listServ()
            for controller in resp.controller:
                if controller.name in controllers and controller.state == "running":
                    active_controller = controller.name
        except rospy.ServiceException as e:
            status = "Service call failed: %s" % e

        # Switch Controllers
        if active_controller:
            controllers.remove(active_controller)

            rospy.wait_for_service('controller_manager/switch_controller')
            try:
                switchServ = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
                resp = switchServ(start_controllers=controllers,
                                stop_controllers=[active_controller],
                                strictness=1,
                                start_asap=False,
                                timeout=2.0)
                status = "Switched Controllers: " + controllers[0] + " is active"
            except rospy.ServiceException as e:
                status = "Service call failed: %s" % e
        else:
            status = "No controller is active. There might be a problem with the driver"
        
        # self._widget.com_response.setPlainText(status)


    def resend_program_button_clicked(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/resend_robot_program', timeout=1)
            resend = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program', Trigger)
            if resend().success:
                resp = "Program sucessfyly sent to robot"
            else:
                resp = "Error un resending program to robot"
        except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
            resp = "Service call failed: %s" % e

        # self._widget.com_response.setPlainText(resp)


    def zero_ft_sensor_button_clicked(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=1)
            zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
            if zero().success:
                resp = "FT Sensor sucessfully tared"
            else:
                resp = "Error in taring FT Sensor"
        except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
            resp = "Service call failed: %s" % e

        # self._widget.com_response.setPlainText(resp)


    def set_speed_slider_button_clicked(self):
        speed_slider = self._widget.speed_slider.text()
        speed_slider = float(speed_slider)
        
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
        try:
            set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
            resp = set_speed(speed_slider)
            resp = resp.success
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog