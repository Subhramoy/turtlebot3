#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
from std_msgs.msg import Float32

"""
uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
uint8 POWER_SUPPLY_STATUS_CHARGING=1
uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
uint8 POWER_SUPPLY_STATUS_FULL=4
uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
uint8 POWER_SUPPLY_HEALTH_GOOD=1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
uint8 POWER_SUPPLY_HEALTH_DEAD=3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
uint8 POWER_SUPPLY_HEALTH_COLD=6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 voltage
float32 current
float32 charge
float32 capacity
float32 design_capacity
float32 percentage
uint8 power_supply_status
uint8 power_supply_health
uint8 power_supply_technology
bool present
float32[] cell_voltage
string location
string serial_number
"""


class SimBateryStatusPublisher(object):

    def __init__(self):

        self.pub = rospy.Publisher('battery_state', BatteryState, queue_size=10)

        self.battery_input_value = 0.0
        rospy.Subscriber("battery_input", Float32, self.battery_input_callback)
        self.init_battery()
    
    def init_battery(self):

        self.bat_state_msg = BatteryState()

        header_msg = Header()

        self.bat_state_msg.header = header_msg

        # Used eample in this docs o create values: https://emanual.robotis.com/docs/en/platform/turtlebot3/topic_monitor/
        self.bat_state_msg.voltage = 12.2
        
        # In amperes
        self.NOMINAL_CURRENT = 1.0
        
        self.bat_state_msg.charge = 0.0
        self.bat_state_msg.capacity = 0.0
        self.bat_state_msg.design_capacity = 1.8       

        """
        uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
        uint8 POWER_SUPPLY_HEALTH_GOOD=1
        uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
        uint8 POWER_SUPPLY_HEALTH_DEAD=3
        uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
        uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
        uint8 POWER_SUPPLY_HEALTH_COLD=6
        uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
        uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
        """
        self.bat_state_msg.power_supply_health = 0

        """
        uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
        uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
        uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
        uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
        uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
        uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
        uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
        """
        self.bat_state_msg.power_supply_technology = 0

        self.bat_state_msg.present =True
        self.bat_state_msg.cell_voltage = [0.0]
        self.bat_state_msg.location = ""
        self.bat_state_msg.serial_number = "01"

        ### And these ar ethe only values that we will simulate:
        self.INIT_BATTERY_PERC = 100.0
        self.DELTA_BATTERY_CLICK = 20.0#0.01
        self.bat_state_msg.percentage = self.INIT_BATTERY_PERC
        
        # If < 0 is discharging, > 0 charging, 0 charged
        self.bat_state_msg.current = 0.0

        """
        uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
        uint8 POWER_SUPPLY_STATUS_CHARGING=1
        uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
        uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
        uint8 POWER_SUPPLY_STATUS_FULL=4
        """
        self.bat_state_msg.power_supply_status = 0

    def update_battery_status(self):
        """
        Here we simulate the battery and its status
        """

        if self.battery_input_value != 0.0:

            self.bat_state_msg.current = self.NOMINAL_CURRENT
            self.bat_state_msg.percentage = self.bat_state_msg.percentage + self.DELTA_BATTERY_CLICK
            # POWER_SUPPLY_STATUS_CHARGING=1
            self.bat_state_msg.power_supply_status = 1
            

            if self.bat_state_msg.percentage >= self.INIT_BATTERY_PERC:
                self.bat_state_msg.percentage = self.INIT_BATTERY_PERC
                # POWER_SUPPLY_STATUS_FULL=4
                self.bat_state_msg.power_supply_status = 4
                rospy.loginfo("Plugged-Robot is Fully Charged....")                
            else:
                rospy.loginfo("Plugged-Robot is Charging....")

        else:

            self.bat_state_msg.current = -1.0 * self.NOMINAL_CURRENT
            self.bat_state_msg.percentage = self.bat_state_msg.percentage - self.DELTA_BATTERY_CLICK
            # POWER_SUPPLY_STATUS_DISCHARGING=2
            self.bat_state_msg.power_supply_status = 2

            if self.bat_state_msg.percentage <= 0.0:
                self.bat_state_msg.percentage = 0.0
                # POWER_SUPPLY_STATUS_NOT_CHARGING=3
                self.bat_state_msg.power_supply_status = 3
                rospy.loginfo("Unplugged-Robot is Completelly DISCHARGED....")
            else:
                rospy.loginfo("Unplugged-Robot is Discharging....")

        self.pub.publish(self.bat_state_msg)

    def get_battery_info(self):


        status = self.bat_state_msg.power_supply_status
        percentage = self.bat_state_msg.percentage

        """
        uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
        uint8 POWER_SUPPLY_STATUS_CHARGING=1
        uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
        uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
        uint8 POWER_SUPPLY_STATUS_FULL=4
        """

        if status == 0:
            info = "UNKNOWN"
        elif status == 1:
            info = "CHARGING"
        elif status == 2:
            info = "DISCHARGING"
        elif status == 3:
            info = "DISCHARGED"
        elif status == 4:
            info = "FULL"
        else:
            assert False, "This value of status shouldnt be happening ="+str(status)

        return percentage, info



    def battery_input_callback(self, msg):
        """
        This is called when we plug or unplug the simulated robot
        This value can onlu be posiive, we cant extract energy form the robot plug.
        """
        rospy.loginfo("RobotInput Old = "+str(self.battery_input_value))
        self.battery_input_value = abs(msg.data)
        rospy.loginfo("RobotInput New = "+str(self.battery_input_value))


    def start_loop(self):

        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            self.update_battery_status()
            percentage, info = self.get_battery_info()
            rospy.loginfo("percentage="+str(percentage)+",info="+str(info))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('simulated_battery_status_turtlebot3', anonymous=True, log_level=rospy.DEBUG)
    sim_bat_obj = SimBateryStatusPublisher()
    sim_bat_obj.start_loop()