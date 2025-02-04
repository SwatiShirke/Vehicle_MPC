import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vd_msgs.msg import VDControlCMD, VDstate, VDtraj
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
class PIDPublisher(Node):

    def __init__(self):
        super().__init__('pid_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort, similar to TCP no delay
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)  # Equivalent to queue_size=1)
        
        self.cmd_publisher = self.create_publisher(CarlaEgoVehicleControl, "/carla/ego_vehicle/vehicle_control_cmd", qos_profile)
        self.mpc_sub = self.create_subscription(VDControlCMD,"/mpc/control_cmd",self.mpc_cb, qos_profile)
        self.odom_sub = self.create_subscription(Odometry,'/carla/ego_vehicle/odometry',self.odom_cb, qos_profile)
        self.odom_sub  # prevent unused variable warning
        
        self.ref_vel = 0
        self.current_vel = 0
        self.Kp = 1
        self.Ki = 0.5
        self.Kd = 0
        self.cumm_error = 0
        self.last_error = 0
        self.current_Yaw_Rate = 0
        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def mpc_cb(self, msg):
        Carla_Control_msg = CarlaEgoVehicleControl()
        error = msg.yaw_rate - self.current_Yaw_Rate
        steer_angle = self.Kp * error + self.Ki *  self.cumm_error + self.Kd * (error - self.last_error)
        self.cumm_error += error
        self.last_error =  error 

        if msg.accel >=0:
            Carla_Control_msg.throttle = msg.accel
            Carla_Control_msg.steer = steer_angle
            Carla_Control_msg.brake = 0.0
        else:
            Carla_Control_msg.throttle = 0.0
            Carla_Control_msg.steer = steer_angle
            Carla_Control_msg.brake = -1 * msg.accel        
        self.cmd_publisher.publish(Carla_Control_msg)

    def odom_cb(self, msg):
        self.current_Yaw_Rate = msg.twist.twist.angular.x 




def main(args=None):
    rclpy.init(args=args)

    pid_publisher = PIDPublisher()

    rclpy.spin(pid_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()