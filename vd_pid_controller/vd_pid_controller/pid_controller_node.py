import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vd_msgs.msg import VDControlCMD, VDstate, VDtraj
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class PIDPublisher(Node):

    def __init__(self):
        super().__init__('pid_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort, similar to TCP no delay
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)  # Equivalent to queue_size=1)

        self.cmd_publisher = self.create_publisher(VDControlCMD, 'pid_control_cmd', qos_profile)
        self.state_sub = self.create_subscription(VDstate,'vd_state',self.state_cb, qos_profile)
        self.state_sub  # prevent unused variable warning
        self.traj_sub = self.create_subscription(VDtraj,'vd_traj',self.traj_cb, qos_profile)
        self.traj_sub  # prevent unused variable warning
        
        self.ref_vel = 0
        self.current_vel = 0
        self.Kp = 300
        self.Ki = 1
        self.Kd = 0
        self.cumm_error = 0
        self.last_error = 0
        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    # def timer_callback(self):
    #     msg = VDControlCMD()
    #     msg.velocity = 30
    #     msg.steering_angle = 0.01
    #     self.cmd_publisher(msg)

    def state_cb(self, msg):
        self.current_vel = msg.vx
        #apply PID
        

    def traj_cb(self, msg):
        self.ref_vel = msg.poses[0].vf
        error = self.ref_vel - self.current_vel
        torque_in = self.Kp * error + self.Ki * self.cumm_error + self.Kd * (error - self.last_error)
        self.cumm_error += error
        self.last_error = error

        cmd_msg = VDControlCMD()
        cmd_msg.velocity = torque_in
        cmd_msg.steering_angle = 0.0
        self.cmd_publisher.publish(cmd_msg)
        



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