import rosbag2_py
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import PoseStamped
import os 
def read_vehicle_bag_data(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    topics = ['/carla/ego_vehicle/odometry', '/carla/ego_vehicle/vehicle_control_cmd', '/carla/ego_vehicle/waypoints']
    topic_data = {topic: [] for topic in topics}

    while reader.has_next():
        topic_name, serialized_msg, t = reader.read_next()
        
        if topic_name in topics:
            if topic_name == '/carla/ego_vehicle/odometry':
                msg = deserialize_message(serialized_msg, Odometry)
                topic_data[topic_name].append((t, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                                               msg.pose.pose.orientation.x,  # Yaw angle
                                               msg.twist.twist.linear.x))  # Longitudinal velocity
            elif topic_name == '/carla/ego_vehicle/vehicle_control_cmd':
                msg = deserialize_message(serialized_msg, CarlaEgoVehicleControl)
                topic_data[topic_name].append((t, msg.throttle, msg.brake, msg.steer))
            elif topic_name == '/carla/ego_vehicle/waypoints':
                msg = deserialize_message(serialized_msg, Path)
                if len(msg.poses) > 0:
                    first_pose = msg.poses[0].pose
                    topic_data[topic_name].append((t, first_pose.position.x, first_pose.position.y, first_pose.position.z,
                                                   first_pose.orientation.x,  # Yaw
                                                   first_pose.orientation.w))  # Reference velocity
    
    return topic_data

def plot_vehicle_data(topic_data):
    if '/carla/ego_vehicle/odometry' in topic_data and '/carla/ego_vehicle/waypoints' in topic_data:
        odom_times, odom_x, odom_y, odom_z, odom_yaw, odom_long_vel = zip(*topic_data['/carla/ego_vehicle/odometry'])
        traj_times, traj_x, traj_y, traj_z, traj_yaw, traj_ref_vel = zip(*topic_data['/carla/ego_vehicle/waypoints'])
        
        plt.figure()
        plt.plot(odom_times, odom_x, label='Vehicle X')
        plt.plot(traj_times, traj_x, label='Waypoints X', linestyle='--')
        plt.legend()
        plt.xlabel('Time')
        plt.ylabel('X Position')
        plt.title('Vehicle X Position vs Waypoints')
        
        plt.figure()
        plt.plot(odom_times, odom_y, label='Vehicle Y')
        plt.plot(traj_times, traj_y, label='Waypoints Y', linestyle='--')
        plt.legend()
        plt.xlabel('Time')
        plt.ylabel('Y Position')
        plt.title('Vehicle Y Position vs Waypoints')
        
        plt.figure()
        plt.plot(odom_times, odom_yaw, label='Yaw Angle')
        plt.plot(traj_times, traj_yaw, label='Reference Yaw', linestyle='--')
        plt.legend()
        plt.xlabel('Time')
        plt.ylabel('Yaw Angle')
        plt.title('Yaw Angle vs Reference Yaw')
        
        plt.figure()
        plt.plot(odom_times, odom_long_vel, label='Longitudinal Velocity')
        plt.plot(traj_times, traj_ref_vel, label='Reference Velocity', linestyle='--')
        plt.legend()
        plt.xlabel('Time')
        plt.ylabel('Velocity')
        plt.title('Longitudinal Velocity vs Reference Velocity')

    if '/carla/ego_vehicle/vehicle_control_cmd' in topic_data:
        times, throttle, brake, steer = zip(*topic_data['/carla/ego_vehicle/vehicle_control_cmd'])
        
        plt.figure()
        plt.plot(times, throttle, label='Throttle')
        plt.xlabel('Time')
        plt.ylabel('Throttle')
        plt.title('Throttle Command Over Time')
        
        plt.figure()
        plt.plot(times, brake, label='Brake')
        plt.xlabel('Time')
        plt.ylabel('Brake')
        plt.title('Brake Command Over Time')
        
        plt.figure()
        plt.plot(times, steer, label='Steer')
        plt.xlabel('Time')
        plt.ylabel('Steering Angle')
        plt.title('Steering Command Over Time')
    
    plt.legend()
    plt.show()

# Read and plot the data
bag_path = "/home/swati/acados/my_code/ackerman_carla_ROS2_WS/multi_topic_bag"

data = read_vehicle_bag_data(bag_path)
plot_vehicle_data(data)
