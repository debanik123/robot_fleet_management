from flask import Flask, render_template
import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import PoseStamped
import tf2_ros
from std_msgs.msg import Bool

transform = None

class Ros2_node(Node):
    def __init__(self):
        super().__init__('ros2_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_pose_pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.map_to_base_scan_pub = self.create_publisher(PoseStamped, 'scan_pose', 10)
        self.timer = self.create_timer(0.9, self.update_latest_transform)
        self.timer_publisher = self.create_publisher(Bool, 'bool_timer', 10)

    
    def update_latest_transform(self):
        global transform

        msg = Bool()
        msg.data = True
        self.timer_publisher.publish(msg)

        try:
            
            
            robot_pose = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time().to_msg())
            robot_pose_msg = self.transform_to_pose_stamped(robot_pose)
            self.robot_pose_pose_pub.publish(robot_pose_msg)
            
            map_to_base_scan = self.tf_buffer.lookup_transform('map', 'base_scan', rclpy.time.Time().to_msg())
            map_to_base_scan_msg = self.transform_to_pose_stamped(map_to_base_scan)
            self.map_to_base_scan_pub.publish(map_to_base_scan_msg)
            
            
        except:
            pass
    
    def transform_to_pose_stamped(self, transform):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp.sec = transform.header.stamp.sec
        pose_msg.header.stamp.nanosec = transform.header.stamp.nanosec
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z
        pose_msg.pose.orientation.x = transform.transform.rotation.x
        pose_msg.pose.orientation.y = transform.transform.rotation.y
        pose_msg.pose.orientation.z = transform.transform.rotation.z
        pose_msg.pose.orientation.w = transform.transform.rotation.w
        return pose_msg
    

def publish_topic_data():
    rclpy.init()
    node = Ros2_node()
    rclpy.spin(node)


app = Flask(__name__)

@app.route('/')
def index():
    return render_template('main.html')

if __name__ == '__main__':
    thread = Thread(target=publish_topic_data)
    thread.start()
    app.run(host='0.0.0.0', port=5000,debug=True)
