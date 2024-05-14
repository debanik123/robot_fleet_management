const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // Adjust the URL based on your ROS2 WebSocket server configuration
});

ros.on('connection', function () {
    console.log('Connected to ROS2');
});

ros.on('error', function (error) {
    console.error('Error connecting to ROS2:', error);
});

ros.on('close', function () {
    console.log('Disconnected from ROS2');
});

// ROS map subscription (adjust topic name and message type if needed)
export var mapview = new ROSLIB.Topic({
    ros: ros,
    name: '/map', // Subscribe to all map topics (replace with specific topic if needed)
    messageType: 'nav_msgs/OccupancyGrid'  // Adjust based on your map message type
  });
  
export var pathSubscriber = new ROSLIB.Topic({
    ros : ros,
    name : '/plan',
    messageType : 'nav_msgs/Path'
  });
  
export var robot_poseSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_pose',
    messageType: 'geometry_msgs/PoseStamped'
  });
  
export var scan_pose_Subscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/scan_pose',
    messageType: 'geometry_msgs/PoseStamped'
  });

export var scanSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/scan',
    messageType: 'sensor_msgs/msg/LaserScan'
  });

// Define the cmd_vel publisher
const cmdVelPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',  // Adjust the topic name based on your robot's configuration
    messageType: 'geometry_msgs/Twist'
});

export var goalPosePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/PoseStamped'
});

export function sendVelocities(linearVel, angularVel) {
    // Publish Twist message with calculated velocities
    const twist = new ROSLIB.Message({
        linear: { x: linearVel, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularVel }
    });

    cmdVelPublisher.publish(twist);
}

