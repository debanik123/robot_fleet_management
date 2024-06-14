const ros = new ROSLIB.Ros({
    url: 'wss://192.168.0.25:9090'  // Adjust the URL based on your ROS2 WebSocket server configuration
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

// Define the cmd_vel publisher
const cmdVelPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',  // Adjust the topic name based on your robot's configuration
    messageType: 'geometry_msgs/Twist'
    // needs to modify
});

export function sendVelocities(linearVel, angularVel) {
    // Publish Twist message with calculated velocities
    const twist = new ROSLIB.Message({
        linear: { x: linearVel, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularVel }
    });

    cmdVelPublisher.publish(twist);
}

