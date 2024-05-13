// Connect to ROS2
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

// Define the cmd_vel publisher
const cmdVelPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',  // Adjust the topic name based on your robot's configuration
    messageType: 'geometry_msgs/Twist'
});

// Create a nipple joystick
const options = {
    zone: document.getElementById('joystick'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: 'blue',
    size: 200
};

const joystick = nipplejs.create(options);

// Event listener for joystick movement
joystick.on('move', function (event, nipple) {
    // Normalize the joystick position to [-1, 1] range
    const x = nipple.position.x / 100 - 0.5;
    const y = 0.5 - nipple.position.y / 100;

    // Calculate linear and angular velocities
    var max_linear_vel = 1.5;
    var max_angular_vel = 2.0;

    const linearVel = y * max_linear_vel;
    const angularVel = x * max_angular_vel;

    // Send velocities to robot control
    sendVelocities(linearVel, angularVel);
});

// Event listener for joystick release
joystick.on('end', function () {
    // Stop robot movement
    sendVelocities(0, 0);
});

function sendVelocities(linearVel, angularVel) {
    // Publish Twist message with calculated velocities
    const twist = new ROSLIB.Message({
        linear: { x: linearVel, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularVel }
    });

    cmdVelPublisher.publish(twist);
}
