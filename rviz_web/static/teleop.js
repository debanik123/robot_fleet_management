// Connect to ROS2
var connectStatus = false;
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
var options = {
    zone: document.getElementById('zone_joystick'),
    threshold: 0.1,
    position: { left: 50 + '%' },
    mode: 'static',
    size: 150,
    color: '#000000',
  };

const joystick = nipplejs.create(options);

joystick.on('start', function (_event, _nipple) {
    // Enable a function that runs continuously and send Twist messages
    timer = setInterval(function () {
        sendVelocities(linear_speed, angular_speed);
        // console.log(linear_speed, angular_speed);
      }, 25);
});

joystick.on('move', function (_event, nipple) {
    max_linear = 1.0; // m/s
    max_angular = 2.0; // rad/s
    max_distance = 75.0; // pixels;
    linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
    angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
});

joystick.on('end', function () {
    // Stop timer if it is enabled and set linear speed to zero to stop robot
    if (timer) {
        clearInterval(timer);
    }

    if (connectStatus)
        sendVelocities(0.0, 0.0);
});

function sendVelocities(linearVel, angularVel) {
    // Publish Twist message with calculated velocities
    const twist = new ROSLIB.Message({
        linear: { x: linearVel, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularVel }
    });

    cmdVelPublisher.publish(twist);
}
