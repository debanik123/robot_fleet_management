// Connect to ROS2
var joy_start_point = undefined;
var joy_delta = undefined;

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

const joystick = document.getElementById('joystick');
const handle = document.getElementById('handle');

let isDragging = false;

// Event listeners for mouse/touch interactions
handle.addEventListener('mousedown', startDrag);
handle.addEventListener('touchstart', startDrag);

document.addEventListener('mousemove', drag);
document.addEventListener('touchmove', drag);

document.addEventListener('mouseup', endDrag);
document.addEventListener('touchend', endDrag);

function startDrag(event) {
    var rect = joystick.getBoundingClientRect();
    const { clientX, clientY } = event.touches ? event.touches[0] : event;
    joy_start_point = {
		x: clientX- rect.left,
		y: clientY - rect.top
	};
    event.preventDefault();
}

function drag(event) {
    if (joy_start_point === undefined) return;
    var bounds = joystick.getBoundingClientRect();
	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	joy_delta = {
		x: joy_start_point.x - clientX,
		y: joy_start_point.y - clientY,
	}

    let x = event.clientX - bounds.left;
    let y = event.clientY - bounds.top;

    // Ensure handle stays inside joystick boundary
    x = Math.max(0, Math.min(x, bounds.width));
    y = Math.max(0, Math.min(y, bounds.height));

    handle.style.left = `${x}px`;
    handle.style.top = `${y}px`;
    let resolution = 0.05;
    let linearVel = Math.sqrt(x * x + y * y)*resolution;
    
    let yaw = Math.atan2(y, -x);
    let max_angular = 1.5;
    let angular_speed = max_angular * Math.sin(yaw);

    console.log(x, y, angular_speed, linearVel);
    // Send velocities to robot control
    // sendVelocities(linearVel, angularVel);
}

function endDrag() {
    isDragging = false;
    // Reset handle to center position
    joy_start_point = undefined;
    joy_delta = undefined;

    handle.style.left = '50%';
    handle.style.top = '50%';

    // Stop robot movement
    sendVelocities(0, 0);
}

function sendVelocities(linearVel, angularVel) {
    // Publish Twist message with calculated velocities
    const twist = new ROSLIB.Message({
        linear: { x: linearVel, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularVel }
    });

    cmdVelPublisher.publish(twist);
}
