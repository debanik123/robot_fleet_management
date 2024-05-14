// Connect to ROS2
import '../static/lib/nipplejs.js';
export const nipplejs = window.nipplejs;
let joy_offset_x = "50%";
let joy_offset_y = "85%";
let targetLinearVel = 0;
let targetAngularVel = 0;

// Constants for acceleration and deceleration
const accelerationRate = 0.01;
const decelerationRate = 0.01;

// Initialize current velocities
let currentLinearVel = 0;
let currentAngularVel = 0;


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

function sendVelocities(linearVel, angularVel) {
    // Publish Twist message with calculated velocities
    const twist = new ROSLIB.Message({
        linear: { x: linearVel, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularVel }
    });

    cmdVelPublisher.publish(twist);
}

const joystickContainer = document.getElementById('joystick');
function makeJoystick(){
	return nipplejs.create({
		zone: joystickContainer,
		mode: 'static',
		position: {
			left: joy_offset_x,
			top: joy_offset_y 
		},
		size: 150,
		threshold: 0.1,
		color: 'black',
		restOpacity: 0.7
	})
}

let joystick = makeJoystick();

function addJoystickListeners(){
	joystick.on('move', onJoystickMove);
	joystick.on('touchmove', onJoystickMove);
	joystick.on('end', onJoystickEnd);
	joystick.on('touchend', onJoystickEnd);
}

function onJoystickMove(event, data) {
    // console.log('Joystick ', data);
    const maxLinearVel = 0.7;
	const maxAngularVel = 1.3;
	const force = Math.min(Math.max(data.force, 0.0), 1.0);

	targetLinearVel = maxLinearVel * Math.sin(data.angle.radian) * force;
	targetAngularVel = - maxAngularVel * Math.cos(data.angle.radian) * force;

	// Adjust current velocities based on acceleration
    if (targetLinearVel > currentLinearVel) {
        currentLinearVel = Math.min(currentLinearVel + accelerationRate, targetLinearVel);
    } else if (targetLinearVel < currentLinearVel) {
        currentLinearVel = Math.max(currentLinearVel - decelerationRate, targetLinearVel);
    }

    if (targetAngularVel > currentAngularVel) {
        currentAngularVel = Math.min(currentAngularVel + accelerationRate, targetAngularVel);
    } else if (targetAngularVel < currentAngularVel) {
        currentAngularVel = Math.max(currentAngularVel - decelerationRate, targetAngularVel);
    }

    // console.log('targetLinearVel:', targetLinearVel, 'targetAngularVel: ', targetAngularVel);

	// console.log('Current linear velocity:', currentLinearVel);
    // console.log('Current angular velocity:', currentAngularVel);
	sendVelocities(currentLinearVel, currentAngularVel);


    // console.log('Joystick force:', data.force);
}

function onJoystickEnd(event) {
	targetLinearVel = 0;
	targetAngularVel = 0;

	currentLinearVel = 0;
    currentAngularVel = 0;

    // Print velocities when joystick ends
    console.log('Joystick ended');
    console.log('Current linear velocity:', currentLinearVel);
    console.log('Current angular velocity:', currentAngularVel);
	sendVelocities(currentLinearVel, currentAngularVel);

}

addJoystickListeners();