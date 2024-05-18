// Connect to ROS2
import './lib/nipplejs.js';
// import './robo_utilities.js';
// import { sendVelocities } from './cmd_vel_publisher.js';
// import { sendVelocities } from './src/robo_utilities.js';
// const { createCmdVelPublisher } = require('./cmd_vel_publisher'); // Import the function
// const publishVelocity = createCmdVelPublisher('robot_velocity_publisher', '/cmd_vel'); // Create publisher
 // Publish velocity (1 m/s forward, 0.5 rad/s rotation)

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

    console.log('targetLinearVel:', targetLinearVel, 'targetAngularVel: ', targetAngularVel);

	// console.log('Current linear velocity:', currentLinearVel);
    // console.log('Current angular velocity:', currentAngularVel);
	// sendVelocities(currentLinearVel, currentAngularVel);
	// publishVelocity(currentLinearVel, 0.0, currentAngularVel);


    // console.log('Joystick force:', data.force);
}

function onJoystickEnd(event) {
	targetLinearVel = 0;
	targetAngularVel = 0;

	currentLinearVel = 0;
    currentAngularVel = 0;

	console.log('targetLinearVel:', targetLinearVel, 'targetAngularVel: ', targetAngularVel);

    // Print velocities when joystick ends
    // console.log('Joystick ended');
    // console.log('Current linear velocity:', currentLinearVel);
    // console.log('Current angular velocity:', currentAngularVel);
	// sendVelocities(currentLinearVel, currentAngularVel);
	// publishVelocity(currentLinearVel, 0.0, currentAngularVel);

}

addJoystickListeners();