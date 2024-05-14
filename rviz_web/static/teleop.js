// Connect to ROS2
import '../static/lib/nipplejs.js';
export const nipplejs = window.nipplejs;
let joy_offset_x = "50%";
let joy_offset_y = "85%";
let targetLinearVel = 0;
let targetAngularVel = 0;

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
    console.log('Joystick ', data);
    const maxLinearVel = 0.7;
	const maxAngularVel = 1.3;
	const force = Math.min(Math.max(data.force, 0.0), 1.0);

	targetLinearVel = maxLinearVel * Math.sin(data.angle.radian) * force;
	targetAngularVel = - maxAngularVel * Math.cos(data.angle.radian) * force;

    console.log('targetLinearVel:', targetLinearVel, 'targetAngularVel: ', targetAngularVel);


    // console.log('Joystick force:', data.force);
}

function onJoystickEnd(event) {
	targetLinearVel = 0;
	targetAngularVel = 0;

}

addJoystickListeners();


// const joystick = document.getElementById('joystick');
// const handle = document.getElementById('handle');

// let isDragging = false;

// // Event listeners for mouse/touch interactions
// handle.addEventListener('mousedown', startDrag);
// handle.addEventListener('touchstart', startDrag);

// document.addEventListener('mousemove', drag);
// document.addEventListener('touchmove', drag);

// document.addEventListener('mouseup', endDrag);
// document.addEventListener('touchend', endDrag);

// function startDrag(event) {
//     var rect = joystick.getBoundingClientRect();
//     const { clientX, clientY } = event.touches ? event.touches[0] : event;
//     joy_start_point = {
// 		x: clientX- rect.left,
// 		y: clientY - rect.top
// 	};
//     event.preventDefault();
// }

// function drag(event) {
//     if (joy_start_point === undefined) return;
//     var bounds = joystick.getBoundingClientRect();
// 	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	

//     let x = event.clientX - bounds.left;
//     let y = event.clientY - bounds.top;

//     joy_delta = {
// 		x: joy_start_point.x - clientX,
// 		y: joy_start_point.y - clientY,
// 	}
//     // Ensure handle stays inside joystick boundary
//     x = Math.max(0, Math.min(x, bounds.width));
//     y = Math.max(0, Math.min(y, bounds.height));

//     handle.style.left = `${x}px`;
//     handle.style.top = `${y}px`;

//     let resolution = 0.01;
//     let linearVel = Math.sqrt(x * x + y * y)*resolution;

//     let minLinearVel = 0.0; // Minimum linear velocity in meters per second
//     let maxLinearVel = 1.0; // Maximum linear velocity in meters per second
    
//     linearVel = Math.min(Math.max(linearVel, minLinearVel), maxLinearVel);

//     // let yaw = Math.atan2(y, -x);
//     let yaw = Math.atan(joy_delta.y / -joy_delta.x);
//     let angularVel = yaw;
//     // let max_angular = 1.5;
//     // let angularVel = max_angular * Math.sin(yaw);

//     // console.log(joy_delta, yaw);
//     // Send velocities to robot control
//     sendVelocities(linearVel, angularVel);
// }

// function endDrag() {
//     isDragging = false;
//     // Reset handle to center position
//     joy_start_point = undefined;
//     joy_delta = undefined;

//     handle.style.left = '50%';
//     handle.style.top = '50%';

//     // Stop robot movement
//     sendVelocities(0, 0);
// }

