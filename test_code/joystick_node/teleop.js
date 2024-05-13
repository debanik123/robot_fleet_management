// teleop.js

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
    isDragging = true;
    event.preventDefault();
}

function drag(event) {
    if (!isDragging) return;
    const bounds = joystick.getBoundingClientRect();
    let x = event.clientX - bounds.left;
    let y = event.clientY - bounds.top;

    // Ensure handle stays inside joystick boundary
    x = Math.max(0, Math.min(x, bounds.width));
    y = Math.max(0, Math.min(y, bounds.height));

    // Update handle position
    handle.style.left = `${x}px`;
    handle.style.top = `${y}px`;

    // Calculate velocities (normalized)
    const linearVel = (x - bounds.width / 2) / (bounds.width / 2);
    const angularVel = (bounds.height / 2 - y) / (bounds.height / 2);

    // Send velocities to robot control
    sendVelocities(linearVel, angularVel);
}

function endDrag() {
    isDragging = false;
    // Reset handle to center position
    handle.style.left = '50%';
    handle.style.top = '50%';

    // Stop robot movement
    sendVelocities(0, 0);
}

function sendVelocities(linearVel, angularVel) {
    // Replace with your code to send velocities to the robot
    console.log('Linear Velocity:', linearVel);
    console.log('Angular Velocity:', angularVel);
}
