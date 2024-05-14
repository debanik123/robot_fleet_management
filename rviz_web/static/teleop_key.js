
const teleopButton = document.getElementById('teleopButton');
let targetLinearVel = 0;
let targetAngularVel = 0;

teleopButton.addEventListener('click', function() {
    // Start capturing keyboard data
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
});

// Keyboard event handlers
function onKeyDown(event) {
    // Capture keyboard data and perform teleoperation
    switch(event.key) {
        case 'ArrowUp':
            targetLinearVel = 0.5;  // Example linear velocity
            console.log('Pressed key: ArrowUp');
            break;
        case 'ArrowDown':
            targetLinearVel = -0.5;  // Example linear velocity
            console.log('Pressed key: ArrowDown');
            break;
        case 'ArrowLeft':
            targetAngularVel = 0.5;  // Example angular velocity
            console.log('Pressed key: ArrowLeft');
            break;
        case 'ArrowRight':
            targetAngularVel = -0.5;  // Example angular velocity
            console.log('Pressed key: ArrowRight');
            break;
        default:
            console.log('Pressed key:', event.key);
            break;
    }
}


function onKeyUp(event) {
    // Stop motion when key is released
    targetLinearVel = 0;
    targetAngularVel = 0;

    // Publish zero velocities
    // sendVelocities(targetLinearVel, targetAngularVel);
}