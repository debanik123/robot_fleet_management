
// import './robo_utilities.js';
// import { sendVelocities } from './robo_utilities.js';
const teleopButton = document.getElementById('teleopButton');
let targetLinearVel = 0;
let targetAngularVel = 0;

let maxLinearVel = 0.5; // Initial maximum linear velocity
let maxAngularVel = 1.0; // Initial maximum angular velocity


teleopButton.addEventListener('click', function() {
    // Start capturing keyboard data
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
});

// Keyboard event handlers
function onKeyDown(event) {
    console.log('targetLinearVel: ', targetLinearVel, 'targetAngularVel: ', targetAngularVel);
    // sendVelocities(targetLinearVel, targetAngularVel);
    // Capture keyboard data and perform teleoperation
    switch(event.key) {
        case 'ArrowUp':
        case 'i':
        case 'w':
            targetLinearVel = maxLinearVel;  // Example linear velocity
            targetAngularVel = 0.0;
            // console.log('Pressed key: ArrowUp');
            break;

        case 'ArrowDown':
        case ',':
        case 's':
            targetLinearVel = -maxLinearVel;  // Example linear velocity
            targetAngularVel = 0.0;
            // console.log('Pressed key: ArrowDown');
            break;
        
        case 'ArrowLeft':
        case 'j':
        case 'a':
            targetLinearVel = 0.0;  // Example linear velocity
            targetAngularVel = maxAngularVel;
            // console.log('Pressed key: ArrowLeft');
            break;
        case 'ArrowRight':
        case 'l':
        case 'd':
            targetLinearVel = 0.0;  // Example linear velocity
            targetAngularVel = -maxAngularVel;
            // console.log('Pressed key: ArrowRight');
            break;

        case 'o':
            targetLinearVel = maxLinearVel/2.0;  // Example linear velocity
            targetAngularVel = -maxAngularVel/1.5;
            // console.log('Pressed key: ArrowRight');
            break;
        
        case 'u':
            targetLinearVel = maxLinearVel/2.0;  // Example linear velocity
            targetAngularVel = maxAngularVel/1.5;
            // console.log('Pressed key: ArrowRight');
            break;

        case 'm':
            targetLinearVel = maxLinearVel/2.0;  // Example linear velocity
            targetAngularVel = -maxAngularVel/1.5;
            // console.log('Pressed key: ArrowRight');
            break;

        case '.':
            targetLinearVel = maxLinearVel/2.0;  // Example linear velocity
            targetAngularVel = maxAngularVel/1.5;
            // console.log('Pressed key: ArrowRight');
            break;

        case 'q':
            new_acc_Speeds(10);
            break;

        case 'z':
            new_dacc_Speeds(10);
            break;

        default:
            // console.log('Pressed key:', event.key);
            break;

        
        
    }
}

function new_acc_Speeds(percentage) {
    maxLinearVel += maxLinearVel * (percentage / 100);
    maxAngularVel += maxAngularVel * (percentage / 100);
}

function new_dacc_Speeds(percentage) {
    maxLinearVel -= maxLinearVel * (percentage / 100);
    maxAngularVel -= maxAngularVel * (percentage / 100);
}

function onKeyUp(event) {
    // Stop motion when key is released
    targetLinearVel = 0;
    targetAngularVel = 0;

    // sendVelocities(targetLinearVel, targetAngularVel);

    // Publish zero velocities
    // sendVelocities(targetLinearVel, targetAngularVel);
}