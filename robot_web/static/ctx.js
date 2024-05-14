const mapContainer = document.getElementById('map-container');

const canvasWidth = 480;
const canvasHeight = 480;
// Create a canvas element and set its attributes
const canvas = document.createElement('canvas');
canvas.id = 'map-canvas-map';
canvas.width = canvasWidth;
canvas.height = canvasHeight;
mapContainer.appendChild(canvas);

const ctx = canvas.getContext('2d');

// Function to export the canvas context
function exportContext() {
    return ctx;
}

// Export the function
export { exportContext };