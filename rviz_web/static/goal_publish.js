// Select the map container
let mapContainer = document.getElementById('map-container');

// Create a canvas element and set its attributes
let canvas = document.createElement('canvas');
canvas.id = 'map-canvas-/map1';
canvas.width = 480;
canvas.height = 480;

// Get the 2D rendering context of the canvas
let ctx = canvas.getContext('2d');
// Append the canvas to the map container
mapContainer.appendChild(canvas);

// Define variables for arrow drawing
let sprite = new Image();
sprite.src = "static/icons/simplegoal.png";
let start_point = undefined;
let delta = undefined;
let isDragging = false;

// Listen for mouse events on the map container
mapContainer.addEventListener('mousedown', onMousedown);
mapContainer.addEventListener('mousemove', onMousemove);
mapContainer.addEventListener('mouseup', onMouseup);

// Event handler for mouse down event
function onMousedown(event) {
    var rect = mapContainer.getBoundingClientRect();
    const { clientX, clientY } = event.touches ? event.touches[0] : event;
    start_point = {
        x: clientX - rect.left,
        y: clientY - rect.top
    };
  
    isDragging = true;
}

// Event handler for mouse move event
function onMousemove(event) {
    if (!isDragging) return;

    var rect = mapContainer.getBoundingClientRect();
    const { clientX, clientY } = event.touches ? event.touches[0] : event;
    delta = {
        x: start_point.x - clientX + rect.left,
        y: start_point.y - clientY + rect.top,
    };
  
    drawArrow();
}

// Event handler for mouse up event
function onMouseup(event) {
    isDragging = false;
}

// Function to draw the arrow on the canvas
function drawArrow() {
    if (!start_point || !delta) return;
  
    let ratio = sprite.naturalHeight / sprite.naturalWidth;
  
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.translate(start_point.x, start_point.y);
    ctx.rotate(Math.atan2(delta.y, delta.x));
    ctx.drawImage(sprite, -70, -70 * ratio, 140, 140 * ratio);
    ctx.restore();
}
