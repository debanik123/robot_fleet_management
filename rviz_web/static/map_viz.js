import { mapview, mapToImageCoordinates, getColorForOccupancy } from './robo_utilities.js';
import { scan_viz } from './scan.js';

const canvasWidth = 480;
const canvasHeight = 480;

const mapContainer = document.getElementById('map-container');

// Create a canvas element and set its attributes
const canvas = document.createElement('canvas');
canvas.id = 'map-canvas-/map1';
canvas.width = canvasWidth;
canvas.height = canvasHeight;
mapContainer.appendChild(canvas);

// Get the 2D rendering context of the canvas
const ctx = canvas.getContext('2d');

// Subscribe to the map topic and load the map onto the canvas
mapview.subscribe(map_msg => {
    console.log(`Received map data for: ${mapview.name}`);
    loadMap(map_msg);
});

// Function to load the map onto the canvas
function loadMap(map_msg) {
    if (!map_msg || !map_msg.data || !map_msg.info) {
        console.error("Map data is invalid.");
        return;
    }

    const mapData = map_msg.data;
    const mapInfo = map_msg.info;

    // Clear the canvas
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);

    // Define cell size based on canvas size and map dimensions
    const cellWidth = canvasWidth / mapInfo.width;
    const cellHeight = canvasHeight / mapInfo.height;

    // Loop through map data and draw onto canvas
    for (let y = 0; y < mapInfo.height; y++) {
        for (let x = 0; x < mapInfo.width; x++) {
            const val = mapData[x + y * mapInfo.width];

            // Set fill color based on occupancy value
            ctx.fillStyle = getColorForOccupancy(val);

            // Draw rectangle representing map cell
            ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
        }
    }
}
