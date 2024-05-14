import { mapview, pathSubscriber, 
    robot_poseSubscriber, scan_pose_Subscriber, 
    scanSubscriber, goalPosePublisher, drawFilledCircle,
    mapToImageCoordinates, imageToMapCoordinates, getColorForOccupancy} from './robo_utilities.js';

import { scan_viz } from './scan.js';

const canvasWidth = 480;
const canvasHeight = 480;
let path_g = [];
let map_msg_, cellWidth_, cellHeight_;

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

pathSubscriber.subscribe(function(pathMsg) { 
    // ctx.clearRect(0, 0, canvas.width, canvas.height);
    path_g = pathMsg.poses
    visualizePath(path_g, map_msg_, cellWidth_, cellHeight_);
    
    // visualizeMap(mapData);
      // console.log(pathMsg.poses);
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
    map_msg_ = map_msg;
    cellWidth_ = cellWidth;
    cellHeight_ = cellHeight;
    // visualizePath(path_g, map_msg, cellWidth, cellHeight);
}


function visualizePath(poses, mapData, scaleX, scaleY) {
    ctx.strokeStyle = 'green';
    // const darkGreen = '#006400'; // You can adjust the hex code as needed
    // ctx.strokeStyle = darkGreen;
    ctx.lineWidth = 2;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    for (let i = 0; i < poses.length - 1; i++) {
        // 
        const pose1 = poses[i].pose.position;
        const pose2 = poses[i + 1].pose.position;

        const imageCoords1 = mapToImageCoordinates(pose1.x, pose1.y, mapData, scaleX, scaleY);
        const imageCoords2 = mapToImageCoordinates(pose2.x, pose2.y, mapData, scaleX, scaleY);
        
        ctx.beginPath();
        ctx.moveTo(imageCoords1.x, imageCoords1.y);
        ctx.lineTo(imageCoords2.x, imageCoords2.y);
        ctx.stroke();
    }
    
}
