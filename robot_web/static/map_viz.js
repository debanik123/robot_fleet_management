import { mapview, pathSubscriber, 
    robot_poseSubscriber, scan_pose_Subscriber, 
    scanSubscriber, goalPosePublisher, drawFilledCircle,
    mapToImageCoordinates, imageToMapCoordinates, getColorForOccupancy} from './robo_utilities.js';

// import { scan_viz } from './scan.js';

const canvasWidth = 480;
const canvasHeight = 480;
let path_g = null;
let map_msg_g = null;

let map_msg_, cellWidth_, cellHeight_;

// const mapContainer = document.getElementById('map-container');

// Create a canvas element and set its attributes
const canvas = document.getElementById('map_canvas');
canvas.width = canvasWidth;
canvas.height = canvasHeight;
const ctx = canvas.getContext('2d');

// const canvas_path = document.getElementById('path_canvas');
// canvas_path.width = canvasWidth;
// canvas_path.height = canvasHeight;
// const ctx_path = canvas_path.getContext('2d');

// const offsetX = canvas.offsetLeft - canvas_path.offsetLeft;
// const offsetY = canvas.offsetTop - canvas_path.offsetTop;

// console.log(offsetX, offsetY);


// // Subscribe to the map topic and load the map onto the canvas
mapview.subscribe(map_msg => {
    console.log(`Received map data for: ${mapview.name}`);
    // loadMap(map_msg);
    map_msg_g = map_msg;
});

pathSubscriber.subscribe(function(pathMsg) { 
    // ctx.clearRect(0, 0, canvas.width, canvas.height);
    path_g = pathMsg.poses;
  });

// // Function to load the map onto the canvas
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


// function visualizePath(poses, mapData, scaleX, scaleY) {
//     ctx_path.strokeStyle = 'green';
//     // const darkGreen = '#006400'; // You can adjust the hex code as needed
//     // ctx_path.strokeStyle = darkGreen;
//     ctx_path.lineWidth = 2;
//     ctx_path.clearRect(0, 0, canvasWidth, canvasHeight);

//     for (let i = 0; i < poses.length - 1; i++) {
//         // 
//         const pose1 = poses[i].pose.position;
//         const pose2 = poses[i + 1].pose.position;

        
//         const imageCoords1 = mapToImageCoordinates(pose1.x, pose1.y, mapData, scaleX, scaleY);
//         const imageCoords2 = mapToImageCoordinates(pose2.x, pose2.y, mapData, scaleX, scaleY);
        
//         ctx_path.beginPath();
//         ctx_path.moveTo(imageCoords1.x, imageCoords1.y);
//         ctx_path.lineTo(imageCoords2.x, imageCoords2.y);
//         ctx_path.stroke();
//     }
    
// }
