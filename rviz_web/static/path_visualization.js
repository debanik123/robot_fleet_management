
import { mapview, pathSubscriber, 
  robot_poseSubscriber, scan_pose_Subscriber, 
  scanSubscriber, goalPosePublisher, drawFilledCircle,
  mapToImageCoordinates, imageToMapCoordinates} from './robo_utilities.js';

import {scan_viz} from './scan.js';
// 


var maps = {}; // Dictionary to store maps and their canvas elements
var canvas, ctx, scaleX, scaleY, startX, startY, mouseUpPose, mouseDownPose;
var mapName;
var p1_x = null;
var p1_y = null;
var path_g = null;
var robot_pose = null;

var mapData = null;
var mouse_x = null;
var mouse_y = null;
var init_start_point = null;
var init_delta = null;
var scan_pose = null;

let active = false;
let sprite = new Image();
let start_point = undefined;
let delta = undefined;
let scan_msg = undefined;

// sprite.src = "static/icons/simplegoal.png";


// const Quaternion = require('quaternion');
// let tf = tfModule.tf;

// Map visualization functions
function createCanvas(mapName) {
  
  const mapContainer = document.getElementById('map-container');
  canvas = document.createElement('canvas');
  canvas.id = `map-canvas-${mapName}`;
  canvas.width = 0; // Will be set later
  canvas.height = 0; // Will be set later
  mapContainer.appendChild(canvas);
  maps[mapName] = { canvas: canvas };
  return canvas;
}

function clearCanvas(mapName) {
  if (maps[mapName]) {
    canvas = maps[mapName].canvas;
    ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }
}



scan_pose_Subscriber.subscribe(function(msg) {
  scan_pose = msg.pose;
  if (mapData !== null) 
  {
    visualizeMap(mapData);
  }

});

scanSubscriber.subscribe(function(msg) {
  scan_msg = msg;
});


robot_poseSubscriber.subscribe(function(message) {
  // ctx.clearRect(0, 0, canvas.width, canvas.height);
  robot_pose = message.pose;

});

mapview.subscribe(function(map_msg) {
  mapName = mapview.name; // Assuming topic name represents map name
  console.log(`Received map data for: ${mapName}`);
  mapData = map_msg;

  if (!maps[mapName]) {
    canvas = createCanvas(mapName);
    maps[mapName].ctx = canvas.getContext('2d');
    // console.log(`Inside: ${mapName}`);

    canvas = maps[mapName].canvas;
    ctx = maps[mapName].ctx;

    // Update canvas dimensions based on map data
    canvas.width = 480;
    canvas.height = 480;

    // Clear previous map visualization (optional)
    // clearCanvas(mapName);

    scaleX = canvas.width / map_msg.info.width;
    scaleY = canvas.height / map_msg.info.height;
    
    visualizeMap(map_msg);
  }

  
});



function visualizeMap(map_msg) {
  for (var y = 0; y < map_msg.info.height; y++) {
      for (var x = 0; x < map_msg.info.width; x++) {
          var index = x + y * map_msg.info.width;
          var value = map_msg.data[index];
          var color = getColorForOccupancy(value);
          ctx.fillStyle = color;
          // ctx.fillRect(x, y, 1, 1);
          ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);
      }
  }

  if (path_g !== null) 
  {
    visualizePath(path_g);
  }
  if (robot_pose !== null) 
  {
    // var px = robot_pose.position.x;
    // var py = robot_pose.position.y;
    const image_robot_pose = mapToImageCoordinates(robot_pose.position.x, robot_pose.position.y, mapData, scaleX, scaleY);
    // console.log('image_robot_pose:', image_robot_pose);
    drawFilledCircle(ctx, image_robot_pose.x, image_robot_pose.y, 10, "red");
  }

  // drawArrow();
  if (init_start_point !== null && init_delta !== null)
  {
    static_drawArrow(ctx, init_start_point, init_delta);
  }

  if (typeof scan_msg !== 'undefined') {
    scan_viz(scan_msg, scan_pose, mapData, scaleX, scaleY, ctx);
  }

  
}



pathSubscriber.subscribe(function(pathMsg) { 
  // ctx.clearRect(0, 0, canvas.width, canvas.height);
  path_g = pathMsg.poses
  
  // visualizeMap(mapData);
    // console.log(pathMsg.poses);
});

function visualizePath(poses) {
    ctx.strokeStyle = 'green';
    // const darkGreen = '#006400'; // You can adjust the hex code as needed
    // ctx.strokeStyle = darkGreen;
    ctx.lineWidth = 2;

    for (let i = 0; i < poses.length - 1; i++) {
        // ctx.clearRect(0, 0, canvas.width, canvas.height);
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



// import './goal_publish.js';