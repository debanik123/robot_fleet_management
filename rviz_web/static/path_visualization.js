// let tfModule = import(`static/tf.js`);
// import tf from 'static.tf.js';
var maps = {}; // Dictionary to store maps and their canvas elements
var canvas, ctx, scaleX, scaleY, startX, startY, mouseUpPose, mouseDownPose;
var mapName;
var p1_x = null;
var p1_y = null;
var path_g = null;
var robot_pose = null;
var isDragging = false;
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

sprite.src = "static/icons/simplegoal.png";
// const Quaternion = require('quaternion');
// let tf = tfModule.tf;

// ros2 run rosbridge_server rosbridge_websocket
// ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
// ROS connection setup (assuming ROSLIB is already included)
var ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Replace with your ROS bridge server address
});

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

// ROS map subscription (adjust topic name and message type if needed)
var mapview = new ROSLIB.Topic({
  ros: ros,
  name: '/map', // Subscribe to all map topics (replace with specific topic if needed)
  messageType: 'nav_msgs/OccupancyGrid'  // Adjust based on your map message type
});

var pathSubscriber = new ROSLIB.Topic({
    ros : ros,
    name : '/plan',
    messageType : 'nav_msgs/Path'
});

// Create the tf2Subscriber
// var tf2Subscriber = new ROSLIB.Topic({
//   ros: ros,
//   name: '/tf',
//   messageType: 'tf2_msgs/msg/TFMessage'
// });

var robot_poseSubscriber = new ROSLIB.Topic({
  ros: ros,
  name: '/robot_pose',
  messageType: 'geometry_msgs/PoseStamped'
});

var scan_pose_Subscriber = new ROSLIB.Topic({
  ros: ros,
  name: '/scan_pose',
  messageType: 'geometry_msgs/PoseStamped'
});

// Create a ROSLIB.Topic object for publishing
var goalPosePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/PoseStamped'
});

var scanSubscriber = new ROSLIB.Topic({
  ros: ros,
  name: '/scan',
  messageType: 'sensor_msgs/msg/LaserScan'
});

// const tfClient = new ROSLIB.TFClient({
//   ros: ros,
//   fixedFrame: 'map'  // Adjust the fixed frame as needed
// });

// // // Lookup a transform
// tfClient.subscribe('/tf', (tfMessage) => {
//   const transform = tfMessage.lookupTransform('map', 'base_link');
//   console.log('Transform:', transform);
// });

scan_pose_Subscriber.subscribe(function(msg) {
  scan_pose = msg.pose;
  if (mapData !== null) 
  {
    visualizeMap(mapData);
  }

});

scanSubscriber.subscribe(function(msg) {
  scan_msg = msg;
  if (mapData !== null) 
  {
    visualizeMap(mapData);
  }

});


robot_poseSubscriber.subscribe(function(message) {
  // ctx.clearRect(0, 0, canvas.width, canvas.height);
  robot_pose = message.pose;
  // console.log('Received pose:', robot_pose);
  if (mapData !== null) 
  {
    visualizeMap(mapData);
  }

});


// tf2Subscriber.subscribe(function(msg) {
//   for (const transform of msg.transforms) {
//     // console.log(transform);
//     if (transform.child_frame_id === 'base_footprint') {
//       // Found the transform between 'scan' and 'base_link'
//       const translation = transform.transform.translation;
//       const rotation = transform.transform.rotation;
//       console.log('Transformation from scan to base_link:');
//       console.log('Translation:', translation);
//       console.log('Rotation:', rotation);
//       return; // Exit the loop if found
//   }
//   }
// });

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

function scan_viz(msg) {
  msg.ranges.forEach(function (item, index) {
    if (item >= msg.range_min && item <= msg.range_max) {
      const angle = msg.angle_min + index * msg.angle_increment;
      var scan_x = item * Math.cos(angle);
      var scan_y = item * Math.sin(angle);
      var scan_vec = {x: scan_x, 
                      y: scan_y, 
                      z: 0}
      if(scan_pose !== null)
      {
        // Apply rotation
        var qn = new Quaternion(scan_pose.orientation);
        var rotated_scan_vec = applyRotation(scan_vec, qn, false);
        
        // let outputVector =  Object.assign({}, inputVector);
        // Apply translation
        var translated_scan_vec = {
          x: rotated_scan_vec.x + scan_pose.position.x,
          y: rotated_scan_vec.y + scan_pose.position.y,
          z: rotated_scan_vec.z + scan_pose.position.z
        };
        
        const image_robot_scan = mapToImageCoordinates(translated_scan_vec.x, translated_scan_vec.y);
        drawFilledCircle(image_robot_scan.x, image_robot_scan.y, 1 , "red");
      }
    }
  });
}

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
    // drawFilledCircle(ctx, p1_x, p1_y, 5, 'red');
    visualizePath(path_g);
  }
  if (robot_pose !== null) 
  {
    // var px = robot_pose.position.x;
    // var py = robot_pose.position.y;
    const image_robot_pose = mapToImageCoordinates(robot_pose.position.x, robot_pose.position.y);
    // console.log('image_robot_pose:', image_robot_pose);
    drawFilledCircle(image_robot_pose.x, image_robot_pose.y, 10, "red");
  }

  drawArrow();
  if (init_start_point !== null && init_delta !== null)
  {
    static_drawArrow(init_start_point, init_delta);
    // drawFilledCircle(mouse_x, mouse_y, 5, 'blue');
  }

  if (typeof scan_msg !== 'undefined') {
    scan_viz(scan_msg);
  }

  
}

function getColorForOccupancy(occupancyValue) {
  if (occupancyValue === 100) {
      return 'black'; // Occupied space
  } else if (occupancyValue === 0) {
      return 'white'; // Free space
  } else {
      // Calculate grayscale color based on occupancy value
      var colorValue = 255 - (occupancyValue * 255) / 100;
      return 'rgb(' + colorValue + ',' + colorValue + ',' + colorValue + ')';
  }
}

pathSubscriber.subscribe(function(pathMsg) { 
  // ctx.clearRect(0, 0, canvas.width, canvas.height);
  path_g = pathMsg.poses
  
  visualizeMap(mapData);
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
        
        // console.log(`Path Segment ${i+1}:`);
        // console.log(`Pose 1: (${pose1.x}, ${pose1.y})`);
        // console.log(`Pose 2: (${pose2.x}, ${pose2.y})`);

        const imageCoords1 = mapToImageCoordinates(pose1.x, pose1.y);
        const imageCoords2 = mapToImageCoordinates(pose2.x, pose2.y);

        // p1_x = parseInt(imageCoords1.x); 
        // p1_y = parseInt(imageCoords1.y);
        // console.log(`Pose 1: (${p1_x}, ${p1_y})`);

        ctx.beginPath();
        ctx.moveTo(imageCoords1.x, imageCoords1.y);
        ctx.lineTo(imageCoords2.x, imageCoords2.y);
        ctx.stroke();
        // visualizeMap(mapData);
        // drawFilledCircle(ctx, imageCoords1.x, imageCoords1.y, 1, 'red');
    }
    
}

function mapToImageCoordinates(robot_x, robot_y) {
    // Extract map information
    const map_resolution = mapData.info.resolution;
    const map_origin_x = mapData.info.origin.position.x;
    const map_origin_y = mapData.info.origin.position.y;
    const image_width = mapData.info.width;
    const image_height = mapData.info.height;

    // Convert robot's map coordinates to image coordinates
    const pixel_x = Math.floor((robot_x - map_origin_x) / map_resolution);
    const pixel_y = Math.floor(image_height - (robot_y - map_origin_y) / map_resolution);  // Invert y-axis

    // return { x: pixel_x, y: pixel_y };
    return { x: pixel_x * scaleX, y: pixel_y * scaleY };
}

var mapContainer = document.getElementById('map-container');
mapContainer.addEventListener('mousedown', function(event) {
  var rect = mapContainer.getBoundingClientRect();
  const { clientX, clientY } = event.touches ? event.touches[0] : event;
	start_point = {
		x: clientX- rect.left,
		y: clientY - rect.top
	};

  isDragging = true;
  console.log('mousedown');
  init_start_point = null;
  init_delta = null;

});

mapContainer.addEventListener('mousemove', function(event) {
  if (start_point === undefined) return;

	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	delta = {
		x: start_point.x - clientX,
		y: start_point.y - clientY,
	};
  // drawArrow();

});

mapContainer.addEventListener('mouseup', function(event) {
  console.log('mouseup');
  send_nav2_goal_Message(start_point, delta);
  drawArrow();
  init_start_point = start_point;
  init_delta = delta;
  // static_drawArrow(start_point, delta);
  // var orientation = calculateOrientationQuaternion(start_point.x, start_point.y, delta.x, delta.y);
  // handleMapClick(start_point.x, start_point.y, orientation);
  start_point = undefined;
	delta = undefined;

});

function static_drawArrow(point, delta)
{
  let ratio = sprite.naturalHeight/sprite.naturalWidth;
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.scale(1.0, 1.0);
  ctx.rotate(Math.atan2(-delta.y, -delta.x));
  ctx.drawImage(sprite, -80, -80*ratio, 160, 160*ratio);
  ctx.restore();
}

function send_nav2_goal_Message(pos, delta){
	if(!pos || !delta){
		status.setError("Could not send message, pose invalid.");
		return;
	}
  
	let yaw = Math.atan2(delta.y, -delta.x);
	let quat = new Quaternion.fromEuler(yaw, 0, 0, 'ZXY');

  var map_pos = imageToMapCoordinates(pos.x / scaleX, pos.y / scaleY);
	// let map_pos = view.screenToFixed(pos);

	const currentTime = new Date();
	const currentTimeSecs = Math.floor(currentTime.getTime() / 1000);
	const currentTimeNsecs = (currentTime.getTime() % 1000) * 1e6;

	const publisher = new ROSLIB.Topic({
		ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/PoseStamped'
	});

	const poseMessage = new ROSLIB.Message({
		header: {
			stamp: {
				secs: currentTimeSecs,
      			nsecs: currentTimeNsecs
			},
			frame_id: 'map'
		},
		pose: {
			position: {
				x: map_pos.x,
				y: map_pos.y,
				z: 0.0
			},
			orientation: {
				x: quat.x,
				y: quat.y,
				z: quat.z,
				w: quat.w
			}
		}
	});	
	publisher.publish(poseMessage);
	// status.setOK();
}

function imageToMapCoordinates(pixel_x, pixel_y) {
  // Extract map information
  const map_resolution = mapData.info.resolution;
  const map_origin_x = mapData.info.origin.position.x;
  const map_origin_y = mapData.info.origin.position.y;
  const image_width = mapData.info.width;
  const image_height = mapData.info.height;

  // Invert y-axis
  pixel_y = image_height - pixel_y;

  // Convert image coordinates to robot's map coordinates with scaling factors
  const robot_x = pixel_x * map_resolution + map_origin_x;
  const robot_y = pixel_y * map_resolution + map_origin_y;

  return { x: robot_x, y: robot_y };
}

function handleMapClick(mouseX, mouseY, orientation) {
  var mapCoordinates = imageToMapCoordinates(mouseX / scaleX, mouseY / scaleY);
  console.log('Clicked at map path coordinates (x:', mouseX / scaleX, ', y:', mouseY / scaleY, ')');
  // var theta = Math.PI / 4; // Angle in radians (45 degrees)
  var goalPose = createGoalPoseWithOrientation(mapCoordinates.x, mapCoordinates.y, orientation);
  // console.log(goalPose);

  goalPosePublisher.publish(goalPose);
}



function drawArrow() {
  // const wid = canvas.width;
  // const hei = canvas.height;

  // ctx.clearRect(0, 0, wid, hei);

  if(delta){
    let ratio = sprite.naturalHeight/sprite.naturalWidth;

    ctx.save();
    ctx.translate(start_point.x, start_point.y);
    ctx.scale(1.0, 1.0);
    ctx.rotate(Math.atan2(-delta.y, -delta.x));
    ctx.drawImage(sprite, -80, -80*ratio, 160, 160*ratio);
    ctx.restore();
  }
}


function drawFilledCircle(centerX, centerY, radius, color) {
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function createQuaternion(theta) {
  var qx = 0.0;
  var qy = 0.0;
  var qz = Math.sin(theta / 2);
  var qw = Math.cos(theta / 2);
  
  return { x: qx, y: qy, z: qz, w: qw };
}

// Function to create and return a goal pose message with position (x, y) and orientation theta
function createGoalPoseWithOrientation(x, y, orientation) {
  // Create the quaternion orientation
  // var orientation = createQuaternion(theta);

  // Define the pose data
  var poseMsg = new ROSLIB.Message({
      header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: 'map'
      },
      pose: {
          position: { x: x, y: y, z: 0.0 },
          orientation: orientation
      }
  });

  return poseMsg;
}

function applyRotation(vector, r, inverse){
	if(inverse)
		r = r.inverse();
		
	const v = r.rotateVector([
		vector.x,
		vector.y,
		vector.z
	]);

	return {
		x: v[0],
		y: v[1],
		z: v[2]
	}
}

function calculateOrientationQuaternion(upPoseX, upPoseY, downPoseX, downPoseY) 
{
  // Convert mouse coordinates to map coordinates
  var upPose = imageToMapCoordinates(upPoseX / scaleX, upPoseY / scaleY);
  var downPose = imageToMapCoordinates(downPoseX / scaleX, downPoseY / scaleY);

  // console.log(`Map upPose coordinates: (${upPose.x}, ${upPose.y})`);
  // console.log(`Map downPose coordinates: (${downPose.x}, ${downPose.y})`);

  // Calculate the difference between map coordinates of upPose and downPose
  var xDelta = upPose.x - downPose.x;
  var yDelta = upPose.y - downPose.y;

  // Calculate theta (rotation angle) based on the difference
  var thetaRadians = -Math.atan2(xDelta, yDelta);

  // Adjust thetaRadians to get the correct orientation
  if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
      thetaRadians += (3 * Math.PI / 2);
  } else {
      thetaRadians -= (Math.PI / 2);
  }

  // Calculate theta in degrees
  var thetaDegrees = thetaRadians * (180.0 / Math.PI);

  // Calculate quaternion components
  var qz = Math.sin(-thetaRadians / 2.0);
  var qw = Math.cos(-thetaRadians / 2.0);

  // Create the orientation quaternion
  var orientation = new ROSLIB.Quaternion({
      x: 0,
      y: 0,
      z: qz,
      w: qw
  });

  // Log the calculated values
  // console.log(`Map coordinates: (${mapCoordinates.x}, ${mapCoordinates.y})`);
  console.log(`xDelta: ${xDelta}, yDelta: ${yDelta}, thetaDegrees: ${thetaDegrees}`);

  return orientation;
}


// ROS connection events
ros.on('connected', function() {
  console.log('Connected to ROS server');
});

ros.on('error', function(error) {
  console.error('Error connecting to ROS server:', error);
});
