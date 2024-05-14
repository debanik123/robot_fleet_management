import {applyRotation, drawFilledCircle,
    mapToImageCoordinates} from './robo_utilities.js';

export function scan_viz(msg, scan_pose, mapData, scaleX, scaleY, ctx) {
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
            
            const image_robot_scan = mapToImageCoordinates(translated_scan_vec.x, translated_scan_vec.y, mapData, scaleX, scaleY);
            drawFilledCircle(ctx, image_robot_scan.x, image_robot_scan.y, 1 , "red");
        }
        }
    });
    }