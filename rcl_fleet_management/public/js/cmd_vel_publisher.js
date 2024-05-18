const rclnodejs = require('rclnodejs');

async function cmdVelPublisher() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('cmd_vel_publisher');

  // Create the publisher for cmd_vel
  const publisher = node.createPublisher('geometry_msgs/msg/Twist', 'cmd_vel');
  
  // Function to publish velocity messages
  function publishVelocities(linear, angular) {
    const twistMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist');
    twistMsg.linear.x = linear;
    twistMsg.linear.y = 0.0;
    twistMsg.linear.z = 0.0;
    twistMsg.angular.x = 0.0;
    twistMsg.angular.y = 0.0;
    twistMsg.angular.z = angular;
    publisher.publish(twistMsg);
  }

  // Export the function to be used in other modules
  // module.exports = {
  //   publishVelocities
  // };

  node.spin();
}

cmdVelPublisher().catch(() => {
  process.exitCode = 1;
});

export function sendVelocities(linear, angular) {
  publishVelocities(linear, angular);
}