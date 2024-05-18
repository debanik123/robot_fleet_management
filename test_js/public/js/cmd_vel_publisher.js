const rclnodejs = require('rclnodejs');

let publisher;

function initializePublisher() {
  rclnodejs.init().then(() => {
    const node = new rclnodejs.Node('robot_velocity_publisher');
    publisher = node.createPublisher('geometry_msgs/msg/Twist', '/cmd_vel');
    rclnodejs.spin(node);
    console.log('Publisher initialized');
  }).catch((err) => {
    console.error('Error initializing rclnodejs:', err);
  });
}

function publishVelocity(linearX, linearY, angularZ) {
  if (!publisher) {
    console.error('Publisher is not initialized');
    return;
  }

  const msg = rclnodejs.createMessageObject('geometry_msgs/msg/Twist');
  msg.linear.x = linearX;
  msg.linear.y = linearY;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = angularZ;

  publisher.publish(msg);
}

module.exports = { initializePublisher, publishVelocity };
