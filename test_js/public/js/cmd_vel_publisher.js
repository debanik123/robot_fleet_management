const { createNode, ROS_DOMAIN_ID } = require('rclnodejs');
const { Twist } = require('geometry_msgs/msg/twist'); // Import Twist message

function createCmdVelPublisher(nodeName, topicName) {
  const node = createNode(nodeName, ROS_DOMAIN_ID);

  const publisher = node.createPublisher(Twist, topicName);

  // Function to publish a Twist message
  function publishVelocity(linearX, linearY, angularZ) {
    const twistMsg = new Twist();
    twistMsg.linear.x = linearX;
    twistMsg.linear.y = linearY;
    twistMsg.angular.z = angularZ;

    publisher.publish(twistMsg);
  }

  return publishVelocity; // Return the publishVelocity function
}

module.exports = createCmdVelPublisher;
