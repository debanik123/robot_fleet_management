const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
  let msg = rclnodejs.createMessageObject('std_msgs/msg/String');
  let cnt = 0;
  node.createTimer(100, () => {
        msg.data = `msg: ${cnt += 1}`;
        publisher.publish(msg);
    });
    node.spin();
});