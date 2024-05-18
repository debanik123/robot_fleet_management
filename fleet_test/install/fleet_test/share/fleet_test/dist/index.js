"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs = __importStar(require("rclnodejs"));
// Create a node that publishes a msg to the topic 'foo' every 1 second.
// View the topic from the ros2 commandline as shown below:
//     ros2 topic echo foo std_msgs/msg/String
async function example() {
    await rclnodejs.init();
    let node = rclnodejs.createNode('MyNode');
    // Create main working components here, e.g., publisher, subscriber, service, client, action
    // For example, a publisher sending a msg every 1 sec
    let publisher = node.createPublisher('std_msgs/msg/String', 'foo');
    let cnt = 0;
    let msg = rclnodejs.createMessageObject('std_msgs/msg/String');
    node.createTimer(1000, () => {
        msg.data = `msg: ${cnt += 1}`;
        publisher.publish(msg);
    });
    node.spin();
    console.log('Use this command to view the node\'s published messages: ros2 topic echo foo std_msgs/msg/String');
}
(async function main() {
    example();
})().catch(() => {
    process.exitCode = 1;
});
//# sourceMappingURL=index.js.map