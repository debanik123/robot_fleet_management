
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

var ros = new ROSLIB.Ros({
	url: 'ws://localhost:9090'  // Replace with your ROS bridge server address
  });

class TF {
	constructor() {
		this.fixed_frame = '';

		this.tf_tree = {};
		this.transforms = {};
		this.absoluteTransforms = {};
		//this.absoluteTransformsHistory = new TimeStampedData(20);
		this.frame_list = new Set();
		this.frame_timestamps = {};

		this.tf_topic = new ROSLIB.Topic({
			ros: ros,
			name: '/tf',
			messageType: 'tf2_msgs/msg/TFMessage',
			throttle_rate: 33
		});

		this.tf_listener = this.tf_topic.subscribe((msg) => {
			//local timestamping for removing inactive frames
			const time_stamp = new Date();
			msg.transforms.forEach((pose) => {
				// console.log(pose);
				this.frame_timestamps[pose.child_frame_id] = time_stamp;
				this.frame_timestamps[pose.header.frame_id] = time_stamp;
			})

			this.updateTransforms(msg.transforms, false);
			//this.absoluteTransformsHistory.add(msg.transforms[0].header.stamp, this.absoluteTransforms);
		});

		this.tf_static_topic = new ROSLIB.Topic({
			ros: ros,
			name: 'tf_static',
			messageType: 'tf2_msgs/msg/TFMessage'
		});

		this.tf_static_listener = this.tf_static_topic.subscribe((msg) => {
			this.updateTransforms(msg.transforms, true);
		});

		this.event_timestamp = performance.now();

		window.addEventListener("view_changed", ()=> {
			this.event_timestamp = performance.now();
		});

		//removing inactive TF frames
		setInterval(()=>{
			const now = new Date()
			let deleted_anything = false;
			for (const [frame_id, time_stamp] of Object.entries(this.frame_timestamps)) {
				if(now - time_stamp > 1000 * 10){
					delete this.tf_tree[frame_id];
					delete this.frame_list[frame_id];
					delete this.transforms[frame_id];
					delete this.absoluteTransforms[frame_id];
					deleted_anything = true;
				}
			}

			if(deleted_anything){
				window.dispatchEvent(new Event("tf_changed"));
			}
		},5000)
	}

	async sendUpdateEvent(){
		if(performance.now() - this.event_timestamp > 12){
			window.dispatchEvent(new Event("tf_changed"));
			this.event_timestamp = performance.now();
		}
	}

	addToTree(parentFrameId, childFrameId) {

		if (this.tf_tree.hasOwnProperty(childFrameId)) {
			delete this.tf_tree[childFrameId];
		}
	
		if (this.tf_tree.hasOwnProperty(parentFrameId)) {
			this.tf_tree[parentFrameId][childFrameId] = {};
		} else {
			const foundParent = (node) => {
				for (const key in node) {
					if (key === parentFrameId) {
						node[key][childFrameId] = {};
						return true;
					} else {
						if (foundParent(node[key])) {
							return true;
						}
					}
				}
				return false;
			};
			if (!foundParent(this.tf_tree)) {
				this.tf_tree[parentFrameId] = { [childFrameId]: {} };
			}
		}
	}

	getPathToRoot(frame) {
		const currentFrame = this.transforms[frame];
		if (!currentFrame) {
			return [frame];
		}
		if (!currentFrame.parent) {
			return [frame];
		}
		return [frame].concat(this.getPathToRoot(currentFrame.parent));
	}
	
	findPath(startFrame, endFrame) {
		const p = this.getPathToRoot(startFrame);
		const q = this.getPathToRoot(endFrame);
		
		let common = null;
		while (p.length > 0 && q.length > 0 && p[p.length - 1] === q[q.length - 1]) {
			common = p.pop();
			q.pop();
		}
		// console.log(p);
		return p.concat(common, q.reverse());
	}

	updateTransforms(newtransforms) {
		newtransforms.forEach((pose) => {

			const childFrameId = pose.child_frame_id;
			const parentFrameId = pose.header.frame_id;

			this.frame_list.add(childFrameId);
			this.frame_list.add(parentFrameId);
	
			this.transforms[childFrameId] = {
				translation: pose.transform.translation,
				rotation: new Quaternion(
					pose.transform.rotation.w,
					pose.transform.rotation.x,
					pose.transform.rotation.y,
					pose.transform.rotation.z
				),
				parent: parentFrameId
			};
			this.addToTree(parentFrameId, childFrameId);
		});

		this.recalculateAbsoluteTransforms();
		this.sendUpdateEvent();
	}

	setFixedFrame(newframe) {
		this.fixed_frame = newframe;
		this.recalculateAbsoluteTransforms();
	}

	recalculateAbsoluteTransforms() {
		for (const key of this.frame_list.values()) {
			this.absoluteTransforms[key] = this.transformPose(key, this.fixed_frame, {x: 0, y:0, z:0}, new Quaternion());
		}
	}

	getZeroFrame(){
		return {
			translation:{x: 0, y:0, z:0},
			rotation: new Quaternion()
		}
	}

	transformPose(sourceFrame, targetFrame, inputVector, inputQuat) {

		let outputVector =  Object.assign({}, inputVector);
		let outputQuat =  new Quaternion(inputQuat);

		if(sourceFrame == targetFrame){
			return {
				translation: outputVector,
				rotation: outputQuat
			};
		}

		const path = this.findPath(sourceFrame, targetFrame);

		for (let i = 0; i < path.length - 1; i++) {
			let source = this.transforms[path[i]];

			if(!source)
				source = this.getZeroFrame();

			if(source.parent == path[i+1]){
				outputQuat = source.rotation.mul(outputQuat);
	
				outputVector = applyRotation(outputVector, source.rotation, false);
				outputVector.x += source.translation.x;
				outputVector.y += source.translation.y;
				outputVector.z += source.translation.z;
			}else{
				source = this.transforms[path[i+1]];

				if(!source)
					source = this.getZeroFrame();

				outputQuat = source.rotation.inverse().mul(outputQuat);
	
				outputVector.x -= source.translation.x;
				outputVector.y -= source.translation.y;
				outputVector.z -= source.translation.z;
				outputVector = applyRotation(outputVector, source.rotation, true);
			}
		}
	
		return {
			translation: outputVector,
			rotation: outputQuat
		};
	}
}

let tf = new TF();

// const path = tf.findPath('base_link', 'map');
// var p = tf.transformPose('base_link', 'map', {x: 0, y:0, z:0}, new Quaternion());
// console.log(p);