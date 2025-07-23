import * as THREE from "https://cdn.jsdelivr.net/npm/three/build/three.module.js";

const LIGHT_PINK = "#ffb6c1";
const CORNFLOWER_BLUE = "#6495ed";
const MPS_TO_KPH = 3.6;

export async function getJsonParams(requestPath) {
	let parametersList = null;

	try {
		const response = await fetch(requestPath);
		if (!response.ok) {
			throw new Error(`HTTP error! status: ${response.status}`);
		}
		const list = await response.json();
		parametersList = JSON.parse(JSON.stringify(list));
		return parametersList;
	} catch (err) {
		console.error("Failed to load JSON:", err);
		return null;
	}
}

function createSpeedIndicator({
	ros,
	elementId = "speed-indicator",
	paramList,
}) {
	paramList.speedIndicator.id = elementId;
	const speedGauge = new JustGage(paramList.speedIndicator);
	const speedTopic = new ROSLIB.Topic({
		ros,
		name: "odom",
		messageType: "nav_msgs/msg/Odometry",
	});

	function handlePedalMessage(message) {
		const { x, y } = message.twist.twist.linear;
		const speedKph = Math.hypot(x, y) * MPS_TO_KPH;
		speedGauge.refresh(speedKph);
	}

	speedTopic.subscribe(handlePedalMessage);

	return {
		stop: () => speedTopic.unsubscribe(),
	};
}

function startImageStreamViewer({
	ros,
	topicName = "compressed-img",
	elementId = "zed-compressed-image",
}) {
	const imgElement = document.getElementById(elementId);
	const imageTopic = new ROSLIB.Topic({
		ros,
		name: topicName,
		messageType: "sensor_msgs/msg/CompressedImage",
	});

	function updateImageMessage(message) {
		const dataURL = "data:image/png;base64," + message.data;
		imgElement.setAttribute("src", dataURL);
	}

	imageTopic.subscribe(updateImageMessage);

	return {
		stop: () => imageTopic.unsubscribe(),
	};
}

let accelBrakeValue = { accelValue: 0, brakeValue: 0 };

function startAccelBrakeIndicators({
	ros,
	topicName = "handle_controller/joy",
	elementId = "accel-brake-indicators",
	paramList,
}) {
	const canvasContext = document.getElementById(elementId).getContext("2d");
	const scaleFactor = paramList.accelBrakeIndicators.scaleFactor;
	const indicatorsMaxValue = paramList.accelBrakeIndicators.indicatorsMaxValue;

	let accelValue = 0;
	let brakeValue = 0;

	const chart = new Chart(
		canvasContext,
		paramList.accelBrakeIndicators.initChart
	);

	const accelBrakeTopic = new ROSLIB.Topic({
		ros,
		name: topicName,
		messageType: "sensor_msgs/msg/Joy",
	});

	function updateChart(message) {
		const acceleratorPedalInput = message.axes[2]; //Natural: -1.0, Full Throttle: 1.0
		const brakePedalInput = message.axes[3]; //Natural: -1.0, Full Throttle: 1.0

		accelValue = scaleFactor * (acceleratorPedalInput + 1);
		brakeValue = scaleFactor * (brakePedalInput + 1);
		//  Visually emphasize that the brake is applied
		if (brakeValue > 0) {
			brakeValue = indicatorsMaxValue;
		}
		accelBrakeValue.accelValue = accelValue;
		accelBrakeValue.brakeValue = brakeValue;
		chart.data.datasets[0].data = [accelValue, brakeValue];
		chart.update();
	}

	accelBrakeTopic.subscribe(updateChart);

	return {
		stop: () => accelBrakeTopic.unsubscribe(),
	};
}

function startDrivingModeIndicator({
	ros,
	topicName = "twist_mux_gamepad/lock",
	elementId = "vehicle-running-status",
	paramList,
}) {
	const statusElement = document.getElementById(elementId);
	const minimumViewValue = paramList.drivingModeIndicator.minimumViewValue;

	const drivingModeTopic = new ROSLIB.Topic({
		ros,
		name: topicName,
		messageType: "std_msgs/msg/Bool",
	});

	function updateStatus(message) {
		const isManualMode = message.data;
		if (
			isManualMode ||
			accelBrakeValue.accelValue > minimumViewValue ||
			accelBrakeValue.brakeValue > minimumViewValue
		) {
			statusElement.textContent = "Manual";
			statusElement.style.background = LIGHT_PINK;
		} else {
			statusElement.textContent = "Autonomous";
			statusElement.style.background = CORNFLOWER_BLUE;
		}
	}

	drivingModeTopic.subscribe(updateStatus);

	return {
		stop: () => drivingModeTopic.unsubscribe(),
	};
}

function startMapViewer({
	ros,
	topicName = "vectornav/gnss",
	elementId = "map",
	paramList,
}) {
	const map = L.map(elementId, paramList.mapViewer.initMap).setView(
		paramList.mapViewer.mapCenterPosition,
		paramList.mapViewer.initMapSize
	);

	const imageUrl = "images/circuit-birds-eye-view.png";
	const boundsmap = L.latLngBounds(
		paramList.mapViewer.imageTopLeftPosition,
		paramList.mapViewer.imageBottomRightPosition
	);
	const boundsView = L.latLngBounds(
		paramList.mapViewer.viewerTopLeftPosition,
		paramList.mapViewer.viewerBottomRightPosition
	);

	L.imageOverlay(imageUrl, boundsmap).addTo(map);
	const marker = L.circleMarker(
		paramList.mapViewer.mapCenterPosition,
		paramList.mapViewer.initMaker
	).addTo(map);
	map.fitBounds(boundsView);

	const navSatTopic = new ROSLIB.Topic({
		ros,
		name: topicName,
		messageType: "sensor_msgs/msg/NavSatFix",
	});

	function updateMarker(message) {
		const latitude = message.latitude;
		const longitude = message.longitude;
		marker.setLatLng([latitude, longitude]);
	}

	navSatTopic.subscribe(updateMarker);

	return {
		stop: () => navSatTopic.unsubscribe(),
	};
}

function startTwistMuxViewer({
	ros,
	topicName = "twist_mux/cmd_vel",
	elementId = "cmd-vel-arrow-indicator",
	paramList,
}) {
	const canvasElement = document.getElementById(elementId);
	const scene = new THREE.Scene();
	paramList.twistMuxViewer.initCamera.aspect =
		canvasElement.clientWidth / canvasElement.clientHeight;
	const { fov, aspect, near, far } = paramList.twistMuxViewer.initCamera;
	const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
	const renderer = new THREE.WebGLRenderer(
		paramList.twistMuxViewer.initRenderer
	);
	renderer.domElement.classList.add("animation-canvas");
	canvasElement.appendChild(renderer.domElement);
	let arrowHelper = null;
	let vector = new THREE.Vector2(0, 0);
	const scaleFactor = paramList.twistMuxViewer.scaleFactor;
	const vectorMaxLenght = paramList.twistMuxViewer.vectorMaxLenght;

	const twistTopic = new ROSLIB.Topic({
		ros,
		name: topicName,
		messageType: "geometry_msgs/msg/Twist",
	});

	function drawArrow(message) {
		if (arrowHelper) {
			scene.remove(arrowHelper);
			arrowHelper = null;
		}
		// Adjust according to the size of the rendering area and direction
		let angularZVelocity = message.angular.z * -1 * scaleFactor;
		let linerXVelocity = message.linear.x * scaleFactor;

		if (Math.abs(angularZVelocity) > vectorMaxLenght) {
			angularZVelocity = vectorMaxLenght * Math.sign(angularZVelocity);
		}
		if (Math.abs(linerXVelocity) > vectorMaxLenght) {
			linerXVelocity = vectorMaxLenght * Math.sign(linerXVelocity);
		}
		vector = new THREE.Vector2(angularZVelocity, linerXVelocity);

		arrowHelper = new THREE.ArrowHelper(
			new THREE.Vector3(vector.x, vector.y, 0).normalize(),
			new THREE.Vector3(...paramList.twistMuxViewer.arrowInitialPoint),
			vector.length(),
			paramList.twistMuxViewer.arrowcolor,
			paramList.twistMuxViewer.arrowHeadLenght,
			paramList.twistMuxViewer.arrowHeadWidth
		);
		scene.add(arrowHelper);
		camera.position.z = paramList.twistMuxViewer.cameraPositionZ;
	}

	function animate() {
		requestAnimationFrame(animate);
		renderer.render(scene, camera);
	}

	twistTopic.subscribe(drawArrow);
	animate();

	return {
		stop: () => twistTopic.unsubscribe(),
	};
}

async function rostopicViewer() {
	const requestJsonFile = "config/settings.json";
	const parametersList = await getJsonParams(requestJsonFile);

	const ros2 = new ROSLIB.Ros({
		url: "ws://" + location.hostname + ":9090",
	});

	const speedIndicator = createSpeedIndicator({
		ros: ros2,
		elementId: "speed-indicator",
		paramList: parametersList,
	});

	const imageViewer = startImageStreamViewer({
		ros: ros2,
		topicName: "compressed_img",
		elementId: "zed-compressed-image",
	});

	const accelBrakeViewer = startAccelBrakeIndicators({
		ros: ros2,
		topicName: "handle_controller/joy",
		canvasID: "accel-brake-indicators",
		paramList: parametersList,
	});

	const drivingModeIndicator = startDrivingModeIndicator({
		ros: ros2,
		topicName: "twist_mux_gamepad/lock",
		elementId: "vehicle-running-status",
		paramList: parametersList,
	});

	const mapViewer = startMapViewer({
		ros: ros2,
		topicName: "vectornav/gnss",
		elementId: "map",
		paramList: parametersList,
	});

	const twistMuxViewer = startTwistMuxViewer({
		ros: ros2,
		topicName: "twist_mux/cmd_vel",
		elementId: "cmd-vel-arrow-indicator",
		paramList: parametersList,
	});
}

rostopicViewer();
