var camera, scene, renderer, controls;
var axes, grid, particles;
var windowWidth, windowHeight;
var userFrame, trueFrame;
var rotationx = 0.0;
var rotationy = 0.0;
var toDegrees = 180 / Math.PI;

const TRACKER_LENGHT = 300
const USER_COLOR = 0xff0000 // Red
const TRUE_COLOR = 0x00ff00 // Green

let track = []
let trackGT = []
let user_tracker = new CBuffer(TRACKER_LENGHT)
let true_tracker = new CBuffer(TRACKER_LENGHT)

function init() {
	windowWidth = document.getElementById("canvas").offsetWidth
	windowHeight = document.getElementById("canvas").offsetHeight
	camera = new THREE.PerspectiveCamera(75, windowWidth / windowHeight, 0.01, 1000);
	camera.position.z = config.camera.z;
	camera.position.y = config.camera.y;
	camera.position.x = config.camera.x;
	scene = new THREE.Scene();
	renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
	renderer.setSize(windowWidth, windowHeight);
	renderer.setClearColor(0x58D3F7);
	document.getElementById("canvas").appendChild(renderer.domElement);
	controls = new THREE.OrbitControls(camera, renderer.domElement);
	window.addEventListener('resize', onWindowResize, false);
	var ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
	scene.add(ambientLight);
	var light = new THREE.PointLight(0xffffff, 1, 100);
	light.position.set(10, 10, 10);
	scene.add(light);
	var light = new THREE.PointLight(0xffffff, 1, 100);
	light.position.set(20, 20, 20);
	scene.add(light);
	var light = new THREE.PointLight(0xffffff, 1, 100);
	light.position.set(30, 30, 30);
	scene.add(light);
	var light = new THREE.PointLight(0xffffff, 1, 100);
	light.position.set(40, 40, 40);
	scene.add(light);
}

function onWindowResize() {
	windowWidth = document.getElementById("canvas").offsetWidth
	windowHeight = document.getElementById("canvas").offsetHeight

	camera.aspect = windowWidth / windowHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(windowWidth, windowHeight);
}

function animate() {
	requestAnimationFrame(animate);
	renderer.render(scene, camera);
}

function addPoint(point) {
	var geometry = new THREE.Geometry();
	geometry.vertices.push(new THREE.Vector3(point.x, point.y, point.z));

	var material = new THREE.PointsMaterial({ size: config.pointsize, sizeAttenuation: false });
	material.color.setRGB(point.r / 255, point.g / 255, point.b / 255);
	var particles = new THREE.Points(geometry, material);
	particles.position.set(point.x, point.y, point.z);
	particles.name = "points";
	scene.add(particles);
}

function addLine(segment, name) {
	var geometry = new THREE.Geometry();
	geometry.vertices.push(
		new THREE.Vector3(segment.seg.fromPoint.x, segment.seg.fromPoint.z, segment.seg.fromPoint.y),
		new THREE.Vector3(segment.seg.toPoint.x, segment.seg.toPoint.z, segment.seg.toPoint.y),
		new THREE.Vector3(segment.seg.fromPoint.x, segment.seg.fromPoint.z, segment.seg.fromPoint.y));
	var material = new THREE.LineBasicMaterial();
	material.color.setRGB(segment.c.r, segment.c.g, segment.c.b);
	if (name != "plane") {
		material.linewidth = config.linewidth;
	}
	line = new THREE.Line(geometry, material);
	line.name = name;
	scene.add(line);
}

function addGrid() {
	grid = new THREE.GridHelper(1000, 100, 0x888888, 0x888888);
	grid.position.set(0, -0.1, 0);
	scene.add(grid);
}

function deleteObj(name) {
	var selectedObject = scene.getObjectByName(name);
	while (selectedObject != null) {
		scene.remove(selectedObject);
		selectedObject = scene.getObjectByName(name);
	}
}

function addObj(obj, pos) {
	var type = obj.obj.split(":");
	if (type[0] == "https") {
		var url = obj.obj
	} else {
		var file = new Blob([obj.obj], { type: 'text/plain' });
		var url = window.URL.createObjectURL(file);
	}
	if (obj.format == "obj") {
		loadObj(url, obj, pos)
	} else if (obj.format == "dae") {
		loadDae(url, obj, pos);
	}
}

function loadDae(url, obj, pose3d) {
	var loader = new THREE.ColladaLoader();
	var scale = obj.scale;
	loader.load(url, function (collada) {
		var object = collada.scene;
		object.name = obj.id;
		object.position.set(pose3d.x, pose3d.y, pose3d.z);
		object.rotation.set(pose3d.rx * toDegrees, pose3d.ry * toDegrees, pose3d.rz * toDegrees);
		object.scale.set(scale, scale, scale);
		scene.add(object);
	});
}

function loadObj(url, obj, pose3d) {
	var loader = new THREE.OBJLoader();
	var scale = obj.scale;
	loader.load(url,
		function (object) {
			object.name = obj.id;
			object.position.set(pose3d.x, pose3d.y, pose3d.z);
			object.rotation.set(pose3d.rx * toDegrees, pose3d.ry * toDegrees, pose3d.rz * toDegrees);
			object.scale.set(scale, scale, scale);
			scene.add(object);
		},
		function (xhr) { },
		function (error) {
			console.log(error);
		}
	);
}

function moveObj(pose3d) {
	selectedObject = scene.getObjectByName(pose3d.id);
	selectedObject.position.set(pose3d.x, pose3d.y, pose3d.z);
	selectedObject.rotation.set(pose3d.rx * toDegrees, pose3d.ry * toDegrees, pose3d.rz * toDegrees);
}


function addAxis() {
	const axesHelper = new THREE.AxesHelper(10); // The X axis is red. The Y axis is green. The Z axis is blue.
	scene.add(axesHelper);
}


function addSphere(point) {

	const geometry = new THREE.SphereGeometry(config.spheresize, 32, 32);
	const material = new THREE.MeshBasicMaterial();
	material.color.setRGB(point.r / 255, point.g / 255, point.b / 255);
	var sphere = new THREE.Mesh(geometry, material);
	sphere.position.set(point.x, point.y, point.z);
	scene.add(sphere);

}

function addFrames() {
	userFrame = createFrame(1241, 356, 1, USER_COLOR);
	scene.add(userFrame)
	trueFrame = createFrame(1241, 356, 1, TRUE_COLOR);
	scene.add(trueFrame)
}

function createTrack(track, color = 0xff0000) {
    const material = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
    const geometry = new THREE.BufferGeometry().setFromPoints(track.getPoints());
    const line = new THREE.Line(geometry, material);
    return line
}

function reset_scene3d() {
	for (var i = scene.children.length - 1; i >= 0; i--) {
		if (scene.children[i].type == "Mesh") {
			scene.remove(scene.children[i]);
		}
	}
}


function plotTrack(track, color = 0xff0000) {
	const material = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
	const geometry = new THREE.BufferGeometry().setFromPoints(track.getPoints());
	const line = new THREE.Line(geometry, material);
	return line
}


function webGLStart() {
	init();
	addGrid();
	animate();
	addAxis();
	addFrames();
}
