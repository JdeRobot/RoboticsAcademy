var camera, scene, renderer, controls;
var axes, grid, particles;
var rotationx = 0.0;
var rotationy = 0.0;
var toDegrees = 180/Math.PI;


function init() {
	camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 1000 );
	camera.position.z = config.camera.z;
	camera.position.y = config.camera.y;
	camera.position.x = config.camera.x;

	scene = new THREE.Scene();

	renderer = new THREE.WebGLRenderer();
	var canvas = document.getElementById("canvas");
	renderer.setSize( canvas.offsetWidth, canvas.offsetHeight );
	renderer.setClearColor(0x58D3F7);
	canvas.appendChild( renderer.domElement );

	controls = new THREE.OrbitControls(camera, renderer.domElement);

	window.addEventListener( 'resize', onWindowResize, false );

	var ambientLight = new THREE.AmbientLight( 0xffffff, 0.4 );
	scene.add( ambientLight );

	var light = new THREE.PointLight( 0xffffff, 1, 100 );
	light.position.set( 10, 10, 10 );
	scene.add( light );

	light = new THREE.PointLight( 0xffffff, 1, 100 );
	light.position.set( 20, 20, 20 );
	scene.add( light );

	light = new THREE.PointLight( 0xffffff, 1, 100 );
	light.position.set( 30, 30, 30 );
	scene.add( light );

	light = new THREE.PointLight( 0xffffff, 1, 100 );
	light.position.set( 40, 40, 40 );
	scene.add( light );
}

function onWindowResize() {
	var canvas = document.getElementById("canvas");
	camera.aspect = canvas.offsetWidth / canvas.offsetHeight;
	camera.updateProjectionMatrix();
	renderer.setSize( canvas.offsetWidth, canvas.offsetHeight );
}

function animate() {
	requestAnimationFrame( animate );
	renderer.render( scene, camera );
}

function addPoint (point){
	var geometry = new THREE.Geometry();
	geometry.vertices.push( new THREE.Vector3(point.x,point.y,point.z));
	
	var material = new THREE.PointsMaterial( { size: config.pointsize, sizeAttenuation: false} );
	material.color.setRGB( point.r/255, point.g/255, point.b/255);
	var particles = new THREE.Points( geometry, material );
	particles.position.set(point.x, point.y, point.z);
	particles.name ="points";
	scene.add( particles );
}

function addLine(segment,name){
	var geometry = new THREE.Geometry();
	geometry.vertices.push(
		new THREE.Vector3(segment.seg.fromPoint.x,segment.seg.fromPoint.z,segment.seg.fromPoint.y),
		new THREE.Vector3(segment.seg.toPoint.x,segment.seg.toPoint.z,segment.seg.toPoint.y),
		new THREE.Vector3(segment.seg.fromPoint.x,segment.seg.fromPoint.z,segment.seg.fromPoint.y));
	var material = new THREE.LineBasicMaterial();
	material.color.setRGB(segment.c.r,segment.c.g, segment.c.b);
	if (name != "plane"){
		material.linewidth = config.linewidth;
	}
	line = new THREE.Line(geometry,material);
	line.name = name;
	scene.add(line);
}

function addGrid(){
	grid = new THREE.GridHelper( 1000, 100, 0x888888, 0x888888);
	grid.position.set(0,-0.1,0);
	scene.add(grid);
}

function deleteObj(name){
	var selectedObject = scene.getObjectByName(name);
	while (selectedObject != null) 
	{
		scene.remove(selectedObject);
		selectedObject = scene.getObjectByName(name);
	}
}

function addObj(obj,pos){
	var type = obj.obj.split(":");
	if (type[0] == "https" )
	{
		var url = obj.obj
	} else{
		var file = new Blob([obj.obj], {type:'text/plain'});
		var url  = window.URL.createObjectURL(file);
	}
	if (obj.format == "obj")
	{
		loadObj(url, obj,pos)
	} else if (obj.format == "dae") {
		loadDae(url,obj,pos);
	}
}

function loadDae (url,obj,pose3d){
	var loader = new THREE.ColladaLoader();
	var scale = obj.scale;
	loader.load(url, function (collada) {
				var object = collada.scene;
				object.name = obj.id;
				object.position.set(pose3d.x,pose3d.y,pose3d.z);
				object.rotation.set(pose3d.rx*toDegrees,pose3d.ry * toDegrees, pose3d.rz * toDegrees);
				object.scale.set(scale,scale,scale);
				scene.add( object );} );
}

function loadObj(url,obj,pose3d){
	var loader = new THREE.OBJLoader();
	var scale = obj.scale;
	loader.load(url,
				function(object){
					object.name = obj.id;
					object.position.set(pose3d.x,pose3d.y,pose3d.z);
					object.rotation.set(pose3d.rx*toDegrees,pose3d.ry * toDegrees, pose3d.rz * toDegrees);
					object.scale.set(scale,scale,scale);
					scene.add(object);
				},
				function (xhr){},
				function (error){
					console.log(error);
				}
				);
}

function moveObj(pose3d){
	selectedObject = scene.getObjectByName(pose3d.id);
	selectedObject.position.set(pose3d.x,pose3d.y,pose3d.z);
	selectedObject.rotation.set(pose3d.rx*toDegrees,pose3d.ry * toDegrees, pose3d.rz * toDegrees);
}


function addAxis()
{
	const origin = new THREE.Vector3( 0, 0, 0 );
	const length = 40;
	const headLength = 3;
	const headWidth = 3;

	/* Axles Y */
	const dirY = new THREE.Vector3( 0, 1, 0 );
	dirY.normalize();
	const hexY = 0x037c12; /* Verde */
	const arrowHelperY = new THREE.ArrowHelper( dirY, origin, length, hexY, headLength, headWidth);
	scene.add( arrowHelperY );

	/* Axles X */
	const dirX = new THREE.Vector3( 1, 0, 0 );
	dirX.normalize();
	const hexX = 0xff0000; /* Rojo */
	const arrowHelperX = new THREE.ArrowHelper( dirX, origin, length, hexX, headLength, headWidth);
	scene.add( arrowHelperX );

	/* Axles Z */
	const dirZ = new THREE.Vector3( 0, 0, 1 );
	dirZ.normalize();
	const hexZ = 0x004dff; /* Azul */
	const arrowHelperZ = new THREE.ArrowHelper( dirZ, origin, length, hexZ, headLength, headWidth);
	scene.add( arrowHelperZ );
}


function addSphere (point){

	const geometry = new THREE.SphereGeometry( config.spheresize, 32, 32 );
	const material = new THREE.MeshBasicMaterial();
	material.color.setRGB( point.r/255, point.g/255, point.b/255 );
	var sphere = new THREE.Mesh(geometry, material);
	sphere.position.set(point.x, point.y, point.z);
	scene.add(sphere);
  
}

function reset_scene3d (){
	for( var i = scene.children.length - 1; i >= 0; i--) 
	{
		if(scene.children[i].type == "Mesh")
		{
			scene.remove(scene.children[i]);
		}
	}
}

function webGLStart (){
	init();
	addGrid();
	animate();
	addAxis();
}

