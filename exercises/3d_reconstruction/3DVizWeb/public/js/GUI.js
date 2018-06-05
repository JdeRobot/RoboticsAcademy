
var camera, scene, renderer, controls;
var  axes, grid, particles;
var rotationx = 0.0;
var rotationy = 0.0;


			function init() {
				camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 1000 );
				camera.position.z = config.camera.z;
        camera.position.y = config.camera.y;
        camera.position.x = config.camera.x;
				scene = new THREE.Scene();
				renderer = new THREE.WebGLRenderer();
				renderer.setSize( window.innerWidth, window.innerHeight);
        renderer.setClearColor(0x58D3F7);
				document.getElementById("canvas").appendChild( renderer.domElement );
				controls = new THREE.OrbitControls(camera, renderer.domElement);
				window.addEventListener( 'resize', onWindowResize, false );
			}
			function onWindowResize() {
				camera.aspect = window.innerWidth / window.innerHeight;
				camera.updateProjectionMatrix();
				renderer.setSize( window.innerWidth, window.innerHeight );
			}
			function animate() {
				requestAnimationFrame( animate );
				renderer.render( scene, camera );
			}

			function addPoint (point){
				var geometry = new THREE.Geometry();
				geometry.vertices.push( new THREE.Vector3(point.x,point.z,point.y));
				var sprite = new THREE.TextureLoader().load("img/disc.png");
				var material = new THREE.PointsMaterial( { size: config.pointsize, sizeAttenuation: false, map: sprite, alphaTest: 0.5, transparent: true } );
				material.color.setRGB( point.r, point.g, point.b);
				var particles = new THREE.Points( geometry, material );
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

			function deleteObj(name){
				var selectedObject = scene.getObjectByName(name);
				while (selectedObject != null) {
					scene.remove(selectedObject);
					selectedObject = scene.getObjectByName(name);
				}
			}

      function webGLStart (){
        init();
  			animate();
				startWorker();
      }
