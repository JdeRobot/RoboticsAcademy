import { config } from "./3DViz";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader";
import {
  GridHelper,
  Scene,
  WebGLRenderer,
  PerspectiveCamera,
  AmbientLight,
  PointLight,
  Vector3,
  PointsMaterial,
  Points,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshBasicMaterial,
  SphereGeometry,
  ArrowHelper,
  BufferGeometry,
} from "three";
let camera, scene, renderer, controls;
let grid;
let toDegrees = 180 / Math.PI;
const factor = 0.5;
function init(canvasRef) {
  camera = new PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    1,
    1000
  );
  camera.position.z = config.camera.z;
  camera.position.y = config.camera.y;
  camera.position.x = config.camera.x;
  scene = new Scene();
  renderer = new WebGLRenderer();
  renderer.setSize(window.innerWidth * factor, window.innerHeight * factor);
  renderer.setClearColor(0x58d3f7);
  console.log(canvasRef.current);
  canvasRef.current.appendChild(renderer.domElement);
  controls = new OrbitControls(camera, renderer.domElement);
  window.addEventListener("resize", onWindowResize, false);
  const ambientLight = new AmbientLight(0xffffff, 0.4);
  scene.add(ambientLight);
  const light = new PointLight(0xffffff, 1, 100);
  light.position.set(10, 10, 10);
  scene.add(light);
  const light2 = new PointLight(0xffffff, 1, 100);
  light2.position.set(20, 20, 20);
  scene.add(light2);
  const light3 = new PointLight(0xffffff, 1, 100);
  light3.position.set(30, 30, 30);
  scene.add(light3);
  const light4 = new PointLight(0xffffff, 1, 100);
  light4.position.set(40, 40, 40);
  scene.add(light4);
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

function addPoint(point) {
  const geometry = new BufferGeometry();
  geometry.vertices.push(new Vector3(point.x, point.y, point.z));

  const material = new PointsMaterial({
    size: config.pointsize,
    sizeAttenuation: false,
  });
  material.color.setRGB(point.r / 255, point.g / 255, point.b / 255);
  const particles = new Points(geometry, material);
  particles.position.set(point.x, point.y, point.z);
  particles.name = "points";
  scene.add(particles);
}

function addLine(segment, name) {
  const geometry = new BufferGeometry();
  geometry.vertices.push(
    new Vector3(
      segment.seg.fromPoint.x,
      segment.seg.fromPoint.z,
      segment.seg.fromPoint.y
    ),
    new Vector3(
      segment.seg.toPoint.x,
      segment.seg.toPoint.z,
      segment.seg.toPoint.y
    ),
    new Vector3(
      segment.seg.fromPoint.x,
      segment.seg.fromPoint.z,
      segment.seg.fromPoint.y
    )
  );
  const material = new LineBasicMaterial();
  material.color.setRGB(segment.c.r, segment.c.g, segment.c.b);
  if (name !== "plane") {
    material.linewidth = config.linewidth;
  }
  let line = new Line(geometry, material);
  line.name = name;
  scene.add(line);
}

function addGrid() {
  grid = new GridHelper(1000, 100, 0x888888, 0x888888);
  grid.position.set(0, -0.1, 0);
  scene.add(grid);
}

function deleteObj(name) {
  let selectedObject = scene.getObjectByName(name);
  while (selectedObject != null) {
    scene.remove(selectedObject);
    selectedObject = scene.getObjectByName(name);
  }
}

function addObj(obj, pos) {
  let url;
  const type = obj.obj.split(":");
  if (type[0] === "https") {
    url = obj.obj;
  } else {
    const file = new Blob([obj.obj], { type: "text/plain" });
    url = window.URL.createObjectURL(file);
  }
  if (obj.format === "obj") {
    loadObj(url, obj, pos);
  } else if (obj.format === "dae") {
    loadDae(url, obj, pos);
  }
}

function loadDae(url, obj, pose3d) {
  const loader = new ColladaLoader();
  const scale = obj.scale;
  loader.load(url, function (collada) {
    const object = collada.scene;
    object.name = obj.id;
    object.position.set(pose3d.x, pose3d.y, pose3d.z);
    object.rotation.set(
      pose3d.rx * toDegrees,
      pose3d.ry * toDegrees,
      pose3d.rz * toDegrees
    );
    object.scale.set(scale, scale, scale);
    scene.add(object);
  });
}

function loadObj(url, obj, pose3d) {
  const loader = new OBJLoader();
  const scale = obj.scale;
  loader.load(
    url,
    function (object) {
      object.name = obj.id;
      object.position.set(pose3d.x, pose3d.y, pose3d.z);
      object.rotation.set(
        pose3d.rx * toDegrees,
        pose3d.ry * toDegrees,
        pose3d.rz * toDegrees
      );
      object.scale.set(scale, scale, scale);
      scene.add(object);
    },
    function (xhr) {
      console.log(xhr);
    },
    function (error) {
      console.log(error);
    }
  );
}

function moveObj(pose3d) {
  const selectedObject = scene.getObjectByName(pose3d.id);
  selectedObject.position.set(pose3d.x, pose3d.y, pose3d.z);
  selectedObject.rotation.set(
    pose3d.rx * toDegrees,
    pose3d.ry * toDegrees,
    pose3d.rz * toDegrees
  );
}

function addAxis() {
  const origin = new Vector3(0, 0, 0);
  const length = 40;
  const headLength = 3;
  const headWidth = 3;

  /* Axles Y */
  const dirY = new Vector3(0, 1, 0);
  dirY.normalize();
  const hexY = 0x037c12; /* Verde */
  const arrowHelperY = new ArrowHelper(
    dirY,
    origin,
    length,
    hexY,
    headLength,
    headWidth
  );
  scene.add(arrowHelperY);

  /* Axles X */
  const dirX = new Vector3(1, 0, 0);
  dirX.normalize();
  const hexX = 0xff0000; /* Rojo */
  const arrowHelperX = new ArrowHelper(
    dirX,
    origin,
    length,
    hexX,
    headLength,
    headWidth
  );
  scene.add(arrowHelperX);

  /* Axles Z */
  const dirZ = new Vector3(0, 0, 1);
  dirZ.normalize();
  const hexZ = 0x004dff; /* Azul */
  const arrowHelperZ = new ArrowHelper(
    dirZ,
    origin,
    length,
    hexZ,
    headLength,
    headWidth
  );
  scene.add(arrowHelperZ);
}

function addSphere(point) {
  const geometry = new SphereGeometry(config.spheresize, 32, 32);
  const material = new MeshBasicMaterial();
  material.color.setRGB(point.r / 255, point.g / 255, point.b / 255);
  const sphere = new Mesh(geometry, material);
  sphere.position.set(point.x, point.y, point.z);
  scene.add(sphere);
}

function reset_scene3d() {
  for (var i = scene.children.length - 1; i >= 0; i--) {
    if (scene.children[i].type === "Mesh") {
      scene.remove(scene.children[i]);
    }
  }
}

function webGLStart(canvasRef) {
  init(canvasRef);
  addGrid();
  animate();
  addAxis();
}

export {
  webGLStart,
  addPoint,
  reset_scene3d,
  addSphere,
  moveObj,
  addLine,
  deleteObj,
  addObj,
  controls,
};
