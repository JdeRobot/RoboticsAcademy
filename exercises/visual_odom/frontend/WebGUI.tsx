import React, { useState, useEffect, useRef } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, { connectApplication } from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";

const ThreeViewer: React.FC = () => {
  const mountRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;

    // -----------------------
    // SCENE
    // -----------------------
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);

    const camera = new THREE.PerspectiveCamera(
      60,
      mount.clientWidth / mount.clientHeight,
      0.01,
      1000
    );

    camera.position.set(0, -5, 3);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(mount.clientWidth, mount.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);

    mount.innerHTML = "";
    mount.appendChild(renderer.domElement);

    // -----------------------
    // ORBIT CONTROLS (mouse)
    // -----------------------
    const controls = new OrbitControls(camera, renderer.domElement);

    controls.enableDamping = true;
    controls.dampingFactor = 0.08;

    // 🔥 CONTROL DE RATÓN (lo que querías)
    controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.PAN,   // rueda presionada = mover escena
      RIGHT: THREE.MOUSE.DOLLY,  // zoom
    };

    controls.screenSpacePanning = true;
    controls.panSpeed = 1.0;

    controls.target.set(0, 0, 0);
    controls.update();

    // -----------------------
    // GRID
    // -----------------------
    const grid = new THREE.GridHelper(20, 40, 0x444444, 0x222222);
    scene.add(grid);

    // -----------------------
    // AXES (más visibles)
    // -----------------------
    const axes = new THREE.AxesHelper(2.5);
    scene.add(axes);

    // -----------------------
    // RESIZE
    // -----------------------
    const resize = () => {
      const w = mount.clientWidth;
      const h = mount.clientHeight;

      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };

    const ro = new ResizeObserver(resize);
    ro.observe(mount);

    // -----------------------
    // LOOP
    // -----------------------
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };

    animate();

    return () => {
      ro.disconnect();
      controls.dispose();
      renderer.dispose();
    };
  }, []);

  return <div ref={mountRef} style={{ width: "100%", height: "100%" }} />;
};

const WebGUI: React.FC = () => {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState<any>(exerciseContext.manager);

  const [imageSrc, setImageSrc] = useState<string | undefined>(undefined);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext.manager]);

  const updateCallback = (updateData: any) => {
    const update = updateData.update ?? {};

    const raw = update.img1 ?? update.image ?? update.img ?? "";
    if (!raw) return;

    try {
      let base64: string = raw;

      try {
        const parsed = JSON.parse(raw);
        base64 = parsed?.img ?? parsed?.image ?? parsed ?? raw;
      } catch {}

      const finalSrc = base64.startsWith("data:")
        ? base64
        : `data:image/jpeg;base64,${base64}`;

      setImageSrc(finalSrc);
    } catch {
      setImageSrc(undefined);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setImageSrc(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <div
        style={{
          display: "flex",
          width: "100%",
          height: "100%",
          overflow: "hidden",
        }}
      >
        {/* LEFT: IMAGEN (SIN CAMBIOS) */}
        <div
          style={{
            width: "50%",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            padding: 12,
            background: "#000",
            borderRight: "2px solid #333",
            boxSizing: "border-box",
          }}
        >
          {imageSrc ? (
            <WebGUIImage
              id="gui_canvas"
              src={imageSrc}
              style={{
                maxWidth: "50%",
                maxHeight: "90vh",
                width: "auto",
                height: "auto",
                objectFit: "contain",
                display: "block",
              }}
              fit
            />
          ) : (
            <div style={{ color: "#666", fontSize: 14 }}>
              Waiting for image...
            </div>
          )}
        </div>

        {/* RIGHT: 3D VIEWER */}
        <div
          style={{
            width: "50%",
            height: "100%",
            background: "#111",
          }}
        >
          <ThreeViewer />
        </div>
      </div>
    </WebGUIContainer>
  );
};

export default WebGUI;