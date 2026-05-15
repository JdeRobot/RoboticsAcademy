import React, { useState, useEffect, useRef } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, { connectApplication } from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";

const ThreeViewer: React.FC = () => {
  const mountRef = useRef<HTMLDivElement | null>(null);

  const lineRef = useRef<THREE.Line | null>(null);
  const lastGTRef = useRef<[number, number, number] | null>(null);

  const [lastGT, setLastGT] = useState<string>("");

  const GT_SCALE = 0.01;

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;

    const scene = new THREE.Scene();
    scene.scale.x = -1;
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

    // =========================
    // 🔥 RESIZE FIX (AQUÍ ESTÁ LO IMPORTANTE)
    // =========================
    const handleResize = () => {
      if (!mount) return;

      const width = mount.clientWidth;
      const height = mount.clientHeight;

      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };

    // resize del contenedor (cuando cambia el panel)
    const resizeObserver = new ResizeObserver(() => {
      handleResize();
    });

    resizeObserver.observe(mount);

    // resize inicial
    handleResize();

    // =========================
    // CONTROLS (NO TOCADO)
    // =========================
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 0, 0);

    controls.enableZoom = true;
    controls.enablePan = true;

    controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.PAN,
    };

    // GRID + AXES
    scene.add(new THREE.GridHelper(20, 40, 0x2a2a2a, 0x151515));
    scene.add(new THREE.AxesHelper(2.5));

    // LINE
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.LineBasicMaterial({ color: 0x00ff00 });
    const line = new THREE.Line(geometry, material);

    scene.add(line);
    lineRef.current = line;

    // LOOP
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };

    animate();

    return () => {
      window.removeEventListener("resize", handleResize);
      resizeObserver.disconnect();
      renderer.dispose();
      controls.dispose();
    };
  }, []);

  // ===========================
  // GT UPDATE (NO TOCADO)
  // ===========================
  (window as any).updateGTPath = (path: any) => {
    console.log("🔥 [updateGTPath] RAW INPUT:", path);

    if (!path) {
      console.warn("❌ GT path vacío o undefined");
      return;
    }

    try {
      const parsed = typeof path === "string" ? JSON.parse(path) : path;

      console.log("📦 [updateGTPath] PARSED:", parsed);

      if (!Array.isArray(parsed)) {
        console.error("❌ GT no es array:", parsed);
        return;
      }

      const points = parsed.map((p: number[]) =>
        new THREE.Vector3(
          p[0] * GT_SCALE,
          p[1] * GT_SCALE,
          p[2] * GT_SCALE
        )
      );

      const geometry = new THREE.BufferGeometry().setFromPoints(points);

      if (lineRef.current) {
        lineRef.current.geometry.dispose();
        lineRef.current.geometry = geometry;
      }

      const last = points[points.length - 1];
      lastGTRef.current = [last.x, last.y, last.z];

      setLastGT(
        `x: ${last.x.toFixed(2)}, y: ${last.y.toFixed(2)}, z: ${last.z.toFixed(2)}`
      );
    } catch (e) {
      console.error("💥 [GT ERROR PARSE]:", e);
    }
  };

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <div ref={mountRef} style={{ width: "100%", height: "100%" }} />

      <div
        style={{
          position: "absolute",
          top: 10,
          left: 10,
          color: "#00ff00",
          fontFamily: "monospace",
          fontSize: 12,
          background: "rgba(0,0,0,0.7)",
          padding: 8,
        }}
      >
        GT: {lastGT || "None"}
      </div>
    </div>
  );
};

// ===========================
// MAIN GUI (SIN CAMBIOS)
// ===========================
const WebGUI: React.FC = () => {
  const exerciseContext = useExercise();
  const [manager, setManager] = useState<any>(exerciseContext.manager);
  const [imageSrc, setImageSrc] = useState<string | undefined>(undefined);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext.manager]);

  const updateCallback = (updateData: any) => {
    console.log("📡 FULL updateData:", updateData);

    const update = updateData.update ?? {};
    console.log("📦 update:", update);

    const raw = update.img1 ?? update.image ?? update.img ?? "";

    console.log("🧠 possible GT:", update.ground_truth_path);

    if (update.ground_truth_path) {
      console.log("🚀 SENDING GT TO 3D:", update.ground_truth_path.length);
      (window as any).updateGTPath?.(update.ground_truth_path);
    }

    if (!raw) return;

    try {
      let base64 = raw;

      try {
        const parsed = JSON.parse(raw);

        base64 = parsed?.img ?? parsed?.image ?? base64;

        if (parsed?.ground_truth_path) {
          console.log("🚀 GT inside image JSON too!");
          (window as any).updateGTPath?.(parsed.ground_truth_path);
        }
      } catch {}

      const finalSrc = base64.startsWith("data")
        ? base64
        : `data:image/jpeg;base64,${base64}`;

      setImageSrc(finalSrc);
    } catch {
      setImageSrc(undefined);
    }
  };

  const stateCallback = (state: string) => {
    console.log("STATE:", state);
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <div style={{ display: "flex", width: "100%", height: "100%" }}>

        <div style={{ width: "50%", background: "#000" }}>
          {imageSrc ? (
            <WebGUIImage id="gui_canvas" src={imageSrc} fit />
          ) : (
            <div style={{ color: "#666" }}>Waiting for image...</div>
          )}
        </div>

        <div style={{ width: "50%", background: "#111" }}>
          <ThreeViewer />
        </div>

      </div>
    </WebGUIContainer>
  );
};

export default WebGUI;