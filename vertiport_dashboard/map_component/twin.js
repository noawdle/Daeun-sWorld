import * as THREE from "three";
import {OrbitControls} from "./vendor/three-0.180.0/examples/jsm/controls/OrbitControls.js";

const MODEL_SCALE = 1.53;
const MAX_PATH_POINTS = 180;
const ISO = new THREE.Vector3(1, 1.03, 1);
const AXES = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 1, 0)];

export function toThree([x, y, z]) {
  return new THREE.Vector3(x, z, -y);
}
export function toLocal({x, y, z}) {
  return [x, -z, y];
}
export function attitudeQuaternion([roll, pitch, yaw]) {
  const axisRotation = (axis, angle) => new THREE.Quaternion().setFromAxisAngle(axis, THREE.MathUtils.degToRad(angle));
  return axisRotation(AXES[2], yaw).multiply(axisRotation(AXES[1], pitch)).multiply(axisRotation(AXES[0], roll));
}

function label(text, color, width) {
  const canvas = document.createElement("canvas");
  canvas.width = 256; canvas.height = 64;
  const context = canvas.getContext("2d");
  context.font = "bold 38px system-ui";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillStyle = color;
  context.fillText(text, 128, 32);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map: texture, depthWrite: false}));
  sprite.scale.set(width, width / 4, 1);
  return sprite;
}

function makeDrone() {
  const model = new THREE.Group();
  const materials = {
    body: new THREE.MeshStandardMaterial({color: 0x658eb4, metalness: 0.35, roughness: 0.38}),
    dark: new THREE.MeshStandardMaterial({color: 0x40546e, metalness: 0.25, roughness: 0.55}),
    light: new THREE.MeshStandardMaterial({color: 0xc5d7ec, metalness: 0.25}),
    nose: new THREE.MeshStandardMaterial({color: 0x62e9dc, emissive: 0x11605a}),
  };
  function box(x, y, z, material, position, parent = model) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(x, y, z), materials[material]);
    mesh.position.set(...position);
    parent.add(mesh);
    return mesh;
  }
  box(0.43, 0.18, 0.32, "body", [0, 0.14, 0]);
  box(0.19, 0.07, 0.22, "nose", [0.26, 0.15, 0]);
  box(0.27, 0.035, 0.23, "light", [0, 0.245, 0]);
  const propellers = [];
  for (const x of [-0.39, 0.39]) {
    for (const z of [-0.39, 0.39]) {
      const arm = box(Math.hypot(x, z), 0.075, 0.085, "dark", [x / 2, 0.14, z / 2]);
      arm.rotation.y = -Math.atan2(z, x);
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 0.1, 12), materials.light);
      motor.position.set(x, 0.2, z);
      model.add(motor);
      const rotor = new THREE.Group();
      rotor.position.set(x, 0.265, z);
      box(0.46, 0.022, 0.06, "light", [0, 0, 0], rotor);
      box(0.06, 0.022, 0.46, x > 0 ? "nose" : "dark", [0, 0, 0], rotor);
      model.add(rotor);
      propellers.push(rotor);
    }
  }
  for (const z of [-0.21, 0.21]) {
    box(0.63, 0.044, 0.054, "light", [0, 0.022, z]);
    for (const x of [-0.17, 0.17]) box(0.038, 0.11, 0.04, "dark", [x, 0.085, z]);
  }
  model.scale.setScalar(MODEL_SCALE);
  return {model, propellers};
}

export function mountTwin(container, toolbar, onPick, onFailure) {
  const renderer = new THREE.WebGLRenderer({antialias: true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  const canvas = renderer.domElement;
  canvas.setAttribute("aria-label", "3D 버티포트 장면");
  container.appendChild(canvas);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a1423);
  scene.add(new THREE.HemisphereLight(0xdaeaff, 0x465169, 2.2));
  const light = new THREE.DirectionalLight(0xd7e8ff, 2.1);
  light.position.set(5, 14, 7);
  scene.add(light);
  const camera = new THREE.OrthographicCamera(-14, 14, 12, -12, 0.1, 180);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI / 2 + 0.06;
  controls.minZoom = 0.25;
  controls.maxZoom = 12;
  controls.mouseButtons = {LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN};
  const lifecycle = new AbortController();
  const options = {signal: lifecycle.signal};
  canvas.addEventListener("webglcontextlost", event => {
    event.preventDefault();
    onFailure();
  }, options);
  let aspect = 1, halfHeight = 12, shown = true, follow = false, frame = 0, disposed = false;
  let previousFrame = 0, pickTimer, down, current, source;

  function projection() {
    camera.left = -halfHeight * aspect;
    camera.right = halfHeight * aspect;
    camera.top = halfHeight;
    camera.bottom = -halfHeight;
    camera.updateProjectionMatrix();
  }
  function resize() {
    const {width, height} = container.getBoundingClientRect();
    if (width < 1 || height < 1) return;
    aspect = width / height;
    renderer.setSize(width, height, false);
    projection();
  }
  const observer = new ResizeObserver(resize);
  observer.observe(container);

  const ground = new THREE.Mesh(new THREE.PlaneGeometry(20, 20), new THREE.MeshStandardMaterial({color: 0x12263e}));
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);
  const grid = new THREE.GridHelper(20, 20, 0x476688, 0x29465f);
  grid.position.y = 0.012;
  scene.add(grid);
  for (const [axis, name, color] of [[0, "X", 0xff798b], [1, "Y", 0x5cdda7], [2, "Z", 0x70aaff]]) {
    const origin = toThree([0, 0, 0.03]);
    scene.add(new THREE.ArrowHelper(AXES[axis], origin, axis === 2 ? 5 : 10.5, color, .45, .25));
    const annotation = label(`+${name} (m)`, `#${color.toString(16)}`, 2.2);
    annotation.position.copy(origin).addScaledVector(AXES[axis], axis === 2 ? 5.5 : 10.7);
    scene.add(annotation);
  }
  for (const i of [-10, -5, 0, 5, 10]) {
    for (const point of [[i, -10.55, 0.12], [-10.55, i, 0.12], ...(i > 0 ? [[0, 0, i]] : [])]) {
      const tick = label(i === 0 ? "0 m" : String(i), "#a8bad2", 1.15);
      tick.position.copy(toThree(point));
      scene.add(tick);
    }
  }
  const pad = new THREE.Group();
  const slab = new THREE.Mesh(new THREE.BoxGeometry(2.8, 0.055, 2.8),
    new THREE.MeshStandardMaterial({color: 0x155463}));
  slab.position.y = 0.035;
  pad.add(slab);
  const border = new THREE.LineSegments(new THREE.EdgesGeometry(slab.geometry), new THREE.LineBasicMaterial({color: 0x68ede1}));
  border.position.copy(slab.position);
  pad.add(border);
  const hSign = label("H", "#e1fffc", 4.7);
  const hMesh = new THREE.Mesh(new THREE.PlaneGeometry(2.2, 0.55),
    new THREE.MeshBasicMaterial({map: hSign.material.map, transparent: true, depthWrite: false}));
  hSign.material.dispose();
  hMesh.rotation.x = -Math.PI / 2;
  hMesh.position.y = 0.067;
  pad.add(hMesh);
  scene.add(pad);

  const {model, propellers} = makeDrone();
  model.visible = false;
  scene.add(model);
  const target = new THREE.Mesh(new THREE.OctahedronGeometry(.24), new THREE.MeshBasicMaterial({color: 0xffce70, wireframe:true}));
  const start = new THREE.Mesh(new THREE.OctahedronGeometry(.17), new THREE.MeshBasicMaterial({color: 0x9babca}));
  const currentMarker = new THREE.Mesh(new THREE.RingGeometry(.15,.22,16),
    new THREE.MeshBasicMaterial({color:0x76c2ff, side:THREE.DoubleSide}));
  currentMarker.rotation.x = -Math.PI / 2;
  currentMarker.visible = start.visible = target.visible = false;
  const observed = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: 0x72b5ff}));
  const planned = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({color: 0xffce70, dashSize: .38, gapSize: .24}));
  const groundLink = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({color: 0x8796ad, dashSize: .23, gapSize: .17}));
  scene.add(target, start, currentMarker, observed, planned, groundLink);
  function setLine(line, points) {
    line.geometry.dispose();
    line.geometry = new THREE.BufferGeometry().setFromPoints(points);
    if (line.material.isLineDashedMaterial) line.computeLineDistances();
    line.visible = points.length > 1;
  }

  const destination = new THREE.Vector3();
  const from = new THREE.Vector3();
  const nextRotation = new THREE.Quaternion();
  const fromRotation = new THREE.Quaternion();
  const displacement = new THREE.Vector3();
  let motionAt = 0;
  let trajectory = [];
  function applyData(data) {
    current = data;
    observed.material.color.setHex(data.twin?.mode === "GAZEBO" ? 0x65d8c6 : 0x72b5ff);
    const key = `${data.twin?.mode}/${data.twin?.source}`;
    const changedSource = source !== key;
    source = key;
    pad.position.copy(toThree(data.pad));
    target.position.copy(toThree(data.target));
    target.visible = true;
    setLine(groundLink, [target.position, toThree([data.target[0], data.target[1], 0.08])]);
    trajectory = data.path.slice(-MAX_PATH_POINTS).map(toThree);
    start.visible = trajectory.length > 0;
    if (start.visible) start.position.copy(trajectory[0]).y += 0.18;
    if (!data.position) {
      model.visible = currentMarker.visible = false;
      setLine(planned, []);
      setLine(observed, trajectory);
      setFollow(false);
      return;
    }
    const point = toThree(data.position);
    const rotation = data.twin?.attitude ? attitudeQuaternion(data.twin.attitude) : new THREE.Quaternion();
    if (!model.visible || changedSource) {
      model.position.copy(point);
      model.quaternion.copy(rotation);
      if (changedSource) setFollow(false);
    }
    if (!destination.equals(point) || !nextRotation.equals(rotation) || !model.visible || changedSource) {
      from.copy(model.position);
      fromRotation.copy(model.quaternion);
      destination.copy(point);
      nextRotation.copy(rotation);
      motionAt = performance.now();
    }
    model.visible = currentMarker.visible = true;
    currentMarker.position.copy(point);
    currentMarker.position.y = 0.08;
    setLine(planned, [point, target.position]);
    setLine(observed, [...trajectory, point].slice(-MAX_PATH_POINTS));
  }

  function setFollow(enabled) {
    follow = enabled;
    toolbar.querySelector('[data-view="follow"]').setAttribute("aria-pressed", String(enabled));
    toolbar.querySelector('[data-view="free"]').setAttribute("aria-pressed", String(!enabled));
  }
  function view(direction, focus, radius, depth) {
    setFollow(false);
    halfHeight = radius;
    camera.zoom = 1;
    controls.target.copy(focus);
    // TOP keeps +Y dashboard at the top of the screen, with an epsilon off the pole.
    const distance = Math.max(30, depth + 8);
    camera.position.copy(focus).addScaledVector(direction, distance);
    camera.far = Math.max(80, distance + depth + 8);
    projection();
    controls.update();
  }
  function fit(direction = ISO, useCurrent = true) {
    const bounds = new THREE.Box3(new THREE.Vector3(-10, 0, -10), new THREE.Vector3(10, 0, 10));
    if (!useCurrent) bounds.max.y = 3;
    if (useCurrent && current) {
      bounds.expandByObject(target);
      if (model.visible) {
        const modelBounds = new THREE.Box3().setFromObject(model);
        bounds.union(modelBounds);
        const radius = modelBounds.getSize(new THREE.Vector3()).length() / 2 + 0.35;
        bounds.expandByPoint(destination.clone().addScalar(radius));
        bounds.expandByPoint(destination.clone().addScalar(-radius));
      }
      for (const point of trajectory) bounds.expandByPoint(point);
    }
    const depth = direction.clone().normalize();
    const right = new THREE.Vector3().crossVectors(camera.up, depth).normalize();
    const up = new THREE.Vector3().crossVectors(depth, right);
    const halfSize = bounds.getSize(new THREE.Vector3()).multiplyScalar(0.5);
    const extent = axis => Math.abs(axis.x) * halfSize.x + Math.abs(axis.y) * halfSize.y + Math.abs(axis.z) * halfSize.z;
    const radius = Math.max((extent(right) + 1.2) / aspect, extent(up) + 1.2);
    view(depth, bounds.getCenter(new THREE.Vector3()), radius, extent(depth));
  }
  resize();
  fit(ISO, false);
  toolbar.addEventListener("click", event => {
    switch (event.target.dataset.view) {
      case "top": fit(new THREE.Vector3(0, 1, 0.0001)); break;
      case "front": fit(new THREE.Vector3(0, 0.0001, 1)); break;
      case "side": fit(new THREE.Vector3(1, 0.0001, 0)); break;
      case "iso": fit(); break;
      case "reset": fit(ISO, false); break;
      case "fit": fit(camera.position.clone().sub(controls.target)); break;
      case "follow":
        if (model.visible) {
          camera.position.add(model.position.clone().sub(controls.target));
          controls.target.copy(model.position);
          setFollow(true);
        }
        break;
      case "free": setFollow(false); break;
    }
  }, options);

  const raycaster = new THREE.Raycaster();
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  function intersect(event) {
    const rect = canvas.getBoundingClientRect();
    const cursor = new THREE.Vector2(2*(event.clientX-rect.left)/rect.width-1,
                                     1-2*(event.clientY-rect.top)/rect.height);
    camera.updateMatrixWorld();
    raycaster.setFromCamera(cursor, camera);
    return raycaster.ray.intersectPlane(plane, new THREE.Vector3());
  }
  canvas.addEventListener("pointerdown", event => {
    down = event.button === 0 ? {id: event.pointerId, x: event.clientX, y: event.clientY, dragged: false} : null;
  }, options);
  canvas.addEventListener("pointermove", event => {
    if (down?.id === event.pointerId && Math.hypot(event.clientX-down.x, event.clientY-down.y) > 5) down.dragged = true;
  }, options);
  canvas.addEventListener("pointercancel", () => { down = null; }, options);
  canvas.addEventListener("pointerup", event => {
    const click = down?.id === event.pointerId && !down.dragged && event.button === 0;
    down = null;
    if (!click) return;
    clearTimeout(pickTimer);
    const point = intersect(event);
    if (point) pickTimer = setTimeout(() => onPick(...toLocal(point).slice(0, 2)), 430);
  }, options);
  canvas.addEventListener("dblclick", event => {
    clearTimeout(pickTimer);
    const point = intersect(event);
    if (!point) return;
    setFollow(false);
    camera.position.add(point.clone().sub(controls.target));
    controls.target.copy(point);
    controls.update();
  }, options);

  function animate(now) {
    frame = 0;
    if (disposed || !shown) return;
    if (previousFrame && now - previousFrame < 28) {
      frame = requestAnimationFrame(animate);
      return;
    }
    const dt = previousFrame ? Math.min((now - previousFrame) / 1000, .06) : 0;
    previousFrame = now;
    if (model.visible) {
      const t = THREE.MathUtils.clamp((now - motionAt) / 850, 0, 1);
      displacement.copy(model.position).negate();
      model.position.lerpVectors(from, destination, t);
      displacement.add(model.position);
      model.quaternion.slerpQuaternions(fromRotation, nextRotation, t);
      if (follow) {
        camera.position.add(displacement);
        controls.target.add(displacement);
      }
      if (current?.twin?.mode === "DEMO" && current.twin.armed) {
        propellers.forEach((rotor, index) => { rotor.rotation.y += dt * (index % 2 ? -25 : 25); });
      }
    }
    controls.update();
    renderer.render(scene, camera);
    frame = requestAnimationFrame(animate);
  }
  function setVisible(visible) {
    shown = visible;
    if (visible && !frame && !disposed) {
      resize();
      frame = requestAnimationFrame(animate);
    } else if (!visible) {
      cancelAnimationFrame(frame);
      frame = previousFrame = 0;
    }
  }
  function dispose() {
    if (disposed) return;
    disposed = true;
    cancelAnimationFrame(frame);
    clearTimeout(pickTimer);
    lifecycle.abort();
    observer.disconnect();
    controls.dispose();
    const resources = new Set();
    scene.traverse(object => {
      if (object.geometry) resources.add(object.geometry);
      if (object.material?.map) resources.add(object.material.map);
      if (object.material) resources.add(object.material);
    });
    resources.forEach(resource => resource.dispose());
    renderer.dispose();
    renderer.forceContextLoss();
    canvas.remove();
  }
  setVisible(true);
  return {update: applyData, setVisible, dispose};
}
