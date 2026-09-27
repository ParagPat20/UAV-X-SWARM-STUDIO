/* ==========================================================================
   UAV-X Swarm Studio — Three.js Engine & Live Industrial GCS Frontend
   Clean Visual Feedback System (No popups / No Audio SFX)
   ========================================================================== */

// --- Configuration & Constants ---
const WS_URL = `ws://${window.location.hostname || "127.0.0.1"}:8765`;
const DRONE_COLORS = [
  "#00f5d4", // D1: Cyan
  "#a855f7", // D2: Purple
  "#ff007f", // D3: Pink
  "#ffb703", // D4: Amber
  "#00e676", // D5: Emerald
  "#3a86ff", // D6: Blue
  "#fb5607", // D7: Orange
  "#8338ec", // D8: Violet
  "#ff006e", // D9: Coral
  "#00bbf9"  // D10: Sky Blue
];

// --- Application State ---
const state = {
  drones: {},          // id -> drone 3D object & telemetry
  selectedDroneId: 1,
  targetScope: "selected", // "selected" (single UAV) or "all" (entire swarm)
  followSelected: true,    // orbit camera tracks selected drone
  cameraMode: "orbit", // orbit, top, chase
  chaseDist: 38.0,     // Broad tactical chase distance (metres) to capture full point cloud
  chaseHeight: 18.0,   // Elevated tactical chase angle (metres)
  showTrails: true,
  showDropLines: true,
  showLidar: true,
  showPointCloud: true,
  pointCloudPoints: [],
  activeFormation: "line",
  formationConfig: {
    type: "line",
    spacing: 4.0,
    radius: 8.0,
    alt: 5.0,
    heading: 0.0
  },
  surveyConfig: {
    length: 35.0,
    width: 30.0,
    height: 5.0,
    heading: 0.0,
    altitudes: {}
  },
  altitudeHistory: [], // [[t, alt1, alt2, alt3, alt4, alt5], ...]
  origin: null
};

// --- Three.js Globals ---
let scene, camera, renderer, controls;
let container, raycaster, mouse;
let trailsGroup;
let obstaclesGroup, obstaclesInitialized = false;
let pointCloudMesh, pointCloudGeo;
const MAX_POINTCLOUD_PTS = 50000;
const pointCloudPosArray = new Float32Array(MAX_POINTCLOUD_PTS * 3);
const pointCloudColArray = new Float32Array(MAX_POINTCLOUD_PTS * 3);
const pointCloudGrid = new Map();
let radarCanvas, radarCtx, radarSweepAngle = 0;

// --- Transient Drone-Return Flash System ---
// Hits that land on other drones are shown as white sparks for 2s, not stored permanently
const transientFlashes = []; // { mesh: THREE.Points, expiresAt: number }
const DRONE_EXCLUSION_RADIUS = 4.5; // metres — radius around each drone to treat as "drone body" (transient white sparks only, never solidified)
const FLASH_DURATION_MS = 2000;

// --- Persistence & Solidification Timer Controller ---
let solidificationIntervalSec = 1.5;
let solidifyTimer = null;
let isUserDraggingSlider = false;
const LOCAL_STORAGE_SETTINGS_KEY = "uavx_swarm_settings_v1";

function setSolidificationInterval(sec) {
  solidificationIntervalSec = Math.max(0.5, parseFloat(sec) || 1.5);
  if (solidifyTimer) clearInterval(solidifyTimer);
  solidifyTimer = setInterval(solidifyOldPointClusters, solidificationIntervalSec * 1000);
}

function saveSettingsLocally(settings) {
  try {
    const existing = JSON.parse(localStorage.getItem(LOCAL_STORAGE_SETTINGS_KEY) || "{}");
    const merged = { ...existing, ...settings };
    localStorage.setItem(LOCAL_STORAGE_SETTINGS_KEY, JSON.stringify(merged));
  } catch (e) {}
}

function loadLocalSettings() {
  try {
    const saved = localStorage.getItem(LOCAL_STORAGE_SETTINGS_KEY);
    if (saved) return JSON.parse(saved);
  } catch (e) {}
  return null;
}

function applyInitialLocalSettings() {
  const s = loadLocalSettings();
  if (!s) return;
  const setFenceRad = document.getElementById("input-set-fence-radius");
  const valFenceRad = document.getElementById("val-set-fence-radius");
  const setFenceAlt = document.getElementById("input-set-fence-alt");
  const valFenceAlt = document.getElementById("val-set-fence-alt");
  const setAvoidMargin = document.getElementById("input-set-avoid-margin");
  const valAvoidMargin = document.getElementById("val-set-avoid-margin");
  const setSwarmRepel = document.getElementById("input-set-swarm-repel");
  const valSwarmRepel = document.getElementById("val-set-swarm-repel");
  const setSolidifyRate = document.getElementById("input-set-solidify-rate");
  const valSolidifyRate = document.getElementById("val-set-solidify-rate");
  const selectSpawnDrones = document.getElementById("select-spawn-drones");

  if (s.geofence_radius !== undefined && setFenceRad) {
    setFenceRad.value = s.geofence_radius;
    if (valFenceRad) valFenceRad.textContent = `${parseFloat(s.geofence_radius).toFixed(1)} m`;
  }
  if (s.geofence_alt_max !== undefined && setFenceAlt) {
    setFenceAlt.value = s.geofence_alt_max;
    if (valFenceAlt) valFenceAlt.textContent = `${parseFloat(s.geofence_alt_max).toFixed(1)} m`;
  }
  if (s.avoid_margin !== undefined && setAvoidMargin) {
    setAvoidMargin.value = s.avoid_margin;
    if (valAvoidMargin) valAvoidMargin.textContent = `${parseFloat(s.avoid_margin).toFixed(1)} m`;
  }
  if (s.swarm_repel !== undefined && setSwarmRepel) {
    setSwarmRepel.value = s.swarm_repel;
    const minD = s.min_drone_dist !== undefined ? s.min_drone_dist : Math.max(2.5, +(s.swarm_repel / 5).toFixed(1));
    if (valSwarmRepel) valSwarmRepel.textContent = `${parseFloat(minD).toFixed(1)} m (${parseFloat(s.swarm_repel).toFixed(0)}m Repel)`;
  }
  if (s.solidify_rate !== undefined && setSolidifyRate) {
    setSolidifyRate.value = s.solidify_rate;
    if (valSolidifyRate) valSolidifyRate.textContent = `${parseFloat(s.solidify_rate).toFixed(1)} s`;
    setSolidificationInterval(s.solidify_rate);
  }
  if (s.spawn_num_drones !== undefined && selectSpawnDrones) {
    selectSpawnDrones.value = s.spawn_num_drones;
  }
}

// --- Initialize App ---
window.addEventListener("DOMContentLoaded", () => {
  initThreeJS();
  initPointCloud();
  initLidarRadar();
  initWebSocket();
  initUIEventListeners();
  applyInitialLocalSettings();
  initChoreographyModal();
  initSurveyModal();
  initPointCloudModal();
  initWaveChart();
  initFPVVideoDeck();
  
  // Start dynamic SLAM solidification timer
  setSolidificationInterval(solidificationIntervalSec);

  animate();
});

// ==========================================================================
// Three.js 3D Studio Setup
// ==========================================================================
function initThreeJS() {
  container = document.getElementById("canvas-container");
  const width = container.clientWidth;
  const height = container.clientHeight;

  // Scene & Fog
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x121820);
  scene.fog = new THREE.FogExp2(0x121820, 0.0018);

  // Camera
  camera = new THREE.PerspectiveCamera(48, width / height, 0.1, 2000);
  camera.position.set(0, 36, 58);

  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // OrbitControls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2 + 0.02;
  controls.minDistance = 2;
  controls.maxDistance = 600;
  controls.target.set(0, 2, 0);

  // Lighting
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.75);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
  dirLight.position.set(40, 90, 50);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  scene.add(dirLight);

  // Studio Grid & Concentric Tactical Radar Rings
  buildStudioGrid();

  // Trails Group
  trailsGroup = new THREE.Group();
  scene.add(trailsGroup);

  // Fused Solid 3D Obstacles Group
  fusedObstaclesGroup = new THREE.Group();
  scene.add(fusedObstaclesGroup);

  // Downed Drone Emergency Crash Beacons Group
  downedBeaconsGroup = new THREE.Group();
  scene.add(downedBeaconsGroup);

  // Raycaster for Hover & Selection
  raycaster = new THREE.Raycaster();
  mouse = new THREE.Vector2();

  window.addEventListener("resize", onWindowResize);
  container.addEventListener("mousemove", onMouseMove);
  container.addEventListener("click", onMouseClick);
}

function buildStudioGrid() {
  // 1. Dark Base Ground Disc
  const groundGeo = new THREE.CircleGeometry(320, 64);
  const groundMat = new THREE.MeshBasicMaterial({
    color: 0x0e141b,
    depthWrite: false
  });
  const groundMesh = new THREE.Mesh(groundGeo, groundMat);
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.position.y = -0.05;
  scene.add(groundMesh);

  // 2. Primary Tactical Ground Grid (500m wide, 5m minor cells)
  const minorGrid = new THREE.GridHelper(500, 100, 0x1a2634, 0x141e28);
  minorGrid.position.y = 0;
  scene.add(minorGrid);

  // 3. Major Tactical Accented Grid (500m wide, 25m major cells)
  const majorGrid = new THREE.GridHelper(500, 20, 0x00f5d4, 0x1f3042);
  majorGrid.position.y = 0.01;
  majorGrid.material.opacity = 0.25;
  majorGrid.material.transparent = true;
  scene.add(majorGrid);

  // 4. Concentric Tactical Range Circles (15m, 30m, 60m, 100m, 150m, 200m, 250m)
  const ringGroup = new THREE.Group();
  [15, 30, 60, 100, 150, 200, 250].forEach(r => {
    const ringGeo = new THREE.RingGeometry(r - 0.1, r + 0.1, 96);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x00f5d4,
      transparent: true,
      opacity: r <= 60 ? 0.16 : 0.08,
      side: THREE.DoubleSide
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = Math.PI / 2;
    ring.position.y = 0.02;
    ringGroup.add(ring);
  });
  scene.add(ringGroup);
}

// ==========================================================================
// 3D Visual Geofence & Boundary Barrier Visualizer
// ==========================================================================
// 3D RF Link Topology Mesh
// ==========================================================================
let rfMeshGroup = null;
let gcsMarker = null;

function updateRFMeshVisual(dronesData, gcsPos) {
  if (!rfMeshGroup) {
    rfMeshGroup = new THREE.Group();
    scene.add(rfMeshGroup);
  }
  
  // Clear previous lines
  while(rfMeshGroup.children.length > 0){ 
      rfMeshGroup.remove(rfMeshGroup.children[0]); 
  }
  
  if (!gcsPos) return;

  // Add GCS Marker if not present
  if (!gcsMarker) {
    const geo = new THREE.SphereGeometry(1.5, 16, 16);
    const mat = new THREE.MeshBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.8 });
    gcsMarker = new THREE.Mesh(geo, mat);
    scene.add(gcsMarker);
  }
  gcsMarker.position.set(gcsPos[0], gcsPos[1], gcsPos[2]);
  
  // Draw hops
  for (const [idStr, dData] of Object.entries(dronesData)) {
    if (dData.parent_id !== null && dData.parent_id !== undefined) {
      const p1 = new THREE.Vector3(dData.x || 0, dData.y || 0, dData.z || 0);
      let p2 = null;
      if (dData.parent_id === 0) {
        p2 = new THREE.Vector3(gcsPos[0], gcsPos[1], gcsPos[2]);
      } else {
        const parentData = dronesData[dData.parent_id];
        if (parentData) {
          p2 = new THREE.Vector3(parentData.x || 0, parentData.y || 0, parentData.z || 0);
        }
      }
      
      if (p2) {
        const pts = [p1, p2];
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        
        // Link color depends on PDR
        let color = 0x00ff00;
        if (dData.pdr < 50) color = 0xff0000;
        else if (dData.pdr < 85) color = 0xffff00;
        
        const mat = new THREE.LineBasicMaterial({ color: color, transparent: true, opacity: 0.7 });
        const line = new THREE.Line(geo, mat);
        rfMeshGroup.add(line);
      }
    }
  }
}

// ==========================================================================
let geofenceVisualGroup = null;
let currentFenceRadius = null;
let currentFenceAltMax = null;

function updateGeofenceVisual(fenceData, anyBreach) {
  if (!fenceData || !fenceData.enabled) {
    if (geofenceVisualGroup) geofenceVisualGroup.visible = false;
    return;
  }

  const radius = parseFloat(fenceData.radius) || 85.0;
  const altMax = parseFloat(fenceData.alt_max) || 25.0;

  // Dynamically rebuild whenever radius or altitude ceiling change in real-time
  if (!geofenceVisualGroup || currentFenceRadius !== radius || currentFenceAltMax !== altMax) {
    if (geofenceVisualGroup) {
      scene.remove(geofenceVisualGroup);
      geofenceVisualGroup.traverse(child => {
        if (child.isMesh) {
          if (child.geometry) child.geometry.dispose();
          if (child.material) child.material.dispose();
        }
      });
    }

    currentFenceRadius = radius;
    currentFenceAltMax = altMax;
    geofenceVisualGroup = new THREE.Group();
    geofenceVisualGroup.name = "geofence_visual_group";

    // 1. Semi-transparent Cylindrical Boundary Wall
    const cylGeo = new THREE.CylinderGeometry(radius, radius, altMax, 64, 1, true);
    cylGeo.translate(0, altMax / 2, 0);
    const cylMat = new THREE.MeshBasicMaterial({
      color: 0x00f5d4,
      transparent: true,
      opacity: 0.06,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    const cylMesh = new THREE.Mesh(cylGeo, cylMat);
    cylMesh.name = "geofence_cylinder_mesh";
    geofenceVisualGroup.add(cylMesh);

    // 2. Ceiling Grid Disc
    const ceilGeo = new THREE.CircleGeometry(radius, 64);
    ceilGeo.rotateX(Math.PI / 2);
    ceilGeo.translate(0, altMax, 0);
    const ceilMat = new THREE.MeshBasicMaterial({
      color: 0x00f5d4,
      transparent: true,
      opacity: 0.08,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    const ceilMesh = new THREE.Mesh(ceilGeo, ceilMat);
    ceilMesh.name = "geofence_ceiling_mesh";
    geofenceVisualGroup.add(ceilMesh);

    // 3. Perimeter Rings (Base, Mid, Top)
    [0.05, altMax / 2, altMax].forEach((yPos) => {
      const ringGeo = new THREE.RingGeometry(radius - 0.25, radius + 0.25, 96);
      ringGeo.rotateX(Math.PI / 2);
      ringGeo.translate(0, yPos, 0);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0x00f5d4,
        transparent: true,
        opacity: yPos === 0.05 ? 0.6 : 0.35,
        side: THREE.DoubleSide
      });
      const ringMesh = new THREE.Mesh(ringGeo, ringMat);
      ringMesh.name = `geofence_ring_${yPos}`;
      geofenceVisualGroup.add(ringMesh);
    });

    scene.add(geofenceVisualGroup);
  }

  geofenceVisualGroup.visible = true;

  // Pulse color if breach occurs
  const targetColor = anyBreach ? 0xff3b30 : 0x00f5d4;
  geofenceVisualGroup.traverse((child) => {
    if (child.isMesh && child.material) {
      child.material.color.setHex(targetColor);
      if (anyBreach) {
        child.material.opacity = child.name.includes("ring") ? 0.9 : 0.2;
      }
    }
  });
}

// ==========================================================================
// Persistent 2D/3D Point Cloud SLAM Buffer & Accumulator
// ==========================================================================
function initPointCloud() {
  pointCloudGeo = new THREE.BufferGeometry();
  pointCloudGeo.setAttribute('position', new THREE.BufferAttribute(pointCloudPosArray, 3));
  pointCloudGeo.setAttribute('color', new THREE.BufferAttribute(pointCloudColArray, 3));
  pointCloudGeo.setDrawRange(0, 0);

  // Pure white circular particle texture with soft glow (preserves true vertex colors)
  const canvas = document.createElement('canvas');
  canvas.width = 32;
  canvas.height = 32;
  const ctx = canvas.getContext('2d');
  const grad = ctx.createRadialGradient(16, 16, 0, 16, 16, 16);
  grad.addColorStop(0, 'rgba(255, 255, 255, 1.0)');
  grad.addColorStop(0.35, 'rgba(248, 182, 0, 0.9)');
  grad.addColorStop(0.7, 'rgba(17, 255, 0, 0.75)');
  grad.addColorStop(1, 'rgba(3, 27, 246, 0.91)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 32, 32);
  const pointTexture = new THREE.CanvasTexture(canvas);

  const pointMat = new THREE.PointsMaterial({
    size: 1.5,
    map: pointTexture,
    vertexColors: true,
    transparent: true,
    opacity: 0.95,
    blending: THREE.AdditiveBlending,
    depthWrite: false,
    sizeAttenuation: true
  });

  pointCloudMesh = new THREE.Points(pointCloudGeo, pointMat);
  pointCloudMesh.name = "slam_pointcloud";
  scene.add(pointCloudMesh);
}

// Authentic Industrial LiDAR Turbo / Jet Colormap (Velodyne / Ouster / RViz standard)
function getLidarTurboColor(hy, dist) {
  // Height scale: 0.0m (ground) to 14.0m (trees, powerlines, roofs)
  const t = Math.max(0.0, Math.min(1.0, hy / 14.0));

  let r, g, b;
  if (t < 0.15) {
    // 0m - 2m: Deep Blue -> Ice Cyan (ground / low curb)
    const f = t / 0.15;
    r = 0.0;
    g = 0.35 + f * 0.65;
    b = 1.0;
  } else if (t < 0.35) {
    // 2m - 5m: Cyan -> Vivid Emerald Green (hedges, vehicles, lower walls)
    const f = (t - 0.15) / 0.20;
    r = 0.0;
    g = 1.0;
    b = 1.0 - f * 0.95;
  } else if (t < 0.60) {
    // 5m - 8.5m: Green -> Bright Yellow / Gold (walls, roofs, tree crowns)
    const f = (t - 0.35) / 0.25;
    r = f;
    g = 1.0 - f * 0.1;
    b = 0.05;
  } else if (t < 0.82) {
    // 8.5m - 11.5m: Yellow -> Vivid Flame Orange / Red (upper roofs, tall trees)
    const f = (t - 0.60) / 0.22;
    r = 1.0;
    g = 0.9 * (1.0 - f);
    b = 0.0;
  } else {
    // 11.5m - 14m+: Red -> Hot Magenta / Electric Violet (powerlines, high obstacles)
    const f = (t - 0.82) / 0.18;
    r = 1.0;
    g = f * 0.25;
    b = 0.3 + f * 0.7;
  }
  return { r, g, b };
}

// --- High-Performance Fine-Grained Obstacle Solidifier (60 FPS Instanced Mesh) ---
let fusedObstaclesGroup;
const MAX_SOLID_VOXELS = 30000;
let solidInstancedMesh = null;
let solidInstancedCount = 0;

const dummyMatrix = new THREE.Matrix4();
const dummyPos = new THREE.Vector3();
const dummyScale = new THREE.Vector3(0.35, 0.45, 0.35);
const dummyQuat = new THREE.Quaternion();

const solidifiedObstacleMap = new Map(); // "gx_gz" -> { x, y, z, idx }
let pointCloudDirty = false;
let pointCloudHead = 0; // Circular buffer write head

// Discrete Altitude Colormap for 3D Solidified Voxels (Ultra-fast, zero GC overhead)
const SOLID_ALT_COLORS = [
  new THREE.Color(0x00f5d4), // 0.0m - 1.5m : Cyber Cyan (Ground obstacles, curbs)
  new THREE.Color(0x00b4d8), // 1.5m - 3.0m : Electric Blue (Vehicles, low fences)
  new THREE.Color(0x00e676), // 3.0m - 4.5m : Neon Emerald (Lower walls, hedges)
  new THREE.Color(0xa7f432), // 4.5m - 6.0m : Bright Lime (Balconies, mid-walls)
  new THREE.Color(0xffd166), // 6.0m - 8.0m : Amber Gold (Roofs, trees)
  new THREE.Color(0xff8500), // 8.0m - 10.0m: Vivid Flame Orange (Rooftops)
  new THREE.Color(0xff0054)  // 10.0m+      : Bright Crimson / Red (High obstacles, towers)
];

function getSolidVoxelColor(alt) {
  if (alt < 1.5) return SOLID_ALT_COLORS[0];
  if (alt < 3.0) return SOLID_ALT_COLORS[1];
  if (alt < 4.5) return SOLID_ALT_COLORS[2];
  if (alt < 6.0) return SOLID_ALT_COLORS[3];
  if (alt < 8.0) return SOLID_ALT_COLORS[4];
  if (alt < 10.0) return SOLID_ALT_COLORS[5];
  return SOLID_ALT_COLORS[6];
}

function initSolidifiedMesh() {
  if (solidInstancedMesh) return;
  const boxGeo = new THREE.BoxGeometry(1, 1, 1);
  const boxMat = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    transparent: true,
    opacity: 0.82,
    depthWrite: true
  });
  solidInstancedMesh = new THREE.InstancedMesh(boxGeo, boxMat, MAX_SOLID_VOXELS);
  solidInstancedMesh.count = 0;
  solidInstancedMesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(MAX_SOLID_VOXELS * 3), 3);
  solidInstancedMesh.name = "solid_instanced_mesh";
  if (fusedObstaclesGroup) fusedObstaclesGroup.add(solidInstancedMesh);
}

function recordPointCloudHit(hx, hy, hz, dist) {
  if (!state.showPointCloud) return;

  // Filter out ground/floor hits (hy < 0.6m) so open roads and ground stay clean
  if (hy < 0.6) return;

  // Filter out ANY hit near live drones (prevent false obstacles on open roads)
  for (const dObj of Object.values(state.drones)) {
    const dp = dObj.group.position;
    if (Math.sqrt((hx - dp.x)**2 + (hy - dp.y)**2 + (hz - dp.z)**2) < DRONE_EXCLUSION_RADIUS) {
      spawnDroneReturnFlash(hx, hy, hz);
      return;
    }
  }

  // Spatial cell quantization (0.25m) for crisp point cloud resolution
  const cellSize = 0.25;
  const gx = Math.round(hx / cellSize);
  const gy = Math.round(hy / cellSize);
  const gz = Math.round(hz / cellSize);
  const key = `${gx}_${gy}_${gz}`;

  if (pointCloudGrid.has(key)) return;
  pointCloudGrid.set(key, true);

  const { r, g, b } = getLidarTurboColor(hy, dist);
  const now = Date.now();

  let targetIdx;
  if (state.pointCloudPoints.length < MAX_POINTCLOUD_PTS) {
    targetIdx = state.pointCloudPoints.length;
    state.pointCloudPoints.push({ x: hx, y: hy, z: hz, r, g, b, key, t: now });
  } else {
    // O(1) Ring Buffer - Overwrite oldest without O(N) Array.shift()
    targetIdx = pointCloudHead;
    const oldPt = state.pointCloudPoints[targetIdx];
    if (oldPt && oldPt.key) {
      pointCloudGrid.delete(oldPt.key);
    }
    state.pointCloudPoints[targetIdx] = { x: hx, y: hy, z: hz, r, g, b, key, t: now };
    pointCloudHead = (pointCloudHead + 1) % MAX_POINTCLOUD_PTS;
  }

  pointCloudPosArray[targetIdx * 3] = hx;
  pointCloudPosArray[targetIdx * 3 + 1] = hy;
  pointCloudPosArray[targetIdx * 3 + 2] = hz;
  pointCloudColArray[targetIdx * 3] = r;
  pointCloudColArray[targetIdx * 3 + 1] = g;
  pointCloudColArray[targetIdx * 3 + 2] = b;

  pointCloudDirty = true;
}

// Flush point cloud GPU buffers once per frame / telemetry cycle
function flushPointCloudUpdates() {
  if (!pointCloudDirty || !pointCloudGeo) return;
  pointCloudGeo.attributes.position.needsUpdate = true;
  pointCloudGeo.attributes.color.needsUpdate = true;
  pointCloudGeo.setDrawRange(0, state.pointCloudPoints.length);
  pointCloudDirty = false;

  const countElem = document.getElementById("top-pointcloud-count");
  if (countElem) countElem.textContent = `${state.pointCloudPoints.length.toLocaleString()} pts`;
}

// Fine-Grained 2D/3D Obstacle Solidifier: Solidifies only actual obstacle points into crisp connected geometry
function solidifyOldPointClusters() {
  if (!solidInstancedMesh && fusedObstaclesGroup) {
    initSolidifiedMesh();
  }
  if (!solidInstancedMesh || state.pointCloudPoints.length < 10) return;

  const now = Date.now();
  const cellSize = 0.35; // 35cm fine-grained obstacle voxel
  const liveDronePositions = Object.values(state.drones).map(d => d.group.position);
  const retainedPoints = [];
  let newSolidCount = 0;

  for (let i = 0; i < state.pointCloudPoints.length; i++) {
    const pt = state.pointCloudPoints[i];
    if (!pt) continue;
    const age = now - (pt.t || (now - 10000));

    // Purge stale noise
    if (age > 15000) {
      if (pt.key) pointCloudGrid.delete(pt.key);
      continue;
    }

    // STRICT: Do NOT solidify ground level hits (y < 0.8m) or white/dynamic returns
    if (pt.y < 0.8) {
      if (pt.key) pointCloudGrid.delete(pt.key);
      continue;
    }

    const isWhite = (pt.r > 0.85 && pt.g > 0.85 && pt.b > 0.85) || pt.is_dynamic;
    if (isWhite) {
      if (pt.key) pointCloudGrid.delete(pt.key);
      continue;
    }

    let nearDrone = false;
    for (const dp of liveDronePositions) {
      if (Math.sqrt((pt.x - dp.x)**2 + (pt.y - dp.y)**2 + (pt.z - dp.z)**2) < 4.5) {
        nearDrone = true;
        break;
      }
    }
    if (nearDrone) {
      if (pt.key) pointCloudGrid.delete(pt.key);
      continue;
    }

    // If point is stable (>400ms), solidify ONLY its specific fine location on real obstacle walls
    if (age >= 400 && solidInstancedCount < MAX_SOLID_VOXELS) {
      const gx = Math.round(pt.x / cellSize);
      const gz = Math.round(pt.z / cellSize);
      const cellKey = `${gx}_${gz}`;

      if (!solidifiedObstacleMap.has(cellKey)) {
        const ox = gx * cellSize;
        const oz = gz * cellSize;
        const oy = Math.max(0.4, pt.y);

        // Add 1 crisp solid voxel at exact obstacle coordinates with height gradient
        dummyPos.set(ox, oy, oz);
        dummyMatrix.compose(dummyPos, dummyQuat, dummyScale);
        solidInstancedMesh.setMatrixAt(solidInstancedCount, dummyMatrix);

        const vColor = getSolidVoxelColor(oy);
        solidInstancedMesh.setColorAt(solidInstancedCount, vColor);

        solidifiedObstacleMap.set(cellKey, { x: ox, y: oy, z: oz, idx: solidInstancedCount });
        solidInstancedCount++;
        newSolidCount++;
      }
      if (pt.key) pointCloudGrid.delete(pt.key);
    } else {
      retainedPoints.push(pt);
    }
  }

  if (newSolidCount > 0) {
    solidInstancedMesh.count = solidInstancedCount;
    solidInstancedMesh.instanceMatrix.needsUpdate = true;
    if (solidInstancedMesh.instanceColor) {
      solidInstancedMesh.instanceColor.needsUpdate = true;
    }
  }

  if (retainedPoints.length < state.pointCloudPoints.length) {
    state.pointCloudPoints = retainedPoints;
    pointCloudHead = 0;
    for (let i = 0; i < state.pointCloudPoints.length; i++) {
      const pt = state.pointCloudPoints[i];
      pointCloudPosArray[i * 3] = pt.x;
      pointCloudPosArray[i * 3 + 1] = pt.y;
      pointCloudPosArray[i * 3 + 2] = pt.z;
      pointCloudColArray[i * 3] = pt.r;
      pointCloudColArray[i * 3 + 1] = pt.g;
      pointCloudColArray[i * 3 + 2] = pt.b;
    }
    pointCloudDirty = true;
    flushPointCloudUpdates();
  }
}

function clearPointCloudMap() {
  state.pointCloudPoints = [];
  pointCloudHead = 0;
  pointCloudGrid.clear();
  solidifiedObstacleMap.clear();
  if (pointCloudGeo) {
    pointCloudGeo.setDrawRange(0, 0);
    pointCloudGeo.attributes.position.needsUpdate = true;
    pointCloudGeo.attributes.color.needsUpdate = true;
  }
  // Clear Instanced Mesh
  if (solidInstancedMesh) {
    solidInstancedCount = 0;
    solidInstancedMesh.count = 0;
    solidInstancedMesh.instanceMatrix.needsUpdate = true;
  }
  // Remove transient flashes
  for (const flash of transientFlashes) {
    scene.remove(flash.mesh);
    if (flash.mesh.geometry) flash.mesh.geometry.dispose();
    if (flash.mesh.material) flash.mesh.material.dispose();
  }
  transientFlashes.length = 0;
  const countElem = document.getElementById("top-pointcloud-count");
  if (countElem) countElem.textContent = "0 pts";
  logEvent("LiDAR Point Cloud & Solid Objects Cleared.");
}

// Spawn a single transient white spark at the drone-return hit position
function spawnDroneReturnFlash(hx, hy, hz) {
  if (!scene || !state.showPointCloud) return;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute([hx, hy, hz], 3));
  const mat = new THREE.PointsMaterial({
    color: 0xffffff,
    size: 2.2,
    transparent: true,
    opacity: 0.92,
    depthWrite: false,
    sizeAttenuation: true,
    blending: THREE.AdditiveBlending
  });
  const spark = new THREE.Points(geo, mat);
  spark.name = 'drone_return_flash';
  scene.add(spark);
  transientFlashes.push({ mesh: spark, expiresAt: Date.now() + FLASH_DURATION_MS });
}

// ==========================================================================
// 3D Procedural UAV Quadcopter Mesh Builder
// ==========================================================================
function createDrone3DObject(id, hexColor) {
  const group = new THREE.Group();
  group.name = `drone_${id}`;
  group.userData = { id: id, color: hexColor };

  const colorObj = new THREE.Color(hexColor);

  // Central Avionics Pod
  const bodyGeo = new THREE.CylinderGeometry(0.38, 0.46, 0.16, 8);
  const bodyMat = new THREE.MeshPhongMaterial({
    color: 0x18202c,
    specular: 0x334455,
    shininess: 30
  });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.castShadow = true;
  group.add(body);

  // Top Team Color Glowing Beacon Dome
  const domeGeo = new THREE.SphereGeometry(0.24, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2);
  const domeMat = new THREE.MeshPhongMaterial({
    color: colorObj,
    emissive: colorObj,
    emissiveIntensity: 0.9,
    shininess: 80
  });
  const dome = new THREE.Mesh(domeGeo, domeMat);
  dome.position.y = 0.08;
  group.add(dome);

  // Carbon Fiber X-Arms & Motor Mounts
  const armMat = new THREE.MeshPhongMaterial({ color: 0x0d1218, shininess: 10 });
  const motorMat = new THREE.MeshPhongMaterial({ color: 0x2e3d52, shininess: 40 });
  const propMat = new THREE.MeshPhongMaterial({
    color: colorObj,
    transparent: true,
    opacity: 0.75,
    shininess: 60
  });

  const armLength = 0.85;
  const props = [];
  const angles = [Math.PI / 4, (3 * Math.PI) / 4, (5 * Math.PI) / 4, (7 * Math.PI) / 4];

  angles.forEach((angle, i) => {
    const x = Math.cos(angle) * armLength;
    const z = Math.sin(angle) * armLength;

    // Arm Bar
    const armGeo = new THREE.BoxGeometry(0.06, 0.04, armLength);
    const arm = new THREE.Mesh(armGeo, armMat);
    arm.position.set(x / 2, 0, z / 2);
    arm.rotation.y = -angle + Math.PI / 2;
    group.add(arm);

    // Motor Bell
    const motorGeo = new THREE.CylinderGeometry(0.1, 0.1, 0.1, 12);
    const motor = new THREE.Mesh(motorGeo, motorMat);
    motor.position.set(x, 0.05, z);
    group.add(motor);

    // Propeller Blade
    const propGeo = new THREE.BoxGeometry(0.55, 0.012, 0.07);
    const prop = new THREE.Mesh(propGeo, propMat);
    prop.position.set(x, 0.11, z);
    group.add(prop);
    props.push(prop);
  });

  group.userData.props = props;

  // Altitude Drop-Line
  const lineGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, -10, 0)
  ]);
  const lineMat = new THREE.LineDashedMaterial({
    color: colorObj,
    dashSize: 0.35,
    gapSize: 0.25,
    opacity: 0.6,
    transparent: true
  });
  const dropLine = new THREE.Line(lineGeo, lineMat);
  dropLine.computeLineDistances();
  dropLine.visible = false;
  scene.add(dropLine);

  // Ground Target Disc
  const shadowGeo = new THREE.RingGeometry(0.35, 0.7, 24);
  const shadowMat = new THREE.MeshBasicMaterial({
    color: colorObj,
    transparent: true,
    opacity: 0.35,
    side: THREE.DoubleSide
  });
  const groundDisc = new THREE.Mesh(shadowGeo, shadowMat);
  groundDisc.rotation.x = Math.PI / 2;
  groundDisc.visible = false;
  scene.add(groundDisc);

  // 3D Trajectory Ribbon Line
  const trailGeo = new THREE.BufferGeometry();
  const trailMat = new THREE.LineBasicMaterial({
    color: colorObj,
    linewidth: 2,
    transparent: true,
    opacity: 0.85
  });
  const trailLine = new THREE.Line(trailGeo, trailMat);
  trailLine.visible = false;
  trailsGroup.add(trailLine);

  scene.add(group);

  return {
    group: group,
    dropLine: dropLine,
    groundDisc: groundDisc,
    trailLine: trailLine,
    targetPos: new THREE.Vector3(0, 0, 0),
    targetRot: new THREE.Euler(0, 0, 0),
    telemetry: {}
  };
}

// ==========================================================================
// Live WebSocket Telemetry & Command Dispatcher
// ==========================================================================
function initWebSocket() {
  const wsIndicator = document.getElementById("ws-indicator");
  let socket = null;

  function connect() {
    socket = new WebSocket(WS_URL);

    socket.onopen = () => {
      wsIndicator.textContent = "ONLINE";
      wsIndicator.className = "connection-pill live";
      setFeedbackBanner("SITL MAVLink Connected", "Ready for swarm command dispatch", "ok");
      logEvent("Connected to UAV-X Swarm GCS Telemetry Stream.");
    };

    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data) {
          updateSwarmState(data);
        }
      } catch (err) { }
    };

    socket.onclose = () => {
      wsIndicator.textContent = "CONNECTING...";
      wsIndicator.className = "connection-pill";
      setFeedbackBanner("Connecting to SITL...", "Waiting for ports 5762-5802", "alert");
      setTimeout(connect, 2000);
    };

    socket.onerror = () => {
      socket.close();
    };
  }

  connect();

  window.sendGCSCommand = (cmd) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(cmd));
    }
  };
}

// ==========================================================================
// Real-Time Telemetry & HUD State Synchronization
// ==========================================================================
function updateSwarmState(data) {
  const swarm = data.swarm || {};
  const dronesData = data.drones || {};
  const meshLinks = data.mesh_links || [];

  // Synchronize Settings Tab from Server Persistence
  if (data.settings && !isUserDraggingSlider) {
    const s = data.settings;
    const setFenceRad = document.getElementById("input-set-fence-radius");
    const valFenceRad = document.getElementById("val-set-fence-radius");
    const setFenceAlt = document.getElementById("input-set-fence-alt");
    const valFenceAlt = document.getElementById("val-set-fence-alt");
    const setAvoidMargin = document.getElementById("input-set-avoid-margin");
    const valAvoidMargin = document.getElementById("val-set-avoid-margin");
    const setSwarmRepel = document.getElementById("input-set-swarm-repel");
    const valSwarmRepel = document.getElementById("val-set-swarm-repel");
    const setSolidifyRate = document.getElementById("input-set-solidify-rate");
    const valSolidifyRate = document.getElementById("val-set-solidify-rate");

    if (setFenceRad && document.activeElement !== setFenceRad && s.geofence_radius !== undefined) {
      setFenceRad.value = s.geofence_radius;
      if (valFenceRad) valFenceRad.textContent = `${parseFloat(s.geofence_radius).toFixed(1)} m`;
    }
    if (setFenceAlt && document.activeElement !== setFenceAlt && s.geofence_alt_max !== undefined) {
      setFenceAlt.value = s.geofence_alt_max;
      if (valFenceAlt) valFenceAlt.textContent = `${parseFloat(s.geofence_alt_max).toFixed(1)} m`;
    }
    if (setAvoidMargin && document.activeElement !== setAvoidMargin && s.avoid_margin !== undefined) {
      setAvoidMargin.value = s.avoid_margin;
      if (valAvoidMargin) valAvoidMargin.textContent = `${parseFloat(s.avoid_margin).toFixed(1)} m`;
    }
    if (setSwarmRepel && document.activeElement !== setSwarmRepel && s.swarm_repel !== undefined) {
      setSwarmRepel.value = s.swarm_repel;
      const minD = s.min_drone_dist !== undefined ? s.min_drone_dist : Math.max(2.5, +(s.swarm_repel / 5).toFixed(1));
      if (valSwarmRepel) valSwarmRepel.textContent = `${parseFloat(minD).toFixed(1)} m (${parseFloat(s.swarm_repel).toFixed(0)}m Repel)`;
    }
    if (setSolidifyRate && document.activeElement !== setSolidifyRate && s.solidify_rate !== undefined) {
      setSolidifyRate.value = s.solidify_rate;
      if (valSolidifyRate) valSolidifyRate.textContent = `${parseFloat(s.solidify_rate).toFixed(1)} s`;
    }
    saveSettingsLocally(s);
  }

  const activeCount = swarm.active_count || 0;
  const avgAlt = swarm.avg_alt || 0;
  const avgBatt = swarm.avg_battery || 100;
  const allArmed = swarm.all_armed || false;

  // Header & Status Strip
  document.getElementById("header-swarm-count").textContent = `${activeCount} UAVs`;
  document.getElementById("inspector-count-badge").textContent = `${activeCount} UAVs ONLINE`;
  document.getElementById("bottom-speed").textContent = `${(swarm.max_speed || 0).toFixed(1)} m/s`;

  if (swarm.origin) {
    document.getElementById("bottom-datum").textContent = `LAT: ${swarm.origin[0].toFixed(5)} | LON: ${swarm.origin[1].toFixed(5)}`;
  }

  // HUD Card 1: Machine Info (if present)
  const armedElem = document.getElementById("hud-swarm-armed-status");
  if (armedElem) {
    if (allArmed) {
      armedElem.textContent = "ARMED ⚡";
      armedElem.className = "highlight-green";
    } else {
      armedElem.textContent = "DISARMED 🔒";
      armedElem.className = "highlight-red";
    }
  }

  const flightStatElem = document.getElementById("hud-swarm-flight-status");
  if (flightStatElem) {
    if (avgAlt > 0.5) {
      flightStatElem.textContent = `AIRBORNE (${avgAlt.toFixed(1)}m)`;
      flightStatElem.className = "highlight-blue";
    } else if (allArmed) {
      flightStatElem.textContent = "ARMED (ON GROUND)";
      flightStatElem.className = "highlight-green";
    } else {
      flightStatElem.textContent = "READY (STANDBY)";
      flightStatElem.className = "highlight-green";
    }
  }

  // Update Swarm Flight Mode Deck Card
  const activeDronesList = Object.values(dronesData);
  if (activeDronesList.length > 0) {
    const primaryMode = activeDronesList[0].mode || "GUIDED";
    const deckModeElem = document.getElementById("deck-current-mode");
    if (deckModeElem) deckModeElem.textContent = primaryMode;
    document.querySelectorAll(".mode-chip[data-mode]").forEach(chip => {
      chip.classList.toggle("active", chip.dataset.mode === primaryMode);
    });
  }

  // Battery State (if HUD card present)
  const battBigElem = document.getElementById("hud-batt-big");
  if (battBigElem) battBigElem.textContent = `${avgBatt}%`;
  const b1 = document.getElementById("batt-c1");
  const b2 = document.getElementById("batt-c2");
  const b3 = document.getElementById("batt-c3");
  if (b1 && b2 && b3) {
    b1.classList.toggle("filled", avgBatt > 15);
    b2.classList.toggle("filled", avgBatt > 45);
    b3.classList.toggle("filled", avgBatt > 75);
  }

  // Record Altitude History for Golden Wave Spline Chart
  const now = Date.now();
  const alts = [1, 2, 3, 4, 5].map(id => (dronesData[id]?.alt || 0));
  state.altitudeHistory.push({ t: now, alts: alts });
  if (state.altitudeHistory.length > 50) {
    state.altitudeHistory.shift();
  }

  // Sync / Create Drone 3D Entities
  for (const [idStr, dData] of Object.entries(dronesData)) {
    const id = parseInt(idStr);
    if (!state.drones[id]) {
      const hexColor = dData.color || DRONE_COLORS[(id - 1) % DRONE_COLORS.length];
      state.drones[id] = createDrone3DObject(id, hexColor);
      logEvent(`Drone D${id} registered in 3D Studio.`);
    }

    const dObj = state.drones[id];
    dObj.telemetry = dData;

    // Target position in Three.js coordinates (X, Y=Alt, Z=-North)
    dObj.targetPos.set(dData.x || 0, dData.y || 0, dData.z || 0);

    // Target rotation
    const pitchRad = THREE.MathUtils.degToRad(dData.pitch || 0);
    const rollRad = THREE.MathUtils.degToRad(dData.roll || 0);
    const yawRad = -THREE.MathUtils.degToRad(dData.heading || dData.yaw || 0);
    dObj.targetRot.set(pitchRad, yawRad, rollRad, 'YXZ');

    // Update Ribbon Trail
    if (state.showTrails && dData.trail && dData.trail.length > 1) {
      dObj.trailLine.visible = true;
      const pts = dData.trail.map(p => new THREE.Vector3(p[0], p[1], p[2]));
      dObj.trailLine.geometry.setFromPoints(pts);
    } else {
      dObj.trailLine.visible = false;
    }

    // Render Real-Time Future Predicted Trajectory (Dashed Reactive Avoidance Spline)
    if (dData.predicted_trajectory && dData.predicted_trajectory.length > 1) {
      if (!dObj.predLine) {
        const lineGeo = new THREE.BufferGeometry();
        const lineMat = new THREE.LineDashedMaterial({
          color: dData.color ? parseInt(dData.color.replace('#', '0x')) : 0x00f5d4,
          dashSize: 0.9,
          gapSize: 0.45,
          linewidth: 2.5,
          transparent: true,
          opacity: 0.95
        });
        dObj.predLine = new THREE.Line(lineGeo, lineMat);
        dObj.predLine.name = `drone_${id}_pred_trajectory`;
        scene.add(dObj.predLine);
      }
      dObj.predLine.visible = true;
      const rawPts = dData.predicted_trajectory.map(p => new THREE.Vector3(p[0], p[1], p[2]));
      let curvePts = rawPts;
      if (rawPts.length >= 4) {
        try {
          const curve = new THREE.CatmullRomCurve3(rawPts, false, 'centripetal', 0.5);
          curvePts = curve.getPoints(36);
        } catch (e) {
          curvePts = rawPts;
        }
      }
      dObj.predLine.geometry.setFromPoints(curvePts);
      dObj.predLine.computeLineDistances();
    } else if (dObj.predLine) {
      dObj.predLine.visible = false;
    }

    // Ingest all LiDAR 360 points from this drone into the Persistent 3D SLAM Point Cloud map
    // BUT — if the hit point lies within DRONE_EXCLUSION_RADIUS of any other drone's 3D position,
    // treat it as an inter-drone return: show a temporary white spark (2s), do NOT store permanently.
    if (dData.lidar_hits && dData.lidar_hits.length > 0) {
      const hits = dData.lidar_hits;

      // Build a snapshot of every OTHER drone's current 3D world position
      const otherDronePositions = [];
      for (const [otherId, otherObj] of Object.entries(state.drones)) {
        if (parseInt(otherId) !== id) {
          otherDronePositions.push(otherObj.group.position); // live THREE.Vector3
        }
      }

      for (let idx = 0; idx < hits.length; idx++) {
        const [hx, hy, hz, dist] = hits[idx];

        // Check proximity to every other drone
        let isDroneReturn = false;
        for (const pos of otherDronePositions) {
          const dx = hx - pos.x;
          const dy = hy - pos.y;
          const dz = hz - pos.z;
          if (Math.sqrt(dx * dx + dy * dy + dz * dz) < DRONE_EXCLUSION_RADIUS) {
            isDroneReturn = true;
            break;
          }
        }

        if (isDroneReturn) {
          // Show a temporary white spark — do NOT persist in the SLAM map
          spawnDroneReturnFlash(hx, hy, hz);
        } else {
          recordPointCloudHit(hx, hy, hz, dist);
        }
      }
    }
  }

  // Render 3D Geofence Barrier & Check Containment Breaches (Only while armed & flying)
  const anyBreach = Object.values(dronesData).some(d => d.geofence_breach && d.armed && (d.alt || 0) > 0.8);
  updateGeofenceVisual(swarm.geofence, anyBreach);

  // Render RF Mesh Links
  updateRFMeshVisual(dronesData, swarm.gcs_pos);

  // Geofence Breach Top Overlay Alert (Debounced to once every 12 seconds per breach event)
  if (anyBreach) {
    const breachedDrones = Object.values(dronesData).filter(d => d.geofence_breach && d.armed && (d.alt || 0) > 0.8);
    const firstBreached = breachedDrones[0] || {};
    const breachedId = firstBreached.id || "Swarm";
    const fenceRadius = (swarm.geofence && swarm.geofence.radius) ? swarm.geofence.radius : 85;
    const now = Date.now();

    if (now - lastGeofenceBreachAlertTime > 12000) {
      lastGeofenceBreachAlertTime = now;
      logEvent(`⚠️ GEOFENCE VIOLATION: UAV ${breachedId} exceeded boundary limit of ${fenceRadius}m!`);
      setFeedbackBanner(`⚠️ GEOFENCE BREACH: UAV ${breachedId}`, `Exceeded safety radius of ${fenceRadius}m! Containment active.`, "alert");

      showMissionOverlay({
        id: `geofence_breach_${breachedId}`,
        type: "danger",
        badge: "⚠️ GEOFENCE BOUNDARY BREACH",
        title: `UAV ${breachedId} Exceeded Safe Operational Zone!`,
        message: `UAV ${breachedId} reached boundary limit (${fenceRadius}m radius). Swarm elastic containment is actively redirecting the aircraft.`,
        autoDismissSeconds: 15,
        actions: [
          {
            label: "🔙 Recenter Swarm",
            className: "btn-recenter",
            icon: "🎯",
            onClick: () => {
              sendGCSCommand({ action: "recenter_swarm", target: "all", params: {} });
              logEvent("🎯 Swarm recentering commanded towards origin.");
              setFeedbackBanner("RECENTERING", "Swarm vectors oriented towards center.", "exec");
            }
          },
          {
            label: "🛡️ Expand Geofence (+20m)",
            className: "btn-secondary-hover",
            icon: "➕",
            onClick: () => {
              const newRadius = fenceRadius + 20;
              sendGCSCommand({ action: "set_geofence", target: "all", params: { enabled: true, radius: newRadius, alt_max: 25 } });
              const slider = document.getElementById("slider-geofence-radius");
              const label = document.getElementById("label-geofence-radius");
              if (slider) slider.value = newRadius;
              if (label) label.textContent = `${newRadius}m`;
              logEvent(`🛡️ Geofence safety radius expanded to ${newRadius}m.`);
              setFeedbackBanner("GEOFENCE UPDATED", `Boundary radius set to ${newRadius}m`, "exec");
            }
          },
          {
            label: "🏠 Swarm RTL",
            className: "btn-danger-rtl",
            icon: "🏠",
            onClick: () => {
              sendGCSCommand({ action: "mode", target: "all", params: { mode: "RTL" } });
              logEvent("🏠 Emergency Swarm RTL commanded due to geofence breach.");
              setFeedbackBanner("SWARM RTL", "All UAVs returning to launch origin.", "warn");
            }
          },
          {
            label: "Dismiss",
            className: "btn-secondary-hover",
            dismiss: true
          }
        ]
      });
    }
  }

  // Low Battery RTL Overlay Alert
  const anyLowBattery = Object.values(dronesData).some(d => d.low_battery_rtb && d.armed);
  if (anyLowBattery) {
    const lowBattDrones = Object.values(dronesData).filter(d => d.low_battery_rtb && d.armed);
    const firstLowBatt = lowBattDrones[0] || {};
    const lowBattId = firstLowBatt.id || "Swarm";
    const now = Date.now();

    if (now - lastLowBatteryAlertTime > 20000) {
      lastLowBatteryAlertTime = now;
      logEvent(`⚠️ LOW BATTERY: UAV ${lowBattId} dropped below 25% SoC!`);
      setFeedbackBanner(`⚠️ LOW BATTERY: UAV ${lowBattId}`, `State of Charge critical. Auto RTL triggered.`, "alert");

      showMissionOverlay({
        id: `low_batt_${lowBattId}`,
        type: "warning",
        badge: "🔋 CRITICAL BATTERY ALERT",
        title: `UAV ${lowBattId} State of Charge Low (<25%)`,
        message: `UAV ${lowBattId} has reached critical battery thresholds. Autonomous Return-To-Launch (RTL) has been triggered to preserve the asset.`,
        autoDismissSeconds: 15,
        actions: [
          {
            label: "🔄 Acknowledge",
            className: "btn-secondary-hover",
            icon: "✅",
            dismiss: true
          }
        ]
      });
    }
  }

  // Sync Downed Drones 3D Crash Beacons & SAR Trajectories
  syncDownedBeacons(data.downed_drones);

  // Render Telemetry Inspector Cards
  renderDroneCards(dronesData);
}

// ==========================================================================
let missionOverlayTimeout = null;
let lastGeofenceBreachAlertTime = 0;
let lastLowBatteryAlertTime = 0;
let lastActiveOverlayId = null;

function playMissionAlertChime() {
  try {
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = "sine";
    osc.frequency.setValueAtTime(587.33, ctx.currentTime);
    osc.frequency.setValueAtTime(880.00, ctx.currentTime + 0.12);
    gain.gain.setValueAtTime(0.2, ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.01, ctx.currentTime + 0.35);
    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.4);
  } catch (e) {}
}

function showMissionOverlay(options) {
  const container = document.getElementById("top-mission-overlay");
  const card = document.getElementById("mission-overlay-card");
  const badgeText = document.getElementById("overlay-badge-text");
  const timeElem = document.getElementById("overlay-time");
  const titleElem = document.getElementById("overlay-title");
  const msgElem = document.getElementById("overlay-message");
  const actionsContainer = document.getElementById("overlay-actions");
  const closeBtn = document.getElementById("btn-overlay-close");

  if (!container || !card) return;

  if (missionOverlayTimeout) {
    clearTimeout(missionOverlayTimeout);
    missionOverlayTimeout = null;
  }

  const type = options.type || "warn";
  card.className = `mission-overlay-card type-${type}`;

  if (badgeText) badgeText.textContent = options.badge || "MISSION NOTIFICATION";
  if (titleElem) titleElem.textContent = options.title || "Swarm Advisory";
  if (msgElem) msgElem.textContent = options.message || "";
  if (timeElem) timeElem.textContent = new Date().toLocaleTimeString();

  if (actionsContainer) {
    actionsContainer.innerHTML = "";
    (options.actions || []).forEach(act => {
      const btn = document.createElement("button");
      btn.className = `overlay-act-btn ${act.className || 'btn-secondary-hover'}`;
      btn.innerHTML = `${act.icon ? `<span class="act-icon">${act.icon}</span>` : ''} <span>${act.label}</span>`;
      btn.onclick = () => {
        if (typeof act.onClick === 'function') {
          act.onClick();
        }
        if (act.dismiss !== false) {
          hideMissionOverlay();
        }
      };
      actionsContainer.appendChild(btn);
    });
  }

  if (closeBtn) {
    closeBtn.onclick = () => hideMissionOverlay();
  }

  container.style.display = "block";
  lastActiveOverlayId = options.id || null;

  if (options.autoDismissSeconds && options.autoDismissSeconds > 0) {
    missionOverlayTimeout = setTimeout(() => {
      hideMissionOverlay();
    }, options.autoDismissSeconds * 1000);
  }
}

function hideMissionOverlay() {
  const container = document.getElementById("top-mission-overlay");
  if (container) {
    container.style.display = "none";
  }
  lastActiveOverlayId = null;
  if (missionOverlayTimeout) {
    clearTimeout(missionOverlayTimeout);
    missionOverlayTimeout = null;
  }
}

// ==========================================================================
// 3D Downed Drone Emergency Crash Beacons & SAR Handover Engine
// ==========================================================================
let downedBeaconsGroup;
const activeDownedBeacons = new Map(); // id -> { group, ring, sarLine, rescuerId }
const sarArrivedAlerted = new Set(); // Set of downed_ids that have triggered arrival alert

function syncDownedBeacons(downedList) {
  if (!downedBeaconsGroup) return;
  const currentIds = new Set((downedList || []).map(d => d.id));

  // Remove beacons for revived drones
  for (const [id, beacon] of activeDownedBeacons.entries()) {
    if (!currentIds.has(id)) {
      downedBeaconsGroup.remove(beacon.group);
      if (beacon.sarLine) scene.remove(beacon.sarLine);
      activeDownedBeacons.delete(id);
      sarArrivedAlerted.delete(id);
      if (lastActiveOverlayId === `sar_arrival_${id}`) {
        hideMissionOverlay();
      }
    }
  }

  // Create or update beacons for downed drones
  (downedList || []).forEach(item => {
    const id = item.id;
    let beacon = activeDownedBeacons.get(id);

    if (!beacon) {
      const bGroup = new THREE.Group();
      bGroup.position.set(item.x || 0, Math.max(0.1, item.y || 0), item.z || 0);

      // 1. Red glowing crash cylinder pillar
      const cylGeo = new THREE.CylinderGeometry(0.5, 0.5, 6.0, 16);
      const cylMat = new THREE.MeshBasicMaterial({
        color: 0xff385c,
        transparent: true,
        opacity: 0.35,
        depthWrite: false
      });
      const cyl = new THREE.Mesh(cylGeo, cylMat);
      cyl.position.y = 3.0;
      bGroup.add(cyl);

      // 2. Pulsing ground crash beacon rings
      const ringGeo = new THREE.RingGeometry(0.5, 2.5, 32);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0xff385c,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.85
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.05;
      bGroup.add(ring);

      // 3. Red Alert Cross Icon on top
      const crossGeo = new THREE.BoxGeometry(0.25, 1.4, 0.25);
      const crossMat = new THREE.MeshBasicMaterial({ color: 0xff385c });
      const c1 = new THREE.Mesh(crossGeo, crossMat);
      c1.position.y = 6.2;
      const c2 = c1.clone();
      c2.rotation.z = Math.PI / 2;
      bGroup.add(c1);
      bGroup.add(c2);

      downedBeaconsGroup.add(bGroup);

      // Emergency SAR Dashed Trajectory Line
      const sarGeo = new THREE.BufferGeometry();
      const sarMat = new THREE.LineDashedMaterial({
        color: 0xffaa00,
        dashSize: 1.2,
        gapSize: 0.6,
        linewidth: 3,
        transparent: true,
        opacity: 0.95
      });
      const sarLine = new THREE.Line(sarGeo, sarMat);
      sarLine.name = `sar_route_uav_${id}`;
      scene.add(sarLine);

      beacon = { group: bGroup, ring: ring, sarLine: sarLine, rescuerId: item.assigned_rescuer };
      activeDownedBeacons.set(id, beacon);

      logEvent(`🚨 CRASH BEACON DEPLOYED: UAV ${id} at (${item.x.toFixed(1)}, ${item.z.toFixed(1)})`);
      setFeedbackBanner(`UAV ${id} FAILED / CRASHED`, `Nearest UAV ${item.assigned_rescuer || 'Fleet'} dispatched for SAR!`, "warn");
    }

    // Update SAR dashed line and Check Arrival of Rescuer Drone
    beacon.rescuerId = item.assigned_rescuer;
    if (item.assigned_rescuer && state.drones[item.assigned_rescuer]) {
      const rescuerDrone = state.drones[item.assigned_rescuer];
      const rescuerPos = rescuerDrone.group.position;
      const targetPos = beacon.group.position;
      const dist = Math.sqrt((rescuerPos.x - targetPos.x)**2 + (rescuerPos.z - targetPos.z)**2);

      if (beacon.sarLine) {
        beacon.sarLine.geometry.setFromPoints([rescuerPos, targetPos]);
        beacon.sarLine.computeLineDistances();
        beacon.sarLine.visible = true;
      }

      // If rescuer is near crash site (< 6.0m) or backend flags arrival, trigger the Top Screen Alert Notification Overlay with actions!
      const isArrived = dist < 6.0 || Boolean(item.sar_arrived) || Boolean(rescuerDrone?.telemetry?.sar_arrived);
      const arrivalKey = `${id}_${item.assigned_rescuer}`;
      if (isArrived && !sarArrivedAlerted.has(arrivalKey)) {
        sarArrivedAlerted.add(arrivalKey);
        const rescuerId = item.assigned_rescuer;
        const arrivalMsg = `🚨 UAV ${rescuerId} reached UAV ${id}'s accidental area!`;
        setFeedbackBanner(`🚨 SAR ARRIVAL: UAV ${rescuerId}`, `UAV ${rescuerId} is near UAV ${id}'s accidental area (${dist.toFixed(1)}m)! Holding SAR position.`, "warn");
        logEvent(arrivalMsg);
        playMissionAlertChime();

        // Top Mission HUD Overlay with interactive action choices
        showMissionOverlay({
          id: `sar_arrival_${id}`,
          type: "warn",
          badge: "🚨 SEARCH & RESCUE ARRIVAL",
          title: `UAV ${rescuerId} Reached UAV ${id}'s Crash Coordinates`,
          message: `UAV ${rescuerId} is holding inspection hover directly over the accident coordinates (${targetPos.x.toFixed(1)}m, ${targetPos.z.toFixed(1)}m, Alt: ${targetPos.y.toFixed(1)}m). Select mission action:`,
          actions: [
            {
              label: "▶ Continue Swarm Survey",
              className: "btn-primary-survey",
              icon: "🛰️",
              onClick: () => {
                sendGCSCommand({ action: "continue_survey", target: "all", params: { downed_id: id } });
                logEvent(`▶ Swarm autonomous survey resumed by operator.`);
                setFeedbackBanner("SURVEY RESUMED", "Swarm fleet fanning out on autonomous exploration.", "exec");
              }
            },
            {
              label: `🔄 Revive UAV ${id}`,
              className: "btn-primary-survey",
              icon: "⚡",
              onClick: () => {
                window.reviveDrone(id);
              }
            },
            {
              label: "🛸 Hold Observation Hover",
              className: "btn-secondary-hover",
              icon: "👁️",
              onClick: () => {
                logEvent(`🛸 UAV ${rescuerId} holding SAR inspection hover over UAV ${id}.`);
                setFeedbackBanner("SAR HOVER ACTIVE", `UAV ${rescuerId} stationary over UAV ${id} crash site.`, "warn");
              }
            },
            {
              label: "🏠 Swarm RTL",
              className: "btn-danger-rtl",
              icon: "🏠",
              onClick: () => {
                sendGCSCommand({ action: "mode", target: "all", params: { mode: "RTL" } });
                logEvent("🏠 Operator commanded Swarm Return To Launch.");
                setFeedbackBanner("SWARM RTL", "All UAVs returning to launch origin.", "warn");
              }
            },
            {
              label: "Dismiss",
              className: "btn-secondary-hover",
              dismiss: true
            }
          ]
        });
      }
    } else if (beacon.sarLine) {
      beacon.sarLine.visible = false;
    }
  });
}

// Global helper commands for Kill / Revive
window.killDrone = function(id) {
  const numId = parseInt(id);
  sendGCSCommand({ action: "kill_drone", target: numId, params: { drone_id: numId } });
  logEvent(`💀 FAULT INJECTED: UAV ${numId} motor cutoff & crash simulated.`);
  setFeedbackBanner(`UAV ${numId} MOTOR CUTOFF`, "Swarm self-healing SAR re-routing triggered.", "warn");
};

window.reviveDrone = function(id) {
  const numId = parseInt(id);
  sendGCSCommand({ action: "revive_drone", target: numId, params: { drone_id: numId } });
  logEvent(`🔄 UAV ${numId} revived and restored to swarm fleet.`);
  setFeedbackBanner(`UAV ${numId} RESTORED`, "UAV re-joined active swarm telemetry.", "exec");
};

window.reviveAllDrones = function() {
  sendGCSCommand({ action: "revive_drone", target: "all" });
  logEvent(`🔄 ALL UAVs revived and restored to swarm fleet.`);
  setFeedbackBanner("SWARM FLEET RESTORED", "All drones re-joined active fleet.", "exec");
};

// ==========================================================================
// Right Inspector UI Cards Renderer (In-Place DOM Mutation - Zero Shaking)
// ==========================================================================
function renderDroneCards(dronesData) {
  const container = document.getElementById("drones-list");
  if (!container) return;

  const droneIds = Object.keys(dronesData).map(Number).sort((a, b) => a - b);
  if (droneIds.length === 0) {
    if (!container.querySelector(".empty-state")) {
      container.innerHTML = `<div class="empty-state" style="padding:20px;text-align:center;color:var(--text-muted);font-size:11px;">Waiting for SITL swarm heartbeat...</div>`;
    }
    return;
  }

  const emptyState = container.querySelector(".empty-state");
  if (emptyState) emptyState.remove();

  if (typeof window.updateFpvTabs === "function") {
    window.updateFpvTabs(droneIds);
  }

  droneIds.forEach(id => {
    const d = dronesData[id];
    const isSelected = id === state.selectedDroneId;
    const isArmed = Boolean(d.armed);
    const mode = d.mode || "GUIDED";
    const modeClass = mode.toLowerCase();
    const flightPhase = d.flight_phase || "STANDBY";
    const statusMsg = d.last_status_msg || "Connected to Flight Controller";
    const isWarning = (d.status_severity <= 4) || statusMsg.toLowerCase().includes("fail") || statusMsg.toLowerCase().includes("prearm");

    const batt = d.battery || 100;
    const speed = (d.speed || 0).toFixed(1);
    const alt = (d.alt || 0).toFixed(1);
    const heading = (d.heading || 0).toFixed(0);

    let card = document.getElementById(`dcard-${id}`);
    if (!card) {
      card = document.createElement("div");
      card.id = `dcard-${id}`;
      card.className = `drone-card ${isSelected ? 'selected' : ''}`;
      card.style.setProperty("--drone-color", d.color || DRONE_COLORS[(id - 1) % DRONE_COLORS.length]);
      card.onclick = () => selectDrone(id);

      card.innerHTML = `
        <div class="dcard-header">
          <div class="dcard-left-title">
            <div class="dcard-team-dot"></div>
            <span class="dcard-name">UAV ${id}</span>
          </div>
          <div class="dcard-badges-group">
            <span class="arm-badge" id="dcard-arm-${id}"></span>
            <select class="dcard-mode-select" id="dcard-mode-select-${id}" onchange="changeDroneMode(${id}, this.value)" onclick="event.stopPropagation()" title="Change Flight Mode for UAV ${id}">
              <option value="GUIDED">GUIDED</option>
              <option value="AUTO">AUTO</option>
              <option value="LOITER">LOITER</option>
              <option value="RTL">RTL</option>
              <option value="LAND">LAND</option>
              <option value="POSHOLD">POSHOLD</option>
              <option value="STABILIZE">STABILIZE</option>
              <option value="BRAKE">BRAKE</option>
            </select>
          </div>
        </div>

        <div class="flight-phase-banner" id="dcard-phase-banner-${id}">
          <span class="phase-icon" id="dcard-phase-icon-${id}"></span>
          <span class="phase-text" id="dcard-phase-text-${id}"></span>
        </div>

        <div class="autopilot-msg-box" id="dcard-msg-box-${id}" title="MAVLink Autopilot Log">
          <span class="ap-icon" id="dcard-msg-icon-${id}">💬</span>
          <span class="ap-text" id="dcard-msg-text-${id}"></span>
        </div>

        <div class="dcard-stats-grid">
          <div class="dstat">
            <span class="dstat-label">ALT</span>
            <span class="dstat-val" id="dcard-alt-${id}">0.0m</span>
          </div>
          <div class="dstat">
            <span class="dstat-label">SPD</span>
            <span class="dstat-val" id="dcard-spd-${id}">0.0m/s</span>
          </div>
          <div class="dstat">
            <span class="dstat-label">HDG</span>
            <span class="dstat-val" id="dcard-hdg-${id}">0°</span>
          </div>
          <div class="dstat">
            <span class="dstat-label">BATT</span>
            <span class="dstat-val highlight-green" id="dcard-batt-${id}">100%</span>
          </div>
        </div>

        <div class="dcard-quick-actions" onclick="event.stopPropagation()">
          <button id="btn-uav-arm-${id}" class="mini-action-btn" onclick="sendIndividualCommand(${id}, 'arm', this)">Arm</button>
          <button id="btn-uav-takeoff-${id}" class="mini-action-btn" onclick="sendIndividualCommand(${id}, 'takeoff', this)">Takeoff</button>
          <button id="btn-uav-rtl-${id}" class="mini-action-btn" onclick="sendIndividualCommand(${id}, 'rtl', this)">RTL</button>
          <button id="btn-uav-land-${id}" class="mini-action-btn" onclick="sendIndividualCommand(${id}, 'land', this)">Land</button>
          <button id="btn-uav-kill-${id}" class="mini-action-btn kill" onclick="killDrone(${id})" title="Cut motors & simulate crash (triggers nearest drone SAR)">💀 Kill</button>
        </div>
      `;
      container.appendChild(card);
    }

    // In-place updates without DOM reconstruction
    card.classList.toggle("selected", isSelected);
    const isFailed = Boolean(d.failed) || (mode && mode.includes("FAILED")) || flightPhase.includes("CRASHED");
    card.classList.toggle("failed-drone", isFailed);

    const killBtn = document.getElementById(`btn-uav-kill-${id}`);
    if (killBtn) {
      if (isFailed) {
        killBtn.textContent = "🔄 Revive";
        killBtn.className = "mini-action-btn revive";
        killBtn.onclick = (e) => { e.stopPropagation(); reviveDrone(id); };
      } else {
        killBtn.textContent = "💀 Kill";
        killBtn.className = "mini-action-btn kill";
        killBtn.onclick = (e) => { e.stopPropagation(); killDrone(id); };
      }
    }

    const armBadge = document.getElementById(`dcard-arm-${id}`);
    if (armBadge) {
      if (isFailed) {
        armBadge.textContent = 'FAILED 💀';
        armBadge.className = 'arm-badge disarmed';
      } else {
        armBadge.textContent = isArmed ? 'ARMED ⚡' : 'DISARMED 🔒';
        armBadge.className = `arm-badge ${isArmed ? 'armed' : 'disarmed'}`;
      }
    }

    const modeSelect = document.getElementById(`dcard-mode-select-${id}`);
    if (modeSelect && document.activeElement !== modeSelect) {
      modeSelect.value = mode;
    }

    const phaseBanner = document.getElementById(`dcard-phase-banner-${id}`);
    const phaseIcon = document.getElementById(`dcard-phase-icon-${id}`);
    const phaseText = document.getElementById(`dcard-phase-text-${id}`);
    if (phaseBanner && phaseIcon && phaseText) {
      phaseBanner.className = `flight-phase-banner ${d.alt > 0.5 ? 'climbing' : (isArmed ? 'auto' : 'disarmed')}`;
      phaseIcon.textContent = d.alt > 0.5 ? '🛫' : (isArmed ? '🟢' : '🛑');
      phaseText.textContent = flightPhase;
    }

    const msgBox = document.getElementById(`dcard-msg-box-${id}`);
    const msgIcon = document.getElementById(`dcard-msg-icon-${id}`);
    const msgText = document.getElementById(`dcard-msg-text-${id}`);
    if (msgBox && msgIcon && msgText) {
      msgBox.className = `autopilot-msg-box ${isWarning ? 'warning' : ''}`;
      msgIcon.textContent = isWarning ? '⚠️' : '💬';
      msgText.textContent = statusMsg;
    }

    const altElem = document.getElementById(`dcard-alt-${id}`);
    if (altElem) altElem.textContent = `${alt}m`;

    const spdElem = document.getElementById(`dcard-spd-${id}`);
    if (spdElem) spdElem.textContent = `${speed}m/s`;

    const hdgElem = document.getElementById(`dcard-hdg-${id}`);
    if (hdgElem) hdgElem.textContent = `${heading}°`;

    const battElem = document.getElementById(`dcard-batt-${id}`);
    if (battElem) battElem.textContent = `${batt}%`;
  });
}

window.selectDrone = function (id) {
  state.selectedDroneId = id;
  state.followSelected = true;

  // Highlight selected card in the right inspector
  document.querySelectorAll(".drone-card").forEach(c => c.classList.remove("selected"));
  document.getElementById(`dcard-${id}`)?.classList.add("selected");

  // Sync Spatial Move & WSADUJ target label
  const targetLabel = document.getElementById("move-target-label");
  if (targetLabel && state.targetScope === "selected") {
    targetLabel.textContent = `UAV ${id}`;
  }
  const selBtn = document.getElementById("btn-target-selected");
  if (selBtn) {
    selBtn.textContent = `🎯 UAV ${id}`;
  }

  // In Chase mode, smoothly switch chase camera to the newly selected drone
  if (state.cameraMode === "chase") {
    const d = state.drones[id];
    if (d) {
      const dPos = d.group.position;
      controls.target.copy(dPos);
      const hdgRad = THREE.MathUtils.degToRad(d.telemetry.heading || 0);
      const chaseDist = state.chaseDist || 38.0;
      const chaseHeight = state.chaseHeight || 18.0;
      const idealCam = new THREE.Vector3(
        dPos.x - Math.sin(hdgRad) * chaseDist,
        dPos.y + chaseHeight,
        dPos.z + Math.cos(hdgRad) * chaseDist
      );
      camera.position.copy(idealCam);
    }
  }

  logEvent(`Selected UAV ${id} in Object Inspector`);
};

window.sendIndividualCommand = function (id, action, btnElem) {
  const alt = parseFloat(document.getElementById("slider-alt")?.value || 5.0);
  sendGCSCommand({ action: action, target: id, params: { alt: alt } });

  // In-place button confirmation
  if (btnElem) {
    const origText = btnElem.textContent;
    btnElem.textContent = "✓ SENT";
    btnElem.classList.add("dispatched");
    setTimeout(() => {
      btnElem.textContent = origText;
      btnElem.classList.remove("dispatched");
    }, 1200);
  }

  setFeedbackBanner(`COMMAND SENT: UAV ${id} -> ${action.toUpperCase()}`, `Dispatched to UAV ${id} | Alt: ${alt}m`, "exec");
  logEvent(`MAVLink Command SENT: UAV ${id} -> ${action.toUpperCase()} (Alt: ${alt}m)`);
};

window.changeDroneMode = function (id, mode) {
  sendGCSCommand({ action: "mode", target: id, params: { mode: mode } });
  setFeedbackBanner(`COMMAND SENT: UAV ${id} -> ${mode}`, `Mode changed to ${mode} for UAV ${id}`, "exec");
  logEvent(`MAVLink Mode Changed: UAV ${id} -> ${mode}`);
};

window.changeSwarmMode = function (mode) {
  sendGCSCommand({ action: "mode", target: "all", params: { mode: mode } });
  document.querySelectorAll(".mode-chip[data-mode]").forEach(b => {
    b.classList.toggle("active", b.dataset.mode === mode);
  });
  const modeValElem = document.getElementById("deck-current-mode");
  if (modeValElem) modeValElem.textContent = mode;
  setFeedbackBanner(`COMMAND SENT: SWARM MODE -> ${mode}`, `Swarm flight mode changed to ${mode}`, "exec");
  logEvent(`MAVLink Mode Changed: SWARM -> ${mode}`);
};

// ==========================================================================
// LiDAR 360 Polar Radar Scope & Proximity Sensor HUD
// ==========================================================================
function initLidarRadar() {
  radarCanvas = document.getElementById("lidar-polar-canvas");
  if (!radarCanvas) return;
  radarCtx = radarCanvas.getContext("2d");

  // Wire Clear Map button
  document.getElementById("btn-clear-pointcloud")?.addEventListener("click", (e) => {
    e.stopPropagation();
    clearPointCloudMap();
  });
}

function drawLidarRadar() {
  if (!radarCanvas || !radarCtx) return;
  const w = (radarCanvas.width = radarCanvas.parentElement.clientWidth || 166);
  const h = (radarCanvas.height = radarCanvas.parentElement.clientHeight || 95);
  const cx = w / 2;
  const cy = h / 2;
  const maxR = Math.min(cx, cy) - 6;

  radarCtx.clearRect(0, 0, w, h);

  // Concentric Tactical Range Rings (5m, 10m, 20m)
  const ranges = [0.25, 0.5, 1.0];
  radarCtx.strokeStyle = "rgba(0, 245, 212, 0.15)";
  radarCtx.lineWidth = 1;
  ranges.forEach(frac => {
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, maxR * frac, 0, Math.PI * 2);
    radarCtx.stroke();
  });

  // Azimuth Crosshairs
  radarCtx.strokeStyle = "rgba(255, 255, 255, 0.08)";
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy - maxR); radarCtx.lineTo(cx, cy + maxR);
  radarCtx.moveTo(cx - maxR, cy); radarCtx.lineTo(cx + maxR, cy);
  radarCtx.stroke();

  // Cardinal North Label
  radarCtx.fillStyle = "rgba(0, 245, 212, 0.45)";
  radarCtx.font = "bold 7px monospace";
  radarCtx.textAlign = "center";
  radarCtx.fillText("N", cx, cy - maxR + 8);

  // Rotating Radar Sweep Beam
  radarSweepAngle += 0.05;
  const sx = cx + Math.cos(radarSweepAngle) * maxR;
  const sy = cy + Math.sin(radarSweepAngle) * maxR;
  radarCtx.strokeStyle = "rgba(0, 245, 212, 0.4)";
  radarCtx.lineWidth = 1.5;
  radarCtx.beginPath();
  radarCtx.moveTo(cx, cy);
  radarCtx.lineTo(sx, sy);
  radarCtx.stroke();

  // Center Drone Blip
  radarCtx.fillStyle = "#00f5d4";
  radarCtx.beginPath();
  radarCtx.arc(cx, cy, 3, 0, Math.PI * 2);
  radarCtx.fill();

  // Selected Drone LiDAR Obstacle Hits
  const selId = state.selectedDroneId || 1;
  const selDrone = state.drones[selId];
  let minDist = 99.0;

  const targetUavElem = document.getElementById("hud-lidar-target-uav");
  if (targetUavElem) targetUavElem.textContent = `Target: UAV ${selId}`;

  if (selDrone && selDrone.telemetry && selDrone.telemetry.lidar_hits) {
    const hits = selDrone.telemetry.lidar_hits;
    hits.forEach(hit => {
      const [hx, hy, hz, dist, deg] = hit;
      if (dist < minDist) minDist = dist;

      const frac = Math.min(1.0, dist / 10.0);
      const r = frac * maxR;
      const angleRad = (deg - 90) * (Math.PI / 180);
      const bx = cx + Math.cos(angleRad) * r;
      const by = cy + Math.sin(angleRad) * r;

      let color = "#00f5d4";
      if (dist <= 3.0) color = "#ff4d4d";
      else if (dist < 6.0) color = "#ffb703";

      radarCtx.fillStyle = color;
      radarCtx.shadowColor = color;
      radarCtx.shadowBlur = 6;
      radarCtx.beginPath();
      radarCtx.arc(bx, by, 2.5, 0, Math.PI * 2);
      radarCtx.fill();
      radarCtx.shadowBlur = 0;
    });
  }

  // Update Closest Proximity Distance & HUD Alert Dot
  const distElem = document.getElementById("hud-lidar-min-dist");
  const dotElem = document.getElementById("radar-status-dot");
  if (distElem && dotElem) {
    if (minDist <= 3.0) {
      distElem.textContent = `${minDist.toFixed(1)}m ALERT`;
      distElem.className = "l-val highlight-red";
      dotElem.className = "status-dot red";
    } else if (minDist < 6.0) {
      distElem.textContent = `${minDist.toFixed(1)}m CAUTION`;
      distElem.className = "l-val highlight-amber";
      dotElem.className = "status-dot amber";
    } else if (minDist <= 10.0) {
      distElem.textContent = `${minDist.toFixed(1)}m CLEAR`;
      distElem.className = "l-val highlight-green";
      dotElem.className = "status-dot green";
    } else {
      distElem.textContent = "ALL CLEAR";
      distElem.className = "l-val highlight-green";
      dotElem.className = "status-dot green";
    }
  }
}

// ==========================================================================
// Bottom Golden Wave Telemetry Spline Chart
// ==========================================================================
function initWaveChart() {
  const canvas = document.getElementById("altitude-wave-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");

  function draw() {
    requestAnimationFrame(draw);
    const w = (canvas.width = canvas.parentElement.clientWidth);
    const h = (canvas.height = canvas.parentElement.clientHeight);

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 1;
    for (let y = 15; y < h; y += 20) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    const history = state.altitudeHistory;
    if (history.length < 2) return;

    const maxAlt = 25.0; // 25 meters scale ceiling
    const stepX = w / Math.max(1, history.length - 1);

    for (let dIdx = 0; dIdx < 5; dIdx++) {
      const color = DRONE_COLORS[dIdx];
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();

      const points = [];
      for (let i = 0; i < history.length; i++) {
        const alt = history[i].alts[dIdx] || 0;
        const x = i * stepX;
        const y = h - 10 - (alt / maxAlt) * (h - 20);
        points.push({ x, y });
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      if (points.length > 0) {
        const lastPt = points[points.length - 1];
        ctx.fillStyle = color;
        ctx.shadowColor = color;
        ctx.shadowBlur = 8;
        ctx.beginPath();
        ctx.arc(lastPt.x, lastPt.y, 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;

        ctx.strokeStyle = color;
        ctx.setLineDash([2, 3]);
        ctx.beginPath();
        ctx.moveTo(lastPt.x, lastPt.y);
        ctx.lineTo(lastPt.x, h);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }
  }
  draw();
}

// ==========================================================================
// Animation & 3D Render Loop (60 FPS Smooth Lerp)
// ==========================================================================
function animate() {
  requestAnimationFrame(animate);

  const delta = 0.09; // Lerp factor

  for (const [id, d] of Object.entries(state.drones)) {
    d.group.position.lerp(d.targetPos, delta);
    d.group.quaternion.slerp(new THREE.Quaternion().setFromEuler(d.targetRot), delta);

    // Rotate propellers ONLY when drone is armed or airborne
    const isArmed = d.telemetry && (d.telemetry.armed === true || d.group.position.y > 0.25);
    if (d.group.userData.props && isArmed) {
      d.group.userData.props.forEach((prop, i) => {
        prop.rotation.y += (i % 2 === 0 ? 0.45 : -0.45);
      });
    }

    if (d.scanRing) {
      d.scanRing.rotation.z += 0.05;
    }

    if (state.showDropLines && d.group.position.y > 0.1) {
      d.dropLine.visible = true;
      d.groundDisc.visible = true;
      const pos = d.group.position;
      const pts = [new THREE.Vector3(pos.x, pos.y, pos.z), new THREE.Vector3(pos.x, 0, pos.z)];
      d.dropLine.geometry.setFromPoints(pts);
      d.dropLine.computeLineDistances();
      d.groundDisc.position.set(pos.x, 0.02, pos.z);
    } else {
      d.dropLine.visible = false;
      d.groundDisc.visible = false;
    }
  }

  // Chase Mode: broadly follow selected drone in 3rd-person tactical perspective to see point clouds & surroundings
  if (state.cameraMode === "chase" && state.selectedDroneId && state.drones[state.selectedDroneId]) {
    const selDrone = state.drones[state.selectedDroneId];
    controls.target.lerp(selDrone.group.position, 0.1);
    const hdgRad = THREE.MathUtils.degToRad(selDrone.telemetry.heading || 0);
    const chaseDist = state.chaseDist || 38.0;
    const chaseHeight = state.chaseHeight || 18.0;
    const camOffsetX = -Math.sin(hdgRad) * chaseDist;
    const camOffsetZ = Math.cos(hdgRad) * chaseDist;
    const idealCam = new THREE.Vector3(
      selDrone.group.position.x + camOffsetX,
      selDrone.group.position.y + chaseHeight,
      selDrone.group.position.z + camOffsetZ
    );
    camera.position.lerp(idealCam, 0.08);
  }

  // Render 2D Polar LiDAR Scope
  drawLidarRadar();

  // Tick transient drone-return flashes: fade them out and remove expired ones
  const now = Date.now();
  for (let i = transientFlashes.length - 1; i >= 0; i--) {
    const flash = transientFlashes[i];
    const remaining = flash.expiresAt - now;
    if (remaining <= 0) {
      scene.remove(flash.mesh);
      flash.mesh.geometry.dispose();
      flash.mesh.material.dispose();
      transientFlashes.splice(i, 1);
    } else {
      // Fade opacity linearly over the last 800ms
      flash.mesh.material.opacity = Math.min(0.92, (remaining / FLASH_DURATION_MS) * 0.92);
    }
  }

  // Batch flush point cloud updates to GPU buffer
  flushPointCloudUpdates();

  controls.update();
  renderer.render(scene, camera);
}

// ==========================================================================
// UI Event Handlers & In-Place Command Feedback
// ==========================================================================
function initUIEventListeners() {
  setupDeckCommandButton("cmd-arm", "arm", "ARM ALL", "COMMAND SENT: ARM ALL -> 5 UAVs", "Arming motors commanded on all swarm drones");
  setupDeckCommandButton("cmd-disarm", "disarm", "DISARM", "COMMAND SENT: DISARM -> 5 UAVs", "Disarm commanded on all swarm drones");
  setupDeckCommandButton("cmd-takeoff", "takeoff", "TAKEOFF", "COMMAND SENT: TAKEOFF -> 5 UAVs", "NAV_TAKEOFF dispatched to target altitude");

  // Auto Survey opens configuration modal
  document.getElementById("cmd-auto")?.addEventListener("click", () => {
    openSurveyModal();
  });

  setupDeckCommandButton("cmd-rtl", "rtl", "SWARM RTL", "COMMAND SENT: SWARM RTL -> 5 UAVs", "Return to launch point commanded");
  setupDeckCommandButton("cmd-land", "land", "LAND ALL", "COMMAND SENT: LAND ALL -> 5 UAVs", "Vertical landing initiated");

  // Swarm Flight Mode Selector Chips
  document.querySelectorAll(".mode-chip[data-mode]").forEach(btn => {
    btn.addEventListener("click", () => {
      const mode = btn.dataset.mode;
      window.changeSwarmMode(mode);
    });
  });

  // Altitude Slider & Preset Chips
  const altSlider = document.getElementById("slider-alt");
  const altValLabel = document.getElementById("slider-alt-val");
  altSlider?.addEventListener("input", (e) => {
    altValLabel.textContent = `${parseFloat(e.target.value).toFixed(1)} m`;
  });

  document.getElementById("btn-apply-alt")?.addEventListener("click", function () {
    const alt = parseFloat(altSlider.value);
    sendGCSCommand({ action: "alt", target: "all", params: { alt: alt } });

    this.textContent = "✓ APPLIED";
    this.classList.add("dispatched");
    setTimeout(() => {
      this.textContent = "APPLY ALT";
      this.classList.remove("dispatched");
    }, 1200);

    setFeedbackBanner(`COMMAND SENT: CLIMB TO ${alt.toFixed(1)}m`, `Altitude vector dispatched to 5 UAVs`, "exec");
    logEvent(`MAVLink Command SENT: ALTITUDE -> ${alt}m to Swarm`);
  });

  document.querySelectorAll(".alt-chip[data-alt]").forEach(btn => {
    btn.addEventListener("click", (e) => {
      document.querySelectorAll(".alt-chip[data-alt]").forEach(b => b.classList.remove("active"));
      btn.classList.add("active");
      const alt = parseFloat(btn.dataset.alt);
      altSlider.value = alt;
      altValLabel.textContent = `${alt.toFixed(1)} m`;
      sendGCSCommand({ action: "alt", target: "all", params: { alt: alt } });
      setFeedbackBanner(`COMMAND SENT: ALTITUDE PRESET ${alt}m`, `Swarm altitude target updated`, "exec");
      logEvent(`MAVLink Command SENT: ALTITUDE PRESET -> ${alt}m to Swarm`);
    });
  });

  // Choreography Formation Buttons in Left Panel
  document.querySelectorAll(".formation-btn").forEach(btn => {
    btn.addEventListener("click", () => {
      const type = btn.dataset.formation;
      openFormationModal(type);
    });
  });

  // Heading Preset Chips
  document.querySelectorAll(".heading-presets-row .alt-chip").forEach(btn => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".heading-presets-row .alt-chip").forEach(b => b.classList.remove("active"));
      btn.classList.add("active");
      const yaw = parseFloat(btn.dataset.yaw);
      document.getElementById("heading-val").textContent = `${yaw}°`;
      sendGCSCommand({ action: "heading", target: "all", params: { yaw: yaw } });
      setFeedbackBanner(`COMMAND SENT: HEADING ${yaw}°`, `Swarm yaw alignment commanded`, "exec");
      logEvent(`MAVLink Command SENT: HEADING LOCK -> ${yaw}°`);
    });
  });

  // Right Inspector Collapse Toggle Button
  document.getElementById("btn-toggle-inspector")?.addEventListener("click", function () {
    const inspector = document.getElementById("telemetry-inspector");
    if (inspector) {
      inspector.classList.toggle("collapsed");
      this.textContent = inspector.classList.contains("collapsed") ? "▼" : "▲";
    }
  });

  // Inspector Sub-Tabs Switching (PROPERTIES, SETTINGS, PERFORMANCE, LOGS)
  document.querySelectorAll(".insp-tab").forEach(tab => {
    tab.addEventListener("click", function () {
      const tabKey = this.dataset.tab || this.textContent.trim().toLowerCase();
      document.querySelectorAll(".insp-tab").forEach(t => t.classList.remove("active"));
      this.classList.add("active");

      document.querySelectorAll(".tab-pane").forEach(pane => {
        pane.classList.add("hidden");
        pane.classList.remove("active");
      });

      const activePane = document.getElementById(`tab-pane-${tabKey}`);
      if (activePane) {
        activePane.classList.remove("hidden");
        activePane.classList.add("active");
      }
    });
  });

  // Settings Tab: Full System Restart Action
  document.getElementById("btn-full-restart")?.addEventListener("click", () => {
    const droneCount = parseInt(document.getElementById("select-restart-drones")?.value || "1");
    if (!confirm(`⚠️ RESTART SIMULATION & SITL?\n\nThis will cleanly terminate ArduPilot & AirSim, purge socket buffers, and relaunch with ${droneCount} drone(s).`)) {
      return;
    }

    sendGCSCommand({
      action: "restart_system",
      target: "all",
      params: { num_drones: droneCount }
    });

    setFeedbackBanner(
      "🔄 RESTARTING SIMULATION...",
      `Terminating stale processes & relaunching SITL with ${droneCount} UAV(s)...`,
      "warn"
    );
    logEvent(`⚡ FULL SYSTEM RESTART DISPATCHED for ${droneCount} UAVs`);

    // Show reloading indicator
    setTimeout(() => {
      window.location.reload();
    }, 4500);
  });

  // Settings Tab: Fault Injection & Resilience Testing Handlers
  document.getElementById("btn-fault-kill-drone")?.addEventListener("click", () => {
    const targetId = parseInt(document.getElementById("select-fault-drone")?.value || "1");
    killDrone(targetId);
  });

  document.getElementById("btn-fault-link-loss")?.addEventListener("click", () => {
    const targetId = parseInt(document.getElementById("select-fault-drone")?.value || "1");
    sendGCSCommand({ action: "fault_inject", target: targetId, params: { type: "disconnect", drone_id: targetId } });
    logEvent(`📡 FAULT INJECTED: UAV ${targetId} RF link loss simulated.`);
    setFeedbackBanner(`UAV ${targetId} LINK LOST`, "RF Disconnect simulated. Swarm reconfiguring.", "warn");
  });

  document.getElementById("btn-fault-revive-all")?.addEventListener("click", () => {
    reviveAllDrones();
  });

  // --- Settings Tab: Live Persistent Sliders & Handlers ---
  const setFenceRad = document.getElementById("input-set-fence-radius");
  const valFenceRad = document.getElementById("val-set-fence-radius");
  const setFenceAlt = document.getElementById("input-set-fence-alt");
  const valFenceAlt = document.getElementById("val-set-fence-alt");
  const setAvoidMargin = document.getElementById("input-set-avoid-margin");
  const valAvoidMargin = document.getElementById("val-set-avoid-margin");
  const setSwarmRepel = document.getElementById("input-set-swarm-repel");
  const valSwarmRepel = document.getElementById("val-set-swarm-repel");
  const setSolidifyRate = document.getElementById("input-set-solidify-rate");
  const valSolidifyRate = document.getElementById("val-set-solidify-rate");
  const selectSpawnDrones = document.getElementById("select-spawn-drones");
  const badgeSync = document.getElementById("badge-params-sync");

  const sliderInputs = [setFenceRad, setFenceAlt, setAvoidMargin, setSwarmRepel, setSolidifyRate];
  sliderInputs.forEach(sl => {
    if (!sl) return;
    sl.addEventListener("pointerdown", () => { isUserDraggingSlider = true; });
    sl.addEventListener("pointerup", () => { isUserDraggingSlider = false; });
    sl.addEventListener("touchstart", () => { isUserDraggingSlider = true; });
    sl.addEventListener("touchend", () => { isUserDraggingSlider = false; });
  });

  const pushCurrentSettings = (showBanner = false) => {
    const rad = parseFloat(setFenceRad?.value || 85);
    const alt = parseFloat(setFenceAlt?.value || 25);
    const avoid = parseFloat(setAvoidMargin?.value || 4.0);
    const repel = parseFloat(setSwarmRepel?.value || 16.0);
    const minD = Math.max(2.5, +(repel / 5).toFixed(1));
    const solidify = parseFloat(setSolidifyRate?.value || 1.5);
    const numDrones = parseInt(selectSpawnDrones?.value || 5);

    const payload = {
      geofence_radius: rad,
      geofence_alt_max: alt,
      avoid_margin: avoid,
      swarm_repel: repel,
      min_drone_dist: minD,
      solidify_rate: solidify,
      spawn_num_drones: numDrones
    };

    saveSettingsLocally(payload);
    sendGCSCommand({ action: "update_settings", params: payload });

    if (state.swarm && state.swarm.geofence) {
      state.swarm.geofence.radius = rad;
      state.swarm.geofence.alt_max = alt;
    }
    updateGeofenceVisual({ enabled: true, radius: rad, alt_max: alt }, false);
    setSolidificationInterval(solidify);

    if (badgeSync) {
      badgeSync.textContent = "SAVED & SYNCED";
      badgeSync.className = "settings-badge green";
      setTimeout(() => {
        if (badgeSync) badgeSync.textContent = "SYNCED";
      }, 1500);
    }

    if (showBanner) {
      setFeedbackBanner("PARAMETERS SAVED", "Local storage & live SITL MAVLink limits updated.", "ok");
      logEvent("Settings: Parameters saved locally and synchronized with simulation.");
    }
  };

  setFenceRad?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    if (valFenceRad) valFenceRad.textContent = `${val.toFixed(1)} m`;
    pushCurrentSettings();
  });

  setFenceAlt?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    if (valFenceAlt) valFenceAlt.textContent = `${val.toFixed(1)} m`;
    pushCurrentSettings();
  });

  setAvoidMargin?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    if (valAvoidMargin) valAvoidMargin.textContent = `${val.toFixed(1)} m`;
    pushCurrentSettings();
  });

  setSwarmRepel?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    const minD = Math.max(2.5, +(val / 5).toFixed(1));
    if (valSwarmRepel) valSwarmRepel.textContent = `${minD.toFixed(1)} m (${val.toFixed(0)}m Repel)`;
    pushCurrentSettings();
  });

  setSolidifyRate?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    if (valSolidifyRate) valSolidifyRate.textContent = `${val.toFixed(1)} s`;
    pushCurrentSettings();
  });

  selectSpawnDrones?.addEventListener("change", () => {
    pushCurrentSettings();
  });

  document.getElementById("btn-save-settings")?.addEventListener("click", () => {
    pushCurrentSettings(true);
  });

  document.getElementById("btn-reset-settings")?.addEventListener("click", () => {
    if (setFenceRad) { setFenceRad.value = 85; if (valFenceRad) valFenceRad.textContent = "85.0 m"; }
    if (setFenceAlt) { setFenceAlt.value = 25; if (valFenceAlt) valFenceAlt.textContent = "25.0 m"; }
    if (setAvoidMargin) { setAvoidMargin.value = 4.0; if (valAvoidMargin) valAvoidMargin.textContent = "4.0 m"; }
    if (setSwarmRepel) { setSwarmRepel.value = 16.0; if (valSwarmRepel) valSwarmRepel.textContent = "3.0 m (16m Repel)"; }
    if (setSolidifyRate) { setSolidifyRate.value = 1.5; if (valSolidifyRate) valSolidifyRate.textContent = "1.5 s"; }
    pushCurrentSettings(true);
    setFeedbackBanner("SETTINGS RESET", "Restored parameters to factory defaults.", "exec");
    logEvent("Settings: Reset all parameters to factory defaults.");
  });

  // Settings Tab: Clear Point Cloud Button
  document.getElementById("btn-clear-pointcloud-settings")?.addEventListener("click", () => {
    clearPointCloudMap();
    logEvent("SLAM: Point Cloud & Solidified 3D Map cleared from Settings.");
    setFeedbackBanner("POINT CLOUD CLEARED", "Raw points and 3D solid blocks wiped.", "exec");
  });

  // Left Tool Strip Buttons
  document.getElementById("tool-orbit")?.addEventListener("click", () => setCameraMode("orbit"));
  document.getElementById("tool-top")?.addEventListener("click", () => setCameraMode("top"));
  document.getElementById("tool-chase")?.addEventListener("click", () => setCameraMode("chase"));

  document.getElementById("tool-tails")?.addEventListener("click", function () {
    state.showTrails = !state.showTrails;
    this.classList.toggle("active", state.showTrails);
  });

  document.getElementById("tool-droplines")?.addEventListener("click", function () {
    state.showDropLines = !state.showDropLines;
    this.classList.toggle("active", state.showDropLines);
  });

  document.getElementById("tool-pointcloud")?.addEventListener("click", function () {
    state.showPointCloud = !state.showPointCloud;
    this.classList.toggle("active", state.showPointCloud);
    if (pointCloudMesh) pointCloudMesh.visible = state.showPointCloud;
    logEvent(`Point Cloud SLAM Buffer Display ${state.showPointCloud ? 'Enabled' : 'Disabled'}`);
  });

  document.getElementById("tool-recenter")?.addEventListener("click", () => {
    state.followSelected = false;
    setCameraMode("orbit");
    camera.position.set(0, 26, 42);
    controls.target.set(0, 2, 0);
    logEvent("3D Camera Recentered to Swarm Origin");
  });

  document.getElementById("btn-fullscreen")?.addEventListener("click", () => {
    if (!document.fullscreenElement) document.documentElement.requestFullscreen().catch(() => { });
    else document.exitFullscreen();
  });
}

function setupDeckCommandButton(btnId, action, label, bannerTitle, bannerSub) {
  const btn = document.getElementById(btnId);
  if (!btn) return;
  const labelElem = btn.querySelector(".deck-btn-label");

  btn.addEventListener("click", () => {
    btn.classList.add("dispatched");
    if (labelElem) labelElem.textContent = "✓ SENT";

    const alt = parseFloat(document.getElementById("slider-alt")?.value || 5.0);
    sendGCSCommand({ action: action, target: "all", params: { alt: alt } });

    setFeedbackBanner(bannerTitle, `${bannerSub} (Alt: ${alt}m)`, "exec");
    logEvent(`MAVLink Command SENT: ${action.toUpperCase()} -> 5 UAVs`);

    setTimeout(() => {
      btn.classList.remove("dispatched");
      if (labelElem) labelElem.textContent = label;
    }, 1400);
  });
}

function setCameraMode(mode) {
  state.cameraMode = mode;
  document.querySelectorAll(".side-tool-strip .tool-btn").forEach(b => b.classList.remove("active"));

  if (mode === "orbit") {
    document.getElementById("tool-orbit")?.classList.add("active");
    state.followSelected = false;

    // Zoomed out 3D overview of the entire swarm and arena
    const activeDrones = Object.values(state.drones);
    let avgX = 0, avgY = 2, avgZ = 0;
    if (activeDrones.length > 0) {
      avgX = activeDrones.reduce((sum, d) => sum + d.group.position.x, 0) / activeDrones.length;
      avgY = Math.max(2, activeDrones.reduce((sum, d) => sum + d.group.position.y, 0) / activeDrones.length);
      avgZ = activeDrones.reduce((sum, d) => sum + d.group.position.z, 0) / activeDrones.length;
    }
    controls.target.set(avgX, avgY, avgZ);
    camera.position.set(avgX, avgY + 34, avgZ + 58);
    logEvent("Camera switched to 3D Orbit (Global Swarm Overview)");
  } else if (mode === "top") {
    document.getElementById("tool-top")?.classList.add("active");
    state.followSelected = false;
    camera.position.set(0, 85, 0.01);
    controls.target.set(0, 0, 0);
    logEvent("Camera switched to Top-Down Tactical Ortho");
  } else if (mode === "chase") {
    document.getElementById("tool-chase")?.classList.add("active");
    state.followSelected = true;
    const selDrone = state.drones[state.selectedDroneId];
    if (selDrone) {
      const dPos = selDrone.group.position;
      controls.target.copy(dPos);
      const hdgRad = THREE.MathUtils.degToRad(selDrone.telemetry.heading || 0);
      const chaseDist = state.chaseDist || 38.0;
      const chaseHeight = state.chaseHeight || 18.0;
      camera.position.set(
        dPos.x - Math.sin(hdgRad) * chaseDist,
        dPos.y + chaseHeight,
        dPos.z + Math.cos(hdgRad) * chaseDist
      );
      logEvent(`Camera switched to Chase Tactical View (UAV ${state.selectedDroneId})`);
    }
  }
}

function setFeedbackBanner(title, subtitle, statusClass = "ok") {
  document.getElementById("feedback-title").textContent = title;
  document.getElementById("feedback-subtitle").textContent = subtitle;
  const badge = document.getElementById("feedback-status");
  if (badge) {
    badge.textContent = statusClass === "exec" ? "SENT ✓" : "STANDBY";
    badge.className = `feedback-badge ${statusClass}`;
  }
  const banner = document.getElementById("feedback-banner");
  if (banner) {
    banner.className = `command-feedback-banner ${statusClass === 'exec' ? 'executing' : ''}`;
  }
}

// ==========================================================================
// Swarm Choreography Parameter Configuration Modal & 2D Preview
// ==========================================================================
function initChoreographyModal() {
  const modal = document.getElementById("formation-modal");
  const btnClose = document.getElementById("modal-btn-close");
  const btnCancel = document.getElementById("modal-btn-cancel");
  const btnExecute = document.getElementById("modal-btn-execute");

  const inputSpacing = document.getElementById("input-modal-spacing");
  const valSpacing = document.getElementById("val-modal-spacing");
  const inputAlt = document.getElementById("input-modal-alt");
  const valAlt = document.getElementById("val-modal-alt");
  const inputHdg = document.getElementById("input-modal-heading");
  const valHdg = document.getElementById("val-modal-heading");

  // Geometry Type Buttons
  document.querySelectorAll(".modal-type-btn").forEach(btn => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".modal-type-btn").forEach(b => b.classList.remove("active"));
      btn.classList.add("active");
      state.formationConfig.type = btn.dataset.type;

      const labelSpacing = document.getElementById("label-spacing");
      if (btn.dataset.type === "circle") {
        labelSpacing.textContent = "Circle Radius";
      } else {
        labelSpacing.textContent = "Inter-Drone Spacing";
      }
      renderFormationPreview();
    });
  });

  // Live Sliders
  inputSpacing?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    state.formationConfig.spacing = val;
    state.formationConfig.radius = val * 2;
    valSpacing.textContent = `${val.toFixed(1)} m`;
    renderFormationPreview();
  });

  inputAlt?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    state.formationConfig.alt = val;
    valAlt.textContent = `${val.toFixed(1)} m`;
    renderFormationPreview();
  });

  inputHdg?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    state.formationConfig.heading = val;
    valHdg.textContent = `${val.toFixed(0)}°`;
    renderFormationPreview();
  });

  btnClose?.addEventListener("click", () => modal.classList.add("hidden"));
  btnCancel?.addEventListener("click", () => modal.classList.add("hidden"));

  // Execute Choreography
  btnExecute?.addEventListener("click", () => {
    modal.classList.add("hidden");
    const cfg = state.formationConfig;

    sendGCSCommand({
      action: "formation",
      target: "all",
      params: {
        type: cfg.type,
        spacing: cfg.spacing,
        radius: cfg.radius || (cfg.spacing * 2),
        alt: cfg.alt,
        heading: cfg.heading
      }
    });

    setFeedbackBanner(`COMMAND SENT: FORMATION ${cfg.type.toUpperCase()}`, `Dispatched ${cfg.type} (Spacing: ${cfg.spacing}m, Alt: ${cfg.alt}m) to 5 UAVs`, "exec");
    logEvent(`MAVLink Command SENT: FORMATION ${cfg.type.toUpperCase()} (Spacing: ${cfg.spacing}m, Alt: ${cfg.alt}m)`);
  });
}

function openFormationModal(type = "line") {
  const modal = document.getElementById("formation-modal");
  if (!modal) return;
  state.formationConfig.type = type;

  document.querySelectorAll(".modal-type-btn").forEach(b => {
    b.classList.toggle("active", b.dataset.type === type);
  });

  const curAlt = parseFloat(document.getElementById("slider-alt")?.value || 5.0);
  state.formationConfig.alt = curAlt;
  const inputAlt = document.getElementById("input-modal-alt");
  if (inputAlt) inputAlt.value = curAlt;
  const valAlt = document.getElementById("val-modal-alt");
  if (valAlt) valAlt.textContent = `${curAlt.toFixed(1)} m`;

  modal.classList.remove("hidden");
  renderFormationPreview();
}

function renderFormationPreview() {
  const canvas = document.getElementById("formation-preview-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;

  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx, 10); ctx.lineTo(cx, h - 10);
  ctx.moveTo(10, cy); ctx.lineTo(w - 10, cy);
  ctx.stroke();

  ctx.strokeStyle = "rgba(0, 230, 118, 0.15)";
  ctx.beginPath();
  ctx.arc(cx, cy, 35, 0, Math.PI * 2);
  ctx.arc(cx, cy, 70, 0, Math.PI * 2);
  ctx.stroke();

  const cfg = state.formationConfig;
  const N = 5;
  const scale = 5.5;
  const theta = (cfg.heading * Math.PI) / 180.0;
  const slots = [];

  for (let k = 0; k < N; k++) {
    let dx = 0, dy = 0;
    if (cfg.type === "line") {
      const s = (k - (N - 1) / 2.0) * cfg.spacing;
      dx = s * Math.cos(theta);
      dy = s * Math.sin(theta);
    } else if (cfg.type === "v_shape") {
      if (k > 0) {
        const side = (k % 2 === 1) ? 1.0 : -1.0;
        const tier = Math.floor((k + 1) / 2);
        const beta = (35.0 * Math.PI) / 180.0;
        const x_loc = side * tier * cfg.spacing * Math.sin(beta);
        const y_loc = -tier * cfg.spacing * Math.cos(beta);
        dx = x_loc * Math.cos(theta) - y_loc * Math.sin(theta);
        dy = x_loc * Math.sin(theta) + y_loc * Math.cos(theta);
      }
    } else if (cfg.type === "grid") {
      const cols = 3;
      const row = Math.floor(k / cols);
      const col = k % cols;
      const x_loc = (col - 1.0) * cfg.spacing;
      const y_loc = (row - 0.5) * cfg.spacing;
      dx = x_loc * Math.cos(theta) - y_loc * Math.sin(theta);
      dy = x_loc * Math.sin(theta) + y_loc * Math.cos(theta);
    } else if (cfg.type === "circle" || cfg.type === "helix") {
      const phi = theta + (2.0 * Math.PI * k / N);
      const rad = cfg.radius || (cfg.spacing * 2);
      dx = (rad * 0.7) * Math.cos(phi);
      dy = (rad * 0.7) * Math.sin(phi);
    }
    slots.push({ k: k + 1, x: cx + dx * scale, y: cy - dy * scale, color: DRONE_COLORS[k] });
  }

  ctx.strokeStyle = "rgba(0, 230, 118, 0.4)";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  slots.forEach((pt, i) => {
    if (i === 0) ctx.moveTo(pt.x, pt.y);
    else ctx.lineTo(pt.x, pt.y);
  });
  if (cfg.type === "circle" || cfg.type === "grid") ctx.closePath();
  ctx.stroke();

  slots.forEach(pt => {
    ctx.fillStyle = pt.color;
    ctx.shadowColor = pt.color;
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;

    ctx.fillStyle = "#000";
    ctx.font = "bold 9px monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(pt.k, pt.x, pt.y);
  });
}

// ==========================================================================
// Autonomous Swarm Survey Parameter Configuration Modal & 2D Preview
// ==========================================================================
function initSurveyModal() {
  const modal = document.getElementById("survey-modal");
  const btnClose = document.getElementById("survey-btn-close");
  const btnCancel = document.getElementById("survey-btn-cancel");
  const btnExecute = document.getElementById("survey-btn-execute");

  const inputHeight = document.getElementById("input-survey-height");
  const valHeight = document.getElementById("val-survey-height");
  const inputHeading = document.getElementById("input-survey-heading");
  const valHeading = document.getElementById("val-survey-heading");

  function setHeading(deg) {
    state.surveyConfig.heading = deg;
    if (inputHeading) inputHeading.value = deg;
    const card = deg === 0 ? " (NORTH)" : (deg === 90 ? " (EAST)" : (deg === 180 ? " (SOUTH)" : (deg === 270 ? " (WEST)" : "")));
    if (valHeading) valHeading.textContent = `${deg.toFixed(0)}°${card}`;
    openSurveyModal(true); // refresh drone sector rows
    renderSurveyPreview();
  }

  document.getElementById("btn-dir-north")?.addEventListener("click", () => setHeading(0));
  document.getElementById("btn-dir-east")?.addEventListener("click", () => setHeading(90));
  document.getElementById("btn-dir-south")?.addEventListener("click", () => setHeading(180));
  document.getElementById("btn-dir-west")?.addEventListener("click", () => setHeading(270));

  inputHeight?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    state.surveyConfig.height = val;
    valHeight.textContent = `${val.toFixed(1)} m`;
    renderSurveyPreview();
  });

  inputHeading?.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    setHeading(val);
  });

  document.getElementById("btn-stagger-alts")?.addEventListener("click", () => {
    const activeIds = Object.keys(state.drones).map(Number).sort((a, b) => a - b);
    const ids = activeIds.length > 0 ? activeIds : [1, 2, 3, 4, 5];
    const baseH = state.surveyConfig.height || 5.0;
    ids.forEach((id, idx) => {
      const step = (idx === 3 ? 1 : (idx === 4 ? 0 : idx)) * 2.0;
      const alt = Math.max(2.0, baseH + step);
      state.surveyConfig.altitudes[id] = alt;
      const inElem = document.getElementById(`survey-alt-in-${id}`);
      if (inElem) inElem.value = alt.toFixed(1);
      const sliderElem = document.getElementById(`survey-alt-slider-${id}`);
      if (sliderElem) sliderElem.value = alt;
    });
    renderSurveyPreview();
  });

  document.getElementById("btn-uniform-alts")?.addEventListener("click", () => {
    const activeIds = Object.keys(state.drones).map(Number).sort((a, b) => a - b);
    const ids = activeIds.length > 0 ? activeIds : [1, 2, 3, 4, 5];
    const baseH = state.surveyConfig.height || 5.0;
    ids.forEach((id) => {
      state.surveyConfig.altitudes[id] = baseH;
      const inElem = document.getElementById(`survey-alt-in-${id}`);
      if (inElem) inElem.value = baseH.toFixed(1);
      const sliderElem = document.getElementById(`survey-alt-slider-${id}`);
      if (sliderElem) sliderElem.value = baseH;
    });
    renderSurveyPreview();
  });

  btnClose?.addEventListener("click", () => modal.classList.add("hidden"));
  btnCancel?.addEventListener("click", () => modal.classList.add("hidden"));

  // Primary: Launch Autonomous Frontier Exploration
  btnExecute?.addEventListener("click", () => {
    modal.classList.add("hidden");
    const cfg = state.surveyConfig;

    sendGCSCommand({
      action: "survey",
      target: "all",
      params: {
        length: 60.0,
        width: 60.0,
        height: cfg.height || 5.0,
        heading: cfg.heading || 0.0,
        altitudes: cfg.altitudes,
        guided: true,
        auto_start: true
      }
    });

    // Check if any drone is still initializing EKF / calibrating sensors
    const dronesList = Object.values(state.drones);
    const isCalibrating = dronesList.some(d => {
      const t = d.telemetry || {};
      const statusMsg = (t.last_status_msg || "").toUpperCase();
      const phase = (t.flight_phase || "").toUpperCase();
      return (!t.armed || (t.alt || 0) < 0.8) && (statusMsg.includes("EKF") || statusMsg.includes("CALIBRAT") || phase.includes("INIT") || phase.includes("EKF"));
    });

    const activeCount = Object.keys(state.drones).length || 1;
    if (isCalibrating) {
      showMissionOverlay({
        type: "warn",
        badge: "⏳ EKF CALIBRATION IN PROGRESS",
        title: "Sensors & GPS Calibrating",
        message: "Swarm is initializing EKF attitude & position lock. Autonomous survey is queued and will automatically take off as soon as all UAVs are ready!",
        timeout: 6000
      });
      setFeedbackBanner(
        "⏳ AUTONOMOUS SURVEY QUEUED",
        "Sensors calibrating. Swarm will automatically arm & take off once EKF is ready...",
        "warn"
      );
    } else {
      setFeedbackBanner(
        "AUTONOMOUS EXPLORATION LAUNCHED",
        `Swarm scanning initiated: ${cfg.heading}° azimuth, ${cfg.height || 5.0}m alt across ${activeCount} UAVs`,
        "exec"
      );
    }
    logEvent(`Autonomous Frontier Exploration Dispatched: Heading ${cfg.heading}° across ${activeCount} UAVs`);
  });
}

window.updateDroneSurveyAlt = function (id, val) {
  const numVal = parseFloat(val);
  if (isNaN(numVal) || numVal < 1.0) return;
  state.surveyConfig.altitudes[id] = numVal;
  const inElem = document.getElementById(`survey-alt-in-${id}`);
  if (inElem && parseFloat(inElem.value) !== numVal) inElem.value = numVal.toFixed(1);
  const sliderElem = document.getElementById(`survey-alt-slider-${id}`);
  if (sliderElem && parseFloat(sliderElem.value) !== numVal) sliderElem.value = numVal;
  renderSurveyPreview();
};

window.openSurveyModal = function (skipClassRemove = false) {
  const modal = document.getElementById("survey-modal");
  if (!modal) return;

  const activeIds = Object.keys(state.drones).map(Number).sort((a, b) => a - b);
  const idsToRender = activeIds.length > 0 ? activeIds : [1];
  const totalDrones = idsToRender.length;
  const droneListContainer = document.getElementById("survey-drones-list");
  const baseHeading = state.surveyConfig.heading || 0.0;

  if (droneListContainer) {
    droneListContainer.innerHTML = "";

    idsToRender.forEach((id, idx) => {
      if (state.surveyConfig.altitudes[id] === undefined) {
        state.surveyConfig.altitudes[id] = state.surveyConfig.height || 5.0;
      }
      const alt = state.surveyConfig.altitudes[id];
      const color = DRONE_COLORS[(id - 1) % DRONE_COLORS.length];
      const sectorAng = (baseHeading + (idx * 360.0 / totalDrones)) % 360.0;

      const row = document.createElement("div");
      row.className = "survey-drone-row";
      row.style.setProperty("--drone-color", color);
      row.innerHTML = `
        <div class="s-drone-id-col">
          <div class="s-drone-dot"></div>
          <span class="s-drone-name">UAV ${id}</span>
        </div>
        <div class="s-drone-lane-info" style="color: ${color}; font-weight:600;">Sector ${sectorAng.toFixed(0)}° (${idx + 1}/${totalDrones})</div>
        <div class="s-drone-alt-controls">
          <input type="range" id="survey-alt-slider-${id}" class="s-drone-alt-slider" min="2" max="25" step="0.5" value="${alt}" oninput="updateDroneSurveyAlt(${id}, this.value)">
          <input type="number" id="survey-alt-in-${id}" class="survey-alt-input" min="2" max="30" step="0.5" value="${alt.toFixed(1)}" onchange="updateDroneSurveyAlt(${id}, this.value)">
          <span style="font-size:10px;color:var(--text-muted);font-family:var(--font-mono)">m</span>
        </div>
      `;
      droneListContainer.appendChild(row);
    });
  }

  if (!skipClassRemove) {
    modal.classList.remove("hidden");
  }
  renderSurveyPreview();
};

function renderSurveyPreview() {
  const canvas = document.getElementById("survey-preview-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const R = (w / 2) - 22;

  ctx.clearRect(0, 0, w, h);

  // High-Tech Radar Background Grid
  ctx.strokeStyle = "rgba(0, 245, 212, 0.08)";
  ctx.lineWidth = 1;
  [0.25, 0.5, 0.75, 1.0].forEach((ratio) => {
    ctx.beginPath();
    ctx.arc(cx, cy, R * ratio, 0, Math.PI * 2);
    ctx.stroke();
  });

  // Crosshairs
  ctx.strokeStyle = "rgba(0, 245, 212, 0.12)";
  ctx.beginPath();
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
  ctx.stroke();

  // Cardinal Direction Labels
  ctx.font = "bold 9px monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "#00e676";
  ctx.fillText("N (0°)", cx, cy - R - 10);
  ctx.fillStyle = "rgba(255,255,255,0.4)";
  ctx.fillText("E (90°)", cx + R + 10, cy);
  ctx.fillText("S (180°)", cx, cy + R + 10);
  ctx.fillText("W (270°)", cx - R - 10, cy);

  const cfg = state.surveyConfig;
  const baseHeading = cfg.heading || 0.0;
  const activeIds = Object.keys(state.drones).map(Number).sort((a, b) => a - b);
  const ids = activeIds.length > 0 ? activeIds : [1];
  const N = ids.length;
  const altsList = [];

  // Draw 360° Distributed Sector Exploration Cones for each drone
  ids.forEach((id, idx) => {
    const color = DRONE_COLORS[(id - 1) % DRONE_COLORS.length];
    const alt = cfg.altitudes[id] || cfg.height || 5.0;
    altsList.push(alt);

    const sectorAngDeg = (baseHeading + (idx * 360.0 / N)) % 360.0;
    // In Canvas: 0° is North (-Y), 90° East (+X) -> angle from +X = (sectorAngDeg - 90)
    const rad = (sectorAngDeg - 90) * Math.PI / 180.0;
    const halfSpread = (360.0 / N) * 0.45 * Math.PI / 180.0;

    // Sector Exploration Beam Fill
    ctx.fillStyle = color.replace(')', ', 0.12)').replace('rgb', 'rgba').replace('#', 'rgba(');
    // fallback if hex color
    ctx.save();
    ctx.fillStyle = `${color}22`;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R * 0.95, rad - halfSpread, rad + halfSpread);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // Direction Vector Line
    const tipX = cx + Math.cos(rad) * (R * 0.92);
    const tipY = cy + Math.sin(rad) * (R * 0.92);

    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // Arrowhead at Tip
    const arrowSz = 6;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(tipX, tipY, 4, 0, Math.PI * 2);
    ctx.fill();

    // Label Drone ID & Alt
    const lblDist = R * 0.65;
    const lblX = cx + Math.cos(rad) * lblDist;
    const lblY = cy + Math.sin(rad) * lblDist;

    ctx.fillStyle = "rgba(10, 15, 22, 0.88)";
    ctx.fillRect(lblX - 18, lblY - 8, 36, 16);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.strokeRect(lblX - 18, lblY - 8, 36, 16);

    ctx.fillStyle = color;
    ctx.font = "bold 8px monospace";
    ctx.fillText(`UAV${id} ${alt.toFixed(0)}m`, lblX, lblY);
  });

  // Center Origin Pulse Marker
  ctx.fillStyle = "#00f5d4";
  ctx.shadowColor = "#00f5d4";
  ctx.shadowBlur = 6;
  ctx.beginPath();
  ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;

  // Update Metric chips
  const metricSwarmN = document.getElementById("metric-swarm-n");
  if (metricSwarmN) metricSwarmN.textContent = `${N} UAV${N > 1 ? 's' : ''}`;
}

// ==========================================================================
// Mouse Raycaster & Tooltips
// ==========================================================================
function onMouseMove(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(mouse, camera);
  const droneGroups = Object.values(state.drones).map(d => d.group);
  const intersects = raycaster.intersectObjects(droneGroups, true);

  const tooltip = document.getElementById("drone-3d-tooltip");
  if (intersects.length > 0) {
    let rootObj = intersects[0].object;
    while (rootObj.parent && !rootObj.name.startsWith("drone_")) {
      rootObj = rootObj.parent;
    }
    const droneId = rootObj.userData?.id;
    if (droneId && state.drones[droneId]) {
      const telem = state.drones[droneId].telemetry;
      document.getElementById("tt-title").textContent = `UAV ${droneId}`;
      document.getElementById("tt-phase").textContent = telem.flight_phase || telem.mode || "GUIDED";
      document.getElementById("tt-alt").textContent = `${(telem.alt || 0).toFixed(1)}m`;
      document.getElementById("tt-speed").textContent = `${(telem.speed || 0).toFixed(1)}m/s`;
      document.getElementById("tt-batt").textContent = `${telem.battery || 100}%`;
      document.getElementById("tt-msg").textContent = telem.last_status_msg || "Normal Flight Operations";

      tooltip.style.left = `${event.clientX + 14}px`;
      tooltip.style.top = `${event.clientY - 20}px`;
      tooltip.classList.remove("hidden");
      return;
    }
  }
  tooltip.classList.add("hidden");
}

function onMouseClick(event) {
  raycaster.setFromCamera(mouse, camera);
  const droneGroups = Object.values(state.drones).map(d => d.group);
  const intersects = raycaster.intersectObjects(droneGroups, true);
  if (intersects.length > 0) {
    let rootObj = intersects[0].object;
    while (rootObj.parent && !rootObj.name.startsWith("drone_")) {
      rootObj = rootObj.parent;
    }
    const droneId = rootObj.userData?.id;
    if (droneId) {
      selectDrone(droneId);
      logEvent(`Inspector focused on UAV ${droneId}`);
    }
  }
}

function onWindowResize() {
  if (!container || !renderer || !camera) return;
  const width = container.clientWidth;
  const height = container.clientHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

function logEvent(msg) {
  const feed = document.getElementById("event-feed-text");
  const timeStr = new Date().toLocaleTimeString();
  if (feed) feed.textContent = `[${timeStr}] ${msg}`;

  const logsContainer = document.getElementById("inspector-event-logs");
  if (logsContainer) {
    const row = document.createElement("div");
    row.className = "log-row";
    row.innerHTML = `<span style="color:var(--accent-cyan)">[${timeStr}]</span> ${msg}`;
    logsContainer.prepend(row);
    if (logsContainer.children.length > 50) {
      logsContainer.removeChild(logsContainer.lastChild);
    }
  }
}

// ==========================================================================
// Point Cloud SLAM Registry Manager Modal & Export
// ==========================================================================
let currentExportFormat = "xyz";

function initPointCloudModal() {
  const modal = document.getElementById("pointcloud-modal");
  const btnClose = document.getElementById("pointcloud-btn-close");
  const btnDone = document.getElementById("pointcloud-btn-done");
  const btnOpenHeader = document.getElementById("btn-open-slam-modal");
  const toolSlamMgr = document.getElementById("tool-slam-manager");
  const btnQuickSave = document.getElementById("btn-quick-save-points");

  const inputTrim = document.getElementById("input-trim-percent");
  const valTrim = document.getElementById("val-trim-percent");
  const valDelete = document.getElementById("val-trim-delete-pts");
  const valKeep = document.getElementById("val-trim-keep-pts");
  const btnExecuteTrim = document.getElementById("btn-execute-trim");
  const btnExecutePurge = document.getElementById("btn-execute-purge");
  const btnExecuteExport = document.getElementById("btn-execute-export");

  // Format selection
  document.querySelectorAll("[data-export-format]").forEach(btn => {
    btn.addEventListener("click", () => {
      document.querySelectorAll("[data-export-format]").forEach(b => b.classList.remove("active"));
      btn.classList.add("active");
      currentExportFormat = btn.getAttribute("data-export-format") || "xyz";
    });
  });

  function updateModalStats() {
    const total = state.pointCloudPoints.length;
    const badge = document.getElementById("modal-pointcloud-count-badge");
    const progress = document.getElementById("modal-pointcloud-progress");
    if (badge) badge.textContent = `${total.toLocaleString()} / ${MAX_POINTCLOUD_PTS.toLocaleString()} pts`;
    if (progress) progress.style.width = `${Math.min(100, (total / MAX_POINTCLOUD_PTS) * 100)}%`;

    const pct = parseInt(inputTrim?.value || "50", 10);
    const delPts = Math.floor(total * (pct / 100));
    const keepPts = total - delPts;

    if (valTrim) valTrim.textContent = `${pct}%`;
    if (valDelete) valDelete.textContent = `${delPts.toLocaleString()} pts`;
    if (valKeep) valKeep.textContent = `${keepPts.toLocaleString()} pts`;
  }

  inputTrim?.addEventListener("input", updateModalStats);

  btnOpenHeader?.addEventListener("click", () => {
    updateModalStats();
    modal?.classList.remove("hidden");
  });

  toolSlamMgr?.addEventListener("click", () => {
    updateModalStats();
    modal?.classList.remove("hidden");
  });

  btnClose?.addEventListener("click", () => modal?.classList.add("hidden"));
  btnDone?.addEventListener("click", () => modal?.classList.add("hidden"));

  // Close modal when clicking on background overlay
  modal?.addEventListener("click", (e) => {
    if (e.target === modal) modal.classList.add("hidden");
  });

  // Trim Action
  btnExecuteTrim?.addEventListener("click", () => {
    const total = state.pointCloudPoints.length;
    if (total === 0) {
      logEvent("Point cloud buffer is empty. Nothing to trim.");
      return;
    }
    const pct = parseInt(inputTrim?.value || "50", 10);
    const countToDelete = Math.floor(total * (pct / 100));
    if (countToDelete <= 0) return;

    const deleted = state.pointCloudPoints.splice(0, countToDelete);
    for (const p of deleted) {
      pointCloudGrid.delete(p.key);
    }

    // Rebuild geometry
    for (let i = 0; i < state.pointCloudPoints.length; i++) {
      const p = state.pointCloudPoints[i];
      pointCloudPosArray[i * 3] = p.x;
      pointCloudPosArray[i * 3 + 1] = p.y;
      pointCloudPosArray[i * 3 + 2] = p.z;
      pointCloudColArray[i * 3] = p.r;
      pointCloudColArray[i * 3 + 1] = p.g;
      pointCloudColArray[i * 3 + 2] = p.b;
    }

    if (pointCloudGeo) {
      pointCloudGeo.attributes.position.needsUpdate = true;
      pointCloudGeo.attributes.color.needsUpdate = true;
      pointCloudGeo.setDrawRange(0, state.pointCloudPoints.length);
    }

    const countElem = document.getElementById("top-pointcloud-count");
    if (countElem) countElem.textContent = `${state.pointCloudPoints.length} pts`;

    updateModalStats();
    logEvent(`Trimmed ${countToDelete.toLocaleString()} oldest SLAM points (${state.pointCloudPoints.length.toLocaleString()} remaining)`);
  });

  // Purge All Action
  btnExecutePurge?.addEventListener("click", () => {
    clearPointCloud();
    updateModalStats();
    logEvent("Purged entire 3D Point Cloud SLAM registry");
  });

  // Export Action
  btnExecuteExport?.addEventListener("click", () => {
    exportPointCloud(currentExportFormat);
  });

  btnQuickSave?.addEventListener("click", () => {
    exportPointCloud("xyz");
  });
}

function exportPointCloud(format = "xyz") {
  const points = state.pointCloudPoints;
  if (!points || points.length === 0) {
    alert("Point cloud registry is currently empty. Run drone flight or simulation to capture LiDAR points.");
    return;
  }

  let content = "";
  let mimeType = "text/plain";
  let filename = `slam_pointcloud_${Date.now()}.${format}`;

  if (format === "xyz") {
    // Standard XYZ with RGB 0-255
    const lines = ["# UAV-X Swarm Studio Point Cloud XYZ Export", "# X Y Z R G B"];
    for (const p of points) {
      const r = Math.round((p.r || 1.0) * 255);
      const g = Math.round((p.g || 1.0) * 255);
      const b = Math.round((p.b || 1.0) * 255);
      lines.push(`${p.x.toFixed(3)} ${p.y.toFixed(3)} ${p.z.toFixed(3)} ${r} ${g} ${b}`);
    }
    content = lines.join("\n");
  } else if (format === "pcd") {
    // Point Cloud Data (ROS/PCL)
    const header = [
      "# .PCD v.7 - Point Cloud Data file format",
      "VERSION .7",
      "FIELDS x y z rgb",
      "SIZE 4 4 4 4",
      "TYPE F F F U",
      "COUNT 1 1 1 1",
      `WIDTH ${points.length}`,
      "HEIGHT 1",
      "VIEWPOINT 0 0 0 1 0 0 0",
      `POINTS ${points.length}`,
      "DATA ascii"
    ];
    const lines = [...header];
    for (const p of points) {
      const r = Math.round((p.r || 1.0) * 255);
      const g = Math.round((p.g || 1.0) * 255);
      const b = Math.round((p.b || 1.0) * 255);
      const rgbInt = (r << 16) | (g << 8) | b;
      lines.push(`${p.x.toFixed(3)} ${p.y.toFixed(3)} ${p.z.toFixed(3)} ${rgbInt}`);
    }
    content = lines.join("\n");
  } else if (format === "json") {
    mimeType = "application/json";
    content = JSON.stringify({
      timestamp: new Date().toISOString(),
      points_count: points.length,
      points: points.map(p => ({
        x: Number(p.x.toFixed(3)),
        y: Number(p.y.toFixed(3)),
        z: Number(p.z.toFixed(3)),
        r: Number((p.r || 1).toFixed(2)),
        g: Number((p.g || 1).toFixed(2)),
        b: Number((p.b || 1).toFixed(2))
      }))
    }, null, 2);
  }

  const blob = new Blob([content], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);

  logEvent(`Exported ${points.length.toLocaleString()} SLAM points to ${filename}`);
}

// ==========================================================================
// Collapsible Live FPV Multi-Camera Video Deck Controller
// ==========================================================================
let fpvPollingInterval = null;
let fpvActiveDrone = 1;
let fpvActiveCam = 0;
let fpvIsMatrixView = false;
let fpvIsOpen = false;
let fpvFpsCounter = 0;
let fpvLastFpsTime = Date.now();

function initFPVVideoDeck() {
  const deck = document.getElementById("fpv-video-deck");
  const btnOpenHeader = document.getElementById("btn-open-fpv-deck");
  const toolFpv = document.getElementById("tool-fpv");
  const btnClose = document.getElementById("fpv-btn-close");
  const btnToggle = document.getElementById("fpv-btn-toggle");
  const deckBody = document.getElementById("fpv-deck-body");
  const camSelect = document.getElementById("fpv-cam-select");
  const singleView = document.getElementById("fpv-single-view");
  const matrixView = document.getElementById("fpv-matrix-view");
  const matrixGrid = document.getElementById("fpv-matrix-grid");
  const liveImg = document.getElementById("fpv-live-img");
  const placeholder = document.getElementById("fpv-placeholder");
  const titleElem = document.getElementById("fpv-active-drone-title");

  if (!deck) return;

  function toggleDeck(open) {
    fpvIsOpen = (typeof open === "boolean") ? open : deck.classList.contains("hidden");
    if (fpvIsOpen) {
      deck.classList.remove("hidden");
      btnOpenHeader?.classList.add("active");
      toolFpv?.classList.add("active");
      startFpvStream();
      logEvent("Live FPV Video Deck Opened");
    } else {
      deck.classList.add("hidden");
      btnOpenHeader?.classList.remove("active");
      toolFpv?.classList.remove("active");
      stopFpvStream();
    }
  }

  btnOpenHeader?.addEventListener("click", () => toggleDeck());
  toolFpv?.addEventListener("click", () => toggleDeck());
  btnClose?.addEventListener("click", () => toggleDeck(false));

  btnToggle?.addEventListener("click", () => {
    if (deckBody) {
      deckBody.classList.toggle("collapsed");
      btnToggle.textContent = deckBody.classList.contains("collapsed") ? "□" : "━";
    }
  });

  // Camera Sensor Selection
  camSelect?.addEventListener("change", (e) => {
    fpvActiveCam = parseInt(e.target.value || "0", 10);
    logEvent(`FPV Camera Sensor changed to Index ${fpvActiveCam}`);
  });

  function bindTabListeners() {
    document.querySelectorAll(".fpv-channel-tabs .fpv-tab").forEach(tab => {
      tab.onclick = () => {
        document.querySelectorAll(".fpv-channel-tabs .fpv-tab").forEach(t => t.classList.remove("active"));
        tab.classList.add("active");
        const target = tab.dataset.drone;

        if (target === "matrix") {
          fpvIsMatrixView = true;
          if (singleView) singleView.classList.add("hidden");
          if (matrixView) matrixView.classList.remove("hidden");
          if (titleElem) titleElem.textContent = "QUAD-VIEW SURVEILLANCE GRID";
          buildMatrixGrid();
        } else {
          fpvIsMatrixView = false;
          fpvActiveDrone = parseInt(target, 10);
          if (matrixView) matrixView.classList.add("hidden");
          if (singleView) singleView.classList.remove("hidden");
          if (titleElem) titleElem.textContent = `UAV ${fpvActiveDrone} OPTICAL FEED`;
        }
      };
    });
  }
  bindTabListeners();

  // Dynamic Tabs sync with active drone count
  window.updateFpvTabs = function(droneIds) {
    const tabsContainer = document.getElementById("fpv-drone-tabs");
    if (!tabsContainer || !droneIds || droneIds.length === 0) return;
    const existingIds = Array.from(tabsContainer.querySelectorAll(".fpv-tab:not([data-drone='matrix'])")).map(t => parseInt(t.dataset.drone));
    const sortedIds = [...droneIds].sort((a, b) => a - b);
    if (existingIds.length === sortedIds.length && existingIds.every((v, i) => v === sortedIds[i])) return;

    tabsContainer.innerHTML = "";
    sortedIds.forEach(id => {
      const btn = document.createElement("button");
      btn.className = `fpv-tab ${id === fpvActiveDrone && !fpvIsMatrixView ? 'active' : ''}`;
      btn.dataset.drone = id;
      btn.innerHTML = `<span class="tab-dot" style="background:${DRONE_COLORS[(id - 1) % DRONE_COLORS.length]}"></span> UAV ${id}`;
      tabsContainer.appendChild(btn);
    });
    const quadBtn = document.createElement("button");
    quadBtn.className = `fpv-tab ${fpvIsMatrixView ? 'active' : ''}`;
    quadBtn.dataset.drone = "matrix";
    quadBtn.textContent = "🔲 QUAD";
    tabsContainer.appendChild(quadBtn);
    bindTabListeners();
  };

  // Dragging support for floating window
  makeElementDraggable(deck, document.getElementById("fpv-deck-header"));

  function buildMatrixGrid() {
    if (!matrixGrid) return;
    matrixGrid.innerHTML = "";
    const droneIds = Object.keys(state.drones).map(Number).sort((a, b) => a - b).slice(0, 4);
    const displayIds = droneIds.length > 0 ? droneIds : [1, 2, 3, 4];
    displayIds.forEach(id => {
      const cell = document.createElement("div");
      cell.className = "fpv-matrix-cell";
      cell.innerHTML = `
        <img id="fpv-matrix-img-${id}" class="fpv-stream-img" src="" alt="UAV ${id} Feed" />
        <div class="fpv-matrix-cell-label" style="border-left: 3px solid ${DRONE_COLORS[(id - 1) % DRONE_COLORS.length] || '#00f5d4'}">
          UAV ${id} | <span id="fpv-matrix-alt-${id}">0.0m</span>
        </div>
      `;
      cell.addEventListener("click", () => {
        document.querySelector(`.fpv-channel-tabs .fpv-tab[data-drone="${id}"]`)?.click();
      });
      matrixGrid.appendChild(cell);
    });
  }

  function startFpvStream() {
    if (fpvPollingInterval) clearInterval(fpvPollingInterval);
    fpvPollingInterval = setInterval(fetchNextFpvFrame, 120); // ~8-10 FPS smooth stream
  }

  function stopFpvStream() {
    if (fpvPollingInterval) {
      clearInterval(fpvPollingInterval);
      fpvPollingInterval = null;
    }
  }

  let isFetching = false;
  async function fetchNextFpvFrame() {
    if (!fpvIsOpen || isFetching) return;
    isFetching = true;

    try {
      if (!fpvIsMatrixView) {
        // Single view fetch
        const targetId = fpvActiveDrone;
        const res = await fetch(`/api/camera?drone=${targetId}&camera=${fpvActiveCam}&t=${Date.now()}`);
        if (res.ok && res.headers.get("content-type")?.includes("image")) {
          const blob = await res.blob();
          if (blob.size > 500) {
            const objectUrl = URL.createObjectURL(blob);
            if (liveImg) {
              const oldSrc = liveImg.src;
              liveImg.src = objectUrl;
              liveImg.style.display = "block";
              if (placeholder) placeholder.style.display = "none";
              if (oldSrc && oldSrc.startsWith("blob:")) URL.revokeObjectURL(oldSrc);
            }
            updateFpvHud(targetId);
          }
        }
      } else {
        // Quad matrix view fetch (interleaved)
        for (let id = 1; id <= 4; id++) {
          const imgElem = document.getElementById(`fpv-matrix-img-${id}`);
          const altElem = document.getElementById(`fpv-matrix-alt-${id}`);
          const d = state.drones[id];
          if (altElem && d && d.telemetry) {
            altElem.textContent = `${(d.telemetry.alt || 0).toFixed(1)}m`;
          }

          try {
            const res = await fetch(`/api/camera?drone=${id}&camera=0&t=${Date.now()}`);
            if (res.ok && res.headers.get("content-type")?.includes("image")) {
              const blob = await res.blob();
              if (blob.size > 500 && imgElem) {
                const objectUrl = URL.createObjectURL(blob);
                const oldSrc = imgElem.src;
                imgElem.src = objectUrl;
                if (oldSrc && oldSrc.startsWith("blob:")) URL.revokeObjectURL(oldSrc);
              }
            }
          } catch (e) { }
        }
      }

      // Compute FPS counter
      fpvFpsCounter++;
      const now = Date.now();
      if (now - fpvLastFpsTime >= 1000) {
        const fpsElem = document.getElementById("fpv-hud-fps");
        if (fpsElem) fpsElem.textContent = `${fpvFpsCounter} FPS | 640x480 RGB | AirSim RPC`;
        fpvFpsCounter = 0;
        fpvLastFpsTime = now;
      }
    } catch (err) {
      // Network or simulation standby
    } finally {
      isFetching = false;
    }
  }

  function updateFpvHud(id) {
    const d = state.drones[id];
    if (!d || !d.telemetry) return;
    const telem = d.telemetry;

    const altElem = document.getElementById("fpv-hud-alt");
    const spdElem = document.getElementById("fpv-hud-spd");
    const hdgElem = document.getElementById("fpv-hud-hdg");
    const battElem = document.getElementById("fpv-hud-batt");
    const modeElem = document.getElementById("fpv-hud-mode");
    const rssiElem = document.getElementById("fpv-hud-rssi");
    const coordsElem = document.getElementById("fpv-hud-coords");
    const ladder = document.getElementById("fpv-horizon-ladder");

    const alt = telem.alt != null ? telem.alt.toFixed(1) : "0.0";
    const spd = telem.speed != null ? telem.speed.toFixed(1) : "0.0";
    const hdg = Math.round(telem.heading || 0);
    const batt = Math.round(telem.battery != null ? telem.battery : 100);
    const mode = telem.flight_mode || "GUIDED";

    if (altElem) altElem.textContent = `${alt}m`;
    if (spdElem) spdElem.textContent = `${spd}m/s`;
    if (hdgElem) hdgElem.textContent = `${hdg}° ${getCompassSector(hdg)}`;
    if (battElem) battElem.textContent = `${batt}%`;
    if (modeElem) modeElem.textContent = mode;
    if (rssiElem) rssiElem.textContent = `${telem.rssi || 98}%`;

    if (coordsElem && telem.lat && telem.lon) {
      coordsElem.textContent = `GPS: ${telem.lat.toFixed(5)}, ${telem.lon.toFixed(5)} | SATS: ${telem.sats || 14}`;
    }

    // Artificial horizon tilt & pitch
    if (ladder && telem.roll != null && telem.pitch != null) {
      const rollDeg = telem.roll;
      const pitchPx = telem.pitch * 2.0;
      ladder.style.transform = `translate(-50%, calc(-50% + ${pitchPx}px)) rotate(${-rollDeg}deg)`;
    }
  }

  function getCompassSector(deg) {
    const sectors = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    return sectors[Math.round(((deg % 360) / 45)) % 8];
  }

  function makeElementDraggable(elm, handle) {
    let pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    if (handle) {
      handle.onmousedown = dragMouseDown;
    } else {
      elm.onmousedown = dragMouseDown;
    }

    function dragMouseDown(e) {
      if (e.target.tagName === "BUTTON" || e.target.tagName === "SELECT" || e.target.classList.contains("fpv-tab")) return;
      e.preventDefault();
      pos3 = e.clientX;
      pos4 = e.clientY;
      document.onmouseup = closeDragElement;
      document.onmousemove = elementDrag;
    }

    function elementDrag(e) {
      e.preventDefault();
      pos1 = pos3 - e.clientX;
      pos2 = pos4 - e.clientY;
      pos3 = e.clientX;
      pos4 = e.clientY;
      elm.style.top = (elm.offsetTop - pos2) + "px";
      elm.style.left = (elm.offsetLeft - pos1) + "px";
      elm.style.bottom = "auto";
      elm.style.right = "auto";
    }

    function closeDragElement() {
      document.onmouseup = null;
      document.onmousemove = null;
    }
  }
}
