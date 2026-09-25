import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { VISION, WeedingScenario } from '@/core/scenarios/weeding/weeding';
import { CHUNK_LENGTH, groundHeight, type FieldPreset } from '@/core/scenarios/weeding/field';
import { TOOL_LENGTH } from '@/core/scenarios/pickPlace/pickPlace';
import { Rng } from '@/core/math/rng';
import { DeltaRobotModel, type RobotSnapshot } from '../robot/DeltaRobotModel';
import { ReachFootprint, WorkspaceEnvelope } from '../robot/WorkspaceEnvelope';
import { PathPreview } from '../robot/PathPreview';
import { PALETTE } from '../helpers';
import { useApp } from '@/state/store';

const FIELD_LENGTH = 28;
const MAX_PLANTS = 4000;
const MAX_MARKERS = 400;

export function WeedingScene({ scenario }: { scenario: WeedingScenario }) {
  const view = useApp((s) => s.view);
  const tool = TOOL_LENGTH.fingers;
  const readRobot = (): RobotSnapshot => ({
    params: scenario.params,
    base: scenario.robotBase(),
    theta: scenario.robot.theta,
    flange: scenario.robot.flange,
    yaw: scenario.robot.state.yaw,
    gripper: scenario.robot.gripper,
    tool,
  });
  const vehicle = useRef<THREE.Group>(null);
  useFrame(() => {
    vehicle.current?.position.set(scenario.vehicleX, 0, 0);
  });
  const baseZ = scenario.robotBase().z;
  const groundZ = scenario.preset.bedHeight - baseZ;

  return (
    <group>
      <Field scenario={scenario} />
      <Plants scenario={scenario} />
      {view.markers && <DetectionMarkers scenario={scenario} />}
      <group ref={vehicle}>
        <Vehicle scenario={scenario} />
        {view.vision && <VisionZone preset={scenario.preset} />}
        <group position={[0, 0, baseZ]}>
          {view.workspace && <WorkspaceEnvelope params={scenario.params} tool={tool} color="#a3e635" />}
          {view.workspace && (
            <ReachFootprint params={scenario.params} tool={tool} z={groundZ + 0.012} color="#a3e635" />
          )}
        </group>
      </group>
      <DeltaRobotModel read={readRobot} tool="fingers" />
      {view.path && (
        <PathPreview
          read={() => ({
            base: scenario.robotBase(),
            points: scenario.robot.phase === 'idle' ? [] : scenario.robot.preview,
          })}
        />
      )}
    </group>
  );
}

// ---- terrain ---------------------------------------------------------------------------------------

function buildFieldGeometry(preset: FieldPreset): THREE.BufferGeometry {
  const halfWidth = 3.2;
  // lateral profile samples: dense around bed edges, coarse elsewhere
  const ys = new Set<number>();
  for (let y = -halfWidth; y <= halfWidth + 1e-9; y += 0.1) ys.add(Number(y.toFixed(3)));
  for (const c of preset.beds) {
    const h = preset.bedTopWidth / 2;
    for (const d of [-h - preset.bedSlope, -h, h, h + preset.bedSlope]) ys.add(Number((c + d).toFixed(3)));
  }
  const yList = [...ys].filter((y) => Math.abs(y) <= halfWidth).sort((a, b) => a - b);
  const dx = 0.2;
  const nx = Math.round(FIELD_LENGTH / dx) + 1;
  const ny = yList.length;
  const pos = new Float32Array(nx * ny * 3);
  const col = new Float32Array(nx * ny * 3);
  const rng = new Rng(7);
  const top = new THREE.Color(PALETTE.soil);
  const low = new THREE.Color(PALETTE.soilDark);
  const c = new THREE.Color();
  // jitter must repeat every chunk so the field can be shifted seamlessly
  const period = Math.round(CHUNK_LENGTH / dx);
  const jitter: number[] = [];
  for (let k = 0; k < period * ny; k++) jitter.push(rng.range(-0.012, 0.012));
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < ny; j++) {
      const y = yList[j]!;
      const g = groundHeight(preset, y);
      const k = (i * ny + j) * 3;
      pos[k] = i * dx - FIELD_LENGTH / 2;
      pos[k + 1] = y;
      pos[k + 2] = g + jitter[(i % period) * ny + j]!;
      const t = preset.bedHeight > 0 ? g / preset.bedHeight : 0.7;
      c.copy(low).lerp(top, t);
      c.offsetHSL(0, 0, jitter[(i % period) * ny + j]! * 2);
      col[k] = c.r;
      col[k + 1] = c.g;
      col[k + 2] = c.b;
    }
  }
  const index: number[] = [];
  for (let i = 0; i < nx - 1; i++)
    for (let j = 0; j < ny - 1; j++) {
      const a = i * ny + j;
      const b = a + ny;
      index.push(a, b, a + 1, b, b + 1, a + 1);
    }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  geo.setIndex(index);
  const flat = geo.toNonIndexed();
  flat.computeVertexNormals();
  return flat;
}

function Field({ scenario }: { scenario: WeedingScenario }) {
  const geometry = useMemo(() => buildFieldGeometry(scenario.preset), [scenario.preset]);
  const ref = useRef<THREE.Group>(null);
  useFrame(() => {
    // snap to chunk length so the repeating jitter pattern stays in place
    if (ref.current) ref.current.position.x = Math.floor(scenario.vehicleX / CHUNK_LENGTH) * CHUNK_LENGTH + 4;
  });
  return (
    <group ref={ref}>
      <mesh geometry={geometry} receiveShadow>
        <meshStandardMaterial vertexColors flatShading roughness={0.95} />
      </mesh>
      <mesh position-z={-0.01} receiveShadow>
        <planeGeometry args={[FIELD_LENGTH, 60]} />
        <meshStandardMaterial color={PALETTE.soilDark} roughness={1} />
      </mesh>
    </group>
  );
}

// ---- plants ----------------------------------------------------------------------------------------

function mergeNonIndexed(parts: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const flat = parts.map((p) => (p.index ? p.toNonIndexed() : p));
  const n = flat.reduce((s, g) => s + g.getAttribute('position').count, 0);
  const pos = new Float32Array(n * 3);
  let o = 0;
  for (const g of flat) {
    pos.set(g.getAttribute('position').array as Float32Array, o * 3);
    o += g.getAttribute('position').count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.computeVertexNormals();
  return out;
}

/** Low-poly lettuce-like crop, unit radius / unit height, standing on z = 0. */
function cropGeometry() {
  const parts: THREE.BufferGeometry[] = [];
  const core = new THREE.IcosahedronGeometry(0.55, 0);
  core.scale(1, 1, 0.9);
  core.translate(0, 0, 0.5);
  parts.push(core);
  for (let k = 0; k < 6; k++) {
    const leaf = new THREE.IcosahedronGeometry(0.42, 0);
    leaf.scale(1.2, 0.7, 0.55);
    const a = (k / 6) * Math.PI * 2;
    leaf.rotateZ(a);
    leaf.translate(Math.cos(a) * 0.55, Math.sin(a) * 0.55, 0.28);
    parts.push(leaf);
  }
  return mergeNonIndexed(parts);
}

/** Spiky weed rosette, unit radius / unit height. */
function weedGeometry() {
  const parts: THREE.BufferGeometry[] = [];
  for (let k = 0; k < 7; k++) {
    const leaf = new THREE.ConeGeometry(0.16, 1, 3);
    leaf.rotateX(Math.PI / 2 - 0.6 - (k % 2) * 0.35);
    leaf.translate(0, 0.45, 0.35);
    leaf.rotateZ((k / 7) * Math.PI * 2);
    parts.push(leaf);
  }
  return mergeNonIndexed(parts);
}

const tmpObj = new THREE.Object3D();
const tmpColor = new THREE.Color();
const CROP_COLORS = ['#4ade80', '#22c55e', '#65d98a'];
const WEED_COLORS = ['#d4d93a', '#e2c93b', '#b8cc2e'];

function Plants({ scenario }: { scenario: WeedingScenario }) {
  const crops = useRef<THREE.InstancedMesh>(null);
  const weeds = useRef<THREE.InstancedMesh>(null);
  const geos = useMemo(() => ({ crop: cropGeometry(), weed: weedGeometry() }), []);

  useFrame(() => {
    let nc = 0;
    let nw = 0;
    for (const p of scenario.plants.values()) {
      if (p.state === 'binned') continue;
      const mesh = p.kind === 'crop' ? crops.current : weeds.current;
      if (!mesh) continue;
      const i = p.kind === 'crop' ? nc++ : nw++;
      if (i >= MAX_PLANTS) continue;
      const lifted = p.state === 'carried';
      const dead = p.state === 'damaged';
      tmpObj.position.set(p.pos.x, p.pos.y, p.pos.z - (lifted ? p.height * 0.3 : 0));
      tmpObj.rotation.set(dead ? 1.2 : 0, 0, p.rotation);
      // weeds are drawn a bit larger than their collision size so they read well on screen
      const k = p.kind === 'weed' ? 1.35 : 1;
      tmpObj.scale.set(p.radius * k, p.radius * k, p.height * k * (dead ? 0.5 : 1));
      tmpObj.updateMatrix();
      mesh.setMatrixAt(i, tmpObj.matrix);
      tmpColor.set((p.kind === 'crop' ? CROP_COLORS : WEED_COLORS)[p.variant % 3]!);
      if (dead) tmpColor.set('#7c6a3a');
      mesh.setColorAt(i, tmpColor);
    }
    for (const [mesh, n] of [
      [crops.current, nc],
      [weeds.current, nw],
    ] as const) {
      if (!mesh) continue;
      mesh.count = Math.min(n, MAX_PLANTS);
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    }
  });

  return (
    <group>
      <instancedMesh
        ref={crops}
        args={[geos.crop, undefined, MAX_PLANTS]}
        castShadow
        receiveShadow
        frustumCulled={false}
      >
        <meshStandardMaterial flatShading roughness={0.7} />
      </instancedMesh>
      <instancedMesh ref={weeds} args={[geos.weed, undefined, MAX_PLANTS]} castShadow frustumCulled={false}>
        <meshStandardMaterial flatShading roughness={0.8} side={THREE.DoubleSide} />
      </instancedMesh>
    </group>
  );
}

/** Rings under detected plants: red = weed target, amber = skipped (too close to a crop), green = crop. */
function DetectionMarkers({ scenario }: { scenario: WeedingScenario }) {
  const mesh = useRef<THREE.InstancedMesh>(null);
  const geometry = useMemo(() => new THREE.RingGeometry(0.82, 1, 20), []);
  useFrame(({ clock }) => {
    const m = mesh.current;
    if (!m) return;
    let n = 0;
    const targetId = scenario.robot.pickTarget?.id;
    for (const d of scenario.detections.values()) {
      if (n >= MAX_MARKERS) break;
      const p = scenario.plants.get(d.plantId);
      if (!p || (p.state !== 'growing' && p.state !== 'reserved')) continue;
      if (p.pos.x < scenario.vehicleX - 0.8) continue;
      const isTarget = d.plantId === targetId;
      const pulse = isTarget ? 1 + 0.15 * Math.sin(clock.elapsedTime * 12) : 1;
      tmpObj.position.set(d.pos.x, d.pos.y, d.pos.z + 0.004);
      tmpObj.rotation.set(0, 0, 0);
      tmpObj.scale.setScalar((p.radius + 0.015) * pulse);
      tmpObj.updateMatrix();
      m.setMatrixAt(n, tmpObj.matrix);
      if (d.label === 'crop') tmpColor.set('#34d399').multiplyScalar(0.3);
      else if (d.tooClose) tmpColor.set('#fbbf24');
      else tmpColor.set(isTarget ? '#ff4d6d' : '#fb7185');
      m.setColorAt(n, tmpColor);
      n++;
    }
    m.count = n;
    m.instanceMatrix.needsUpdate = true;
    if (m.instanceColor) m.instanceColor.needsUpdate = true;
  });
  return (
    <instancedMesh ref={mesh} args={[geometry, undefined, MAX_MARKERS]} frustumCulled={false} renderOrder={5}>
      <meshBasicMaterial transparent opacity={0.9} depthWrite={false} toneMapped={false} />
    </instancedMesh>
  );
}

// ---- vehicle ---------------------------------------------------------------------------------------

const frameMat = new THREE.MeshStandardMaterial({
  color: '#d7dde4',
  roughness: 0.4,
  metalness: 0.3,
  flatShading: true,
});
const darkMat = new THREE.MeshStandardMaterial({
  color: '#2a323c',
  roughness: 0.6,
  metalness: 0.3,
  flatShading: true,
});
const tyreMat = new THREE.MeshStandardMaterial({ color: '#15191e', roughness: 0.9, flatShading: true });
const accentMat = new THREE.MeshStandardMaterial({
  color: PALETTE.accent,
  roughness: 0.4,
  flatShading: true,
});

function Vehicle({ scenario }: { scenario: WeedingScenario }) {
  const preset = scenario.preset;
  const baseZ = scenario.robotBase().z;
  const raised = preset.bedHeight > 0;
  const track = raised ? preset.bedTopWidth / 2 + preset.bedSlope + 0.15 : 1.3;
  const wheelR = 0.22;
  const frameZ = baseZ + 0.16;
  const wheels = useRef<THREE.Group>(null);
  const fill = useRef<THREE.Mesh>(null);
  const bin = scenario.binPosition();
  useFrame(() => {
    wheels.current?.children.forEach((w) => (w.rotation.y = scenario.vehicleX / wheelR));
    if (fill.current) {
      const f = Math.max(0.02, scenario.binFill / WeedingScenario.BIN_CAPACITY);
      fill.current.scale.z = f;
      fill.current.position.z = -0.1 + (f * 0.09) / 2 + 0.005;
    }
  });
  const legs: [number, number][] = [
    [-0.55, -track],
    [0.55, -track],
    [-0.55, track],
    [0.55, track],
  ];
  return (
    <group>
      <group ref={wheels}>
        {legs.map(([x, y], i) => (
          <group key={i} position={[x, y, wheelR]}>
            <mesh material={tyreMat} castShadow>
              <cylinderGeometry args={[wheelR, wheelR, 0.11, 16]} />
            </mesh>
            <mesh material={frameMat}>
              <cylinderGeometry args={[wheelR * 0.45, wheelR * 0.45, 0.13, 8]} />
            </mesh>
          </group>
        ))}
      </group>
      {legs.map(([x, y], i) => (
        <mesh key={i} position={[x, y, (wheelR + frameZ) / 2]} material={frameMat} castShadow>
          <boxGeometry args={[0.07, 0.07, frameZ - wheelR]} />
        </mesh>
      ))}
      {[-track, track].map((y) => (
        <mesh key={y} position={[0, y, frameZ]} material={frameMat} castShadow>
          <boxGeometry args={[1.25, 0.08, 0.08]} />
        </mesh>
      ))}
      {[-0.55, 0.55, 0].map((x) => (
        <mesh key={x} position={[x, 0, frameZ]} material={frameMat} castShadow>
          <boxGeometry args={[0.08, track * 2 + 0.08, 0.08]} />
        </mesh>
      ))}
      {/* roof rails (kept open so the robot stays visible) */}
      {[-0.6, 0.6].map((x) =>
        [-track, track].map((y) => (
          <mesh key={`${x}${y}`} position={[x, y, frameZ + 0.14]} material={darkMat}>
            <boxGeometry args={[0.04, 0.04, 0.28]} />
          </mesh>
        )),
      )}
      {[-track, track].map((y) => (
        <mesh key={y} position={[0, y, frameZ + 0.28]} material={darkMat} castShadow>
          <boxGeometry args={[1.25, 0.05, 0.03]} />
        </mesh>
      ))}
      <mesh position={[-0.6, 0, frameZ + 0.28]} castShadow>
        <boxGeometry args={[0.22, track * 2, 0.02]} />
        <meshStandardMaterial color="#1e3a5f" roughness={0.25} metalness={0.6} />
      </mesh>
      {/* electronics box */}
      <mesh position={[-0.4, -track + 0.14, frameZ - 0.1]} material={darkMat} castShadow>
        <boxGeometry args={[0.3, 0.18, 0.18]} />
      </mesh>
      <mesh position={[-0.3, -track + 0.05, frameZ - 0.02]} material={accentMat}>
        <boxGeometry args={[0.04, 0.012, 0.04]} />
      </mesh>
      {/* camera boom */}
      <mesh position={[(VISION.xNear + VISION.xFar) / 4 + 0.2, 0, frameZ]} material={frameMat} castShadow>
        <boxGeometry args={[(VISION.xNear + VISION.xFar) / 2, 0.05, 0.05]} />
      </mesh>
      <mesh
        position={[(VISION.xNear + VISION.xFar) / 2 + 0.05, 0, frameZ - 0.05]}
        material={darkMat}
        castShadow
      >
        <boxGeometry args={[0.1, 0.16, 0.08]} />
      </mesh>
      {/* weed bin next to the robot */}
      {scenario.config.disposal === 'bin' && (
        <group position={[bin.x, bin.y, baseZ + bin.z]}>
          <mesh position-z={-0.1} material={darkMat} castShadow>
            <boxGeometry args={[0.2, 0.14, 0.012]} />
          </mesh>
          {[-1, 1].map((s) => (
            <group key={s}>
              <mesh position={[0, s * 0.07, -0.05]} material={accentMat} castShadow>
                <boxGeometry args={[0.2, 0.008, 0.1]} />
              </mesh>
              <mesh position={[s * 0.1, 0, -0.05]} material={accentMat} castShadow>
                <boxGeometry args={[0.008, 0.14, 0.1]} />
              </mesh>
            </group>
          ))}
          <mesh ref={fill} position-z={-0.09}>
            <boxGeometry args={[0.18, 0.12, 0.09]} />
            <meshStandardMaterial color="#7d8f33" flatShading />
          </mesh>
          <mesh position={[0, bin.y > 0 ? -0.09 : 0.09, 0.1]} material={frameMat}>
            <boxGeometry args={[0.03, 0.03, 0.3]} />
          </mesh>
        </group>
      )}
    </group>
  );
}

function VisionZone({ preset }: { preset: FieldPreset }) {
  const scan = useRef<THREE.Mesh>(null);
  const len = VISION.xFar - VISION.xNear;
  const w = preset.laneHalfWidth * 2;
  const z = preset.bedHeight + 0.006;
  const camZ = preset.bedHeight + preset.baseHeight + 0.1;
  const camX = (VISION.xNear + VISION.xFar) / 2 + 0.05;
  useFrame(({ clock }) => {
    if (scan.current) scan.current.position.x = VISION.xNear + ((clock.elapsedTime * 0.9) % 1) * len;
  });
  const frustum = useMemo(() => {
    const corners = [
      [VISION.xNear, -w / 2],
      [VISION.xFar, -w / 2],
      [VISION.xFar, w / 2],
      [VISION.xNear, w / 2],
    ];
    const pts: number[] = [];
    for (const [x, y] of corners) pts.push(camX, 0, camZ, x!, y!, z);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    return g;
  }, [w, z, camZ, camX]);
  return (
    <group>
      <mesh position={[(VISION.xNear + VISION.xFar) / 2, 0, z]} renderOrder={3}>
        <planeGeometry args={[len, w]} />
        <meshBasicMaterial color="#38bdf8" transparent opacity={0.1} depthWrite={false} />
      </mesh>
      <lineSegments geometry={frustum}>
        <lineBasicMaterial color="#38bdf8" transparent opacity={0.35} />
      </lineSegments>
      <mesh ref={scan} position={[VISION.xNear, 0, z + 0.002]} renderOrder={4}>
        <planeGeometry args={[0.012, w]} />
        <meshBasicMaterial color="#7dd3fc" transparent opacity={0.8} depthWrite={false} toneMapped={false} />
      </mesh>
    </group>
  );
}
