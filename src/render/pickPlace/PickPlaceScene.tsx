import { useEffect, useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import {
  LAYOUT,
  PRODUCT_SIZE,
  TOOL_LENGTH,
  type PickPlaceScenario,
} from '@/core/scenarios/pickPlace/pickPlace';
import { DeltaRobotModel, type RobotSnapshot } from '../robot/DeltaRobotModel';
import { ReachFootprint, WorkspaceEnvelope } from '../robot/WorkspaceEnvelope';
import { PathPreview } from '../robot/PathPreview';
import { PALETTE } from '../helpers';
import { useApp } from '@/state/store';

const MAX_PRODUCTS = 240;
const MAX_TRAYS = 40;

export function PickPlaceScene({ scenario }: { scenario: PickPlaceScenario }) {
  const view = useApp((s) => s.view);
  const base = scenario.robotBase();
  const pickZ = LAYOUT.beltHeight + PRODUCT_SIZE.box.h - base.z;

  const readRobot = (): RobotSnapshot => ({
    params: scenario.params,
    base: scenario.robotBase(),
    theta: scenario.robot.theta,
    flange: scenario.robot.flange,
    yaw: scenario.robot.state.yaw,
    gripper: scenario.robot.gripper,
    tool: TOOL_LENGTH.suction,
  });

  return (
    <group>
      <Gantry height={base.z} />
      <Conveyor
        y={LAYOUT.belt.y}
        width={LAYOUT.belt.width}
        xStart={LAYOUT.belt.xStart}
        xEnd={LAYOUT.belt.xEnd}
        travel={() => scenario.beltTravel}
      />
      {scenario.config.placeMode === 'trayConveyor' ? (
        <Conveyor
          y={LAYOUT.trayBelt.y}
          width={LAYOUT.trayBelt.width}
          xStart={LAYOUT.trayBelt.xStart}
          xEnd={LAYOUT.trayBelt.xEnd}
          travel={() => scenario.trayTravel}
        />
      ) : (
        <TrayTable />
      )}
      <RejectBin />
      <Products scenario={scenario} />
      <Trays scenario={scenario} />
      <DeltaRobotModel read={readRobot} tool="suction" />
      <group position={[base.x, base.y, base.z]}>
        {view.workspace && <WorkspaceEnvelope params={scenario.params} tool={TOOL_LENGTH.suction} />}
        {view.workspace && (
          <ReachFootprint params={scenario.params} tool={TOOL_LENGTH.suction} z={pickZ - 0.045} />
        )}
      </group>
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

const frameMat = new THREE.MeshStandardMaterial({
  color: PALETTE.frame,
  roughness: 0.35,
  metalness: 0.7,
  flatShading: true,
});
const darkMat = new THREE.MeshStandardMaterial({
  color: PALETTE.frameDark,
  roughness: 0.6,
  metalness: 0.4,
  flatShading: true,
});

/** Portal frame carrying the robot above the conveyors. */
function Gantry({ height }: { height: number }) {
  const posts: [number, number][] = [
    [-0.75, -0.8],
    [0.75, -0.8],
    [-0.75, 0.8],
    [0.75, 0.8],
  ];
  const top = height + 0.12;
  return (
    <group>
      {posts.map(([x, y], i) => (
        <mesh key={i} position={[x, y, top / 2]} material={frameMat} castShadow receiveShadow>
          <boxGeometry args={[0.08, 0.08, top]} />
        </mesh>
      ))}
      {[-0.8, 0.8].map((y) => (
        <mesh key={y} position={[0, y, top]} material={frameMat} castShadow>
          <boxGeometry args={[1.58, 0.08, 0.08]} />
        </mesh>
      ))}
      {[-0.75, 0.75].map((x) => (
        <mesh key={x} position={[x, 0, top]} material={frameMat} castShadow>
          <boxGeometry args={[0.08, 1.68, 0.08]} />
        </mesh>
      ))}
      {/* robot mounting plate */}
      <mesh position={[0, 0, top + 0.02]} material={darkMat} castShadow receiveShadow>
        <boxGeometry args={[1.5, 0.5, 0.03]} />
      </mesh>
      {[-0.4, 0.4].map((x) => (
        <mesh key={x} position={[x, 0, top - 0.02]} material={frameMat}>
          <boxGeometry args={[0.06, 1.6, 0.05]} />
        </mesh>
      ))}
    </group>
  );
}

/** Belt conveyor with a scrolling belt texture. */
function Conveyor({
  y,
  width,
  xStart,
  xEnd,
  travel,
}: {
  y: number;
  width: number;
  xStart: number;
  xEnd: number;
  travel: () => number;
}) {
  const length = xEnd - xStart;
  const h = LAYOUT.beltHeight;
  const texture = useMemo(() => {
    const c = document.createElement('canvas');
    c.width = 64;
    c.height = 64;
    const g = c.getContext('2d')!;
    g.fillStyle = '#20262e';
    g.fillRect(0, 0, 64, 64);
    g.fillStyle = '#2a323c';
    g.fillRect(0, 0, 8, 64);
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(length / 0.12, 1);
    t.colorSpace = THREE.SRGBColorSpace;
    return t;
  }, [length]);
  useEffect(() => () => texture.dispose(), [texture]);

  useFrame(() => {
    texture.offset.x = -travel() / 0.12;
  });

  const legs = [];
  for (let x = xStart + 0.3; x < xEnd; x += 1.2) legs.push(x);

  return (
    <group position={[(xStart + xEnd) / 2, y, 0]}>
      <mesh position-z={h - 0.005} receiveShadow>
        <boxGeometry args={[length, width, 0.01]} />
        <meshStandardMaterial map={texture} roughness={0.85} />
      </mesh>
      {[-1, 1].map((s) => (
        <mesh
          key={s}
          position={[0, (s * (width + 0.04)) / 2, h - 0.01]}
          material={frameMat}
          castShadow
          receiveShadow
        >
          <boxGeometry args={[length, 0.04, 0.07]} />
        </mesh>
      ))}
      <mesh position-z={h - 0.06} material={darkMat} receiveShadow>
        <boxGeometry args={[length, width, 0.09]} />
      </mesh>
      {[-length / 2, length / 2].map((x) => (
        <mesh key={x} position={[x, 0, h - 0.04]} material={frameMat}>
          <cylinderGeometry args={[0.04, 0.04, width + 0.02, 16]} />
        </mesh>
      ))}
      {legs.map((x) => (
        <group key={x} position-x={x - (xStart + xEnd) / 2}>
          {[-1, 1].map((s) => (
            <mesh key={s} position={[0, (s * width) / 2.4, (h - 0.1) / 2]} material={darkMat} castShadow>
              <boxGeometry args={[0.05, 0.05, h - 0.1]} />
            </mesh>
          ))}
        </group>
      ))}
    </group>
  );
}

function TrayTable() {
  return (
    <group position={[-0.05, 0.32, 0]}>
      <mesh position-z={LAYOUT.beltHeight - 0.03} material={darkMat} castShadow receiveShadow>
        <boxGeometry args={[1.0, 0.42, 0.06]} />
      </mesh>
      {[
        [-0.45, -0.17],
        [0.45, -0.17],
        [-0.45, 0.17],
        [0.45, 0.17],
      ].map(([x, y], i) => (
        <mesh key={i} position={[x!, y!, (LAYOUT.beltHeight - 0.06) / 2]} material={frameMat}>
          <boxGeometry args={[0.04, 0.04, LAYOUT.beltHeight - 0.06]} />
        </mesh>
      ))}
    </group>
  );
}

function RejectBin() {
  const x = LAYOUT.belt.xEnd + 0.25;
  return (
    <group position={[x, LAYOUT.belt.y, 0]}>
      {[
        [0, -0.24, 0.02, 0.5],
        [0, 0.24, 0.02, 0.5],
      ].map(([px, py], i) => (
        <mesh key={i} position={[px!, py!, 0.25]} castShadow receiveShadow>
          <boxGeometry args={[0.5, 0.02, 0.5]} />
          <meshStandardMaterial color="#b45309" roughness={0.7} flatShading />
        </mesh>
      ))}
      {[-0.24, 0.24].map((px) => (
        <mesh key={px} position={[px, 0, 0.25]} castShadow receiveShadow>
          <boxGeometry args={[0.02, 0.5, 0.5]} />
          <meshStandardMaterial color="#b45309" roughness={0.7} flatShading />
        </mesh>
      ))}
    </group>
  );
}

const tmpObj = new THREE.Object3D();
const tmpColor = new THREE.Color();

function Products({ scenario }: { scenario: PickPlaceScenario }) {
  const boxes = useRef<THREE.InstancedMesh>(null);
  const cyls = useRef<THREE.InstancedMesh>(null);
  const box = PRODUCT_SIZE.box;
  const cyl = PRODUCT_SIZE.cylinder;

  useFrame(() => {
    let nb = 0;
    let nc = 0;
    for (const p of scenario.products) {
      const mesh = p.kind === 'box' ? boxes.current : cyls.current;
      if (!mesh) continue;
      const i = p.kind === 'box' ? nb++ : nc++;
      if (i >= MAX_PRODUCTS) continue;
      tmpObj.position.set(p.pos.x, p.pos.y, p.pos.z + PRODUCT_SIZE[p.kind].h / 2);
      tmpObj.rotation.set(p.kind === 'box' ? 0 : Math.PI / 2, 0, p.yaw);
      tmpObj.scale.setScalar(1);
      tmpObj.updateMatrix();
      mesh.setMatrixAt(i, tmpObj.matrix);
      tmpColor.set(p.color);
      if (p.state === 'missed') tmpColor.multiplyScalar(0.45);
      mesh.setColorAt(i, tmpColor);
    }
    for (const [mesh, n] of [
      [boxes.current, nb],
      [cyls.current, nc],
    ] as const) {
      if (!mesh) continue;
      mesh.count = Math.min(n, MAX_PRODUCTS);
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    }
  });

  return (
    <group>
      <instancedMesh
        ref={boxes}
        args={[undefined, undefined, MAX_PRODUCTS]}
        castShadow
        receiveShadow
        frustumCulled={false}
      >
        <boxGeometry args={[box.x, box.y, box.h]} />
        <meshStandardMaterial roughness={0.45} flatShading />
      </instancedMesh>
      <instancedMesh
        ref={cyls}
        args={[undefined, undefined, MAX_PRODUCTS]}
        castShadow
        receiveShadow
        frustumCulled={false}
      >
        <cylinderGeometry args={[cyl.x / 2, cyl.x / 2, cyl.h, 12]} />
        <meshStandardMaterial roughness={0.35} flatShading />
      </instancedMesh>
    </group>
  );
}

function trayGeometry(): THREE.BufferGeometry {
  const { size, height } = LAYOUT.tray;
  const parts: THREE.BufferGeometry[] = [];
  const bottom = new THREE.BoxGeometry(size, size, 0.008);
  bottom.translate(0, 0, 0.004);
  parts.push(bottom);
  for (const s of [-1, 1]) {
    const a = new THREE.BoxGeometry(size, 0.008, height);
    a.translate(0, (s * (size - 0.008)) / 2, height / 2);
    const b = new THREE.BoxGeometry(0.008, size, height);
    b.translate((s * (size - 0.008)) / 2, 0, height / 2);
    parts.push(a, b);
  }
  // slot dividers
  const div1 = new THREE.BoxGeometry(size, 0.005, height * 0.6);
  div1.translate(0, 0, height * 0.3);
  const div2 = new THREE.BoxGeometry(0.005, size, height * 0.6);
  div2.translate(0, 0, height * 0.3);
  parts.push(div1, div2);
  return mergeGeometries(parts);
}

/** Tiny merge helper (all parts are non-indexed after toNonIndexed) to avoid importing examples. */
function mergeGeometries(parts: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const flat = parts.map((p) => p.toNonIndexed());
  const count = flat.reduce((n, g) => n + g.getAttribute('position').count, 0);
  const pos = new Float32Array(count * 3);
  const nor = new Float32Array(count * 3);
  let o = 0;
  for (const g of flat) {
    pos.set(g.getAttribute('position').array as Float32Array, o * 3);
    nor.set(g.getAttribute('normal').array as Float32Array, o * 3);
    o += g.getAttribute('position').count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  return out;
}

function Trays({ scenario }: { scenario: PickPlaceScenario }) {
  const mesh = useRef<THREE.InstancedMesh>(null);
  const geometry = useMemo(() => trayGeometry(), []);
  useFrame(() => {
    const m = mesh.current;
    if (!m) return;
    let n = 0;
    for (const t of scenario.trays) {
      if (n >= MAX_TRAYS) break;
      tmpObj.position.set(t.pos.x, t.pos.y, t.pos.z);
      tmpObj.rotation.set(0, 0, 0);
      tmpObj.updateMatrix();
      m.setMatrixAt(n, tmpObj.matrix);
      const full = t.slots.every((s) => s !== null);
      m.setColorAt(n, tmpColor.set(full ? '#1e6f5c' : '#3b4a5a'));
      n++;
    }
    m.count = n;
    m.instanceMatrix.needsUpdate = true;
    if (m.instanceColor) m.instanceColor.needsUpdate = true;
  });
  return (
    <instancedMesh
      ref={mesh}
      args={[geometry, undefined, MAX_TRAYS]}
      castShadow
      receiveShadow
      frustumCulled={false}
    >
      <meshStandardMaterial roughness={0.6} flatShading />
    </instancedMesh>
  );
}
