import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { LEGS, type DeltaParams } from '@/core/kinematics/params';
import { legAxis, poseFromJoints, type Joints } from '@/core/kinematics/kinematics';
import type { Vec3 } from '@/core/math/vec3';
import { PALETTE, placeBetween, v3 } from '../helpers';

export type ToolKind = 'suction' | 'fingers';

export interface RobotSnapshot {
  params: DeltaParams;
  /** World position of the base centre */
  base: Vec3;
  theta: Joints;
  /** Flange (effector plate) position in the robot frame */
  flange: Vec3;
  yaw: number;
  /** 0 open .. 1 closed */
  gripper: number;
  tool: number;
}

const ROD_SPACING = 0.075;
const tmpA = new THREE.Vector3();
const tmpB = new THREE.Vector3();
const tmpC = new THREE.Vector3();

/**
 * Procedural low-poly delta robot. Geometry is built once; each frame the parts are placed from the
 * current joint angles, so it always matches the kinematics exactly.
 */
export function DeltaRobotModel({ read, tool }: { read: () => RobotSnapshot | null; tool: ToolKind }) {
  const root = useRef<THREE.Group>(null);
  const upper = useRef<(THREE.Mesh | null)[]>([]);
  const elbowBars = useRef<(THREE.Mesh | null)[]>([]);
  const rods = useRef<(THREE.Mesh | null)[]>([]);
  const effector = useRef<THREE.Group>(null);
  const fingers = useRef<(THREE.Mesh | null)[]>([]);

  const materials = useMemo(
    () => ({
      white: new THREE.MeshStandardMaterial({
        color: PALETTE.white,
        roughness: 0.45,
        metalness: 0.1,
        flatShading: true,
      }),
      graphite: new THREE.MeshStandardMaterial({
        color: PALETTE.graphite,
        roughness: 0.55,
        metalness: 0.4,
        flatShading: true,
      }),
      accent: new THREE.MeshStandardMaterial({
        color: PALETTE.accent,
        roughness: 0.4,
        metalness: 0.15,
        flatShading: true,
      }),
      carbon: new THREE.MeshStandardMaterial({ color: '#39424e', roughness: 0.3, metalness: 0.5 }),
      chrome: new THREE.MeshStandardMaterial({ color: '#c9d1da', roughness: 0.2, metalness: 0.9 }),
      rubber: new THREE.MeshStandardMaterial({ color: '#111418', roughness: 0.9 }),
    }),
    [],
  );
  const geo = useMemo(
    () => ({
      box: new THREE.BoxGeometry(1, 1, 1),
      cyl: new THREE.CylinderGeometry(1, 1, 1, 10),
      rod: new THREE.CylinderGeometry(1, 1, 1, 6),
    }),
    [],
  );

  useFrame(() => {
    const s = read();
    const g = root.current;
    if (!s || !g) return;
    g.position.set(s.base.x, s.base.y, s.base.z);
    const pose = poseFromJoints(s.params, s.theta, s.flange);
    for (const i of LEGS) {
      const axis = v3(legAxis(i), tmpC);
      const b = v3(pose.base[i], tmpA);
      const e = v3(pose.elbows[i], tmpB);
      const arm = upper.current[i];
      if (arm) placeBetween(arm, b, e, 0.05, 0.035);
      const bar = elbowBars.current[i];
      if (bar) {
        const a = e.clone().addScaledVector(axis, -ROD_SPACING / 2 - 0.012);
        const c = e.clone().addScaledVector(axis, ROD_SPACING / 2 + 0.012);
        placeBetween(bar, a, c, 0.018);
      }
      for (const side of [-1, 1]) {
        const rod = rods.current[i * 2 + (side > 0 ? 1 : 0)];
        if (!rod) continue;
        const from = e.clone().addScaledVector(axis, (side * ROD_SPACING) / 2);
        const to = v3(pose.effectorJoints[i]).addScaledVector(axis, (side * ROD_SPACING) / 2);
        placeBetween(rod, from, to, 0.009);
      }
    }
    const eff = effector.current;
    if (eff) {
      eff.position.set(s.flange.x, s.flange.y, s.flange.z);
      eff.rotation.set(0, 0, s.yaw);
    }
    const gap = 0.012 + (1 - s.gripper) * 0.035;
    fingers.current.forEach((f, k) => f && (f.position.x = (k === 0 ? -1 : 1) * gap));
  });

  return (
    <group ref={root}>
      {/* base plate with motor housings */}
      <ReadOnlyBase materials={materials} geo={geo} read={read} />
      {LEGS.map((i) => (
        <group key={i}>
          <mesh
            ref={(m) => {
              upper.current[i] = m;
            }}
            geometry={geo.box}
            material={materials.accent}
            castShadow
          />
          <mesh
            ref={(m) => {
              elbowBars.current[i] = m;
            }}
            geometry={geo.cyl}
            material={materials.chrome}
            castShadow
          />
          {[0, 1].map((k) => (
            <mesh
              key={k}
              ref={(m) => {
                rods.current[i * 2 + k] = m;
              }}
              geometry={geo.rod}
              material={materials.carbon}
              castShadow
            />
          ))}
        </group>
      ))}
      <group ref={effector}>
        <mesh
          geometry={geo.cyl}
          material={materials.white}
          scale={[0.075, 0.022, 0.075]}
          rotation-x={Math.PI / 2}
          castShadow
        />
        <Tool kind={tool} geo={geo} materials={materials} fingers={fingers} />
      </group>
    </group>
  );
}

type Materials = Record<'white' | 'graphite' | 'accent' | 'carbon' | 'chrome' | 'rubber', THREE.Material>;
type Geo = Record<'box' | 'cyl' | 'rod', THREE.BufferGeometry>;

function ReadOnlyBase({
  materials,
  geo,
  read,
}: {
  materials: Materials;
  geo: Geo;
  read: () => RobotSnapshot | null;
}) {
  const s = read();
  if (!s) return null;
  const R = s.params.baseRadius;
  return (
    <group>
      <mesh
        geometry={geo.cyl}
        material={materials.white}
        scale={[R + 0.1, 0.05, R + 0.1]}
        rotation-x={Math.PI / 2}
        position-z={0.04}
        castShadow
        receiveShadow
      />
      <mesh
        geometry={geo.cyl}
        material={materials.graphite}
        scale={[R * 0.55, 0.12, R * 0.55]}
        rotation-x={Math.PI / 2}
        position-z={0.12}
        castShadow
      />
      {LEGS.map((i) => {
        const a = (i * 2 * Math.PI) / 3;
        return (
          <group key={i} rotation-z={a}>
            {/* motor + gearbox along the joint axis */}
            <mesh
              geometry={geo.cyl}
              material={materials.graphite}
              position={[R, 0, 0]}
              scale={[0.055, 0.16, 0.055]}
              castShadow
            />
            <mesh
              geometry={geo.cyl}
              material={materials.graphite}
              position={[R, 0.14, 0]}
              scale={[0.045, 0.12, 0.045]}
              castShadow
            />
            <mesh
              geometry={geo.cyl}
              material={materials.accent}
              position={[R, 0, 0]}
              scale={[0.058, 0.02, 0.058]}
            />
          </group>
        );
      })}
    </group>
  );
}

function Tool({
  kind,
  geo,
  materials,
  fingers,
}: {
  kind: ToolKind;
  geo: Geo;
  materials: Materials;
  fingers: React.RefObject<(THREE.Mesh | null)[]>;
}) {
  if (kind === 'suction') {
    return (
      <group>
        <mesh
          geometry={geo.cyl}
          material={materials.graphite}
          position-z={-0.035}
          scale={[0.02, 0.05, 0.02]}
          rotation-x={Math.PI / 2}
          castShadow
        />
        {[0, 1, 2].map((k) => (
          <mesh
            key={k}
            geometry={geo.cyl}
            material={materials.rubber}
            position-z={-0.065 - k * 0.011}
            scale={[0.024, 0.009, 0.024]}
            rotation-x={Math.PI / 2}
          />
        ))}
        <mesh position-z={-0.094} rotation-x={-Math.PI / 2} material={materials.rubber}>
          <cylinderGeometry args={[0.018, 0.03, 0.012, 14]} />
        </mesh>
      </group>
    );
  }
  return (
    <group>
      <mesh
        geometry={geo.box}
        material={materials.graphite}
        position-z={-0.03}
        scale={[0.09, 0.035, 0.04]}
        castShadow
      />
      {[0, 1].map((k) => (
        <mesh
          key={k}
          ref={(m) => {
            fingers.current[k] = m;
          }}
          geometry={geo.box}
          material={materials.accent}
          position-z={-0.085}
          scale={[0.01, 0.03, 0.075]}
          castShadow
        />
      ))}
    </group>
  );
}
