import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { PivotControls } from '@react-three/drei';
import * as THREE from 'three';
import { PRESETS } from '@/core/kinematics/params';
import { vec3 } from '@/core/math/vec3';
import { MANUAL_BASE, useManual } from '@/state/manual';
import { DeltaRobotModel, type RobotSnapshot } from '../robot/DeltaRobotModel';
import { WorkspaceEnvelope } from '../robot/WorkspaceEnvelope';
import { PALETTE } from '../helpers';

const frameMat = new THREE.MeshStandardMaterial({
  color: PALETTE.frame,
  roughness: 0.35,
  metalness: 0.7,
  flatShading: true,
});

export function ManualScene() {
  const preset = useManual((s) => s.preset);
  const mode = useManual((s) => s.mode);
  const gizmoKey = useManual((s) => s.gizmoKey);
  const params = PRESETS[preset];
  const start = useMemo(() => useManual.getState().flange, [gizmoKey]); // eslint-disable-line react-hooks/exhaustive-deps

  const read = (): RobotSnapshot => {
    const s = useManual.getState();
    return { params, base: MANUAL_BASE, theta: s.theta, flange: s.flange, yaw: 0, gripper: 0, tool: 0.1 };
  };

  const ghost = useRef<THREE.Mesh>(null);
  const link = useRef<THREE.Line>(null);
  useFrame(() => {
    const s = useManual.getState();
    const g = ghost.current;
    if (g) {
      g.position.set(s.target.x, s.target.y, s.target.z);
      g.visible = !s.status.ok;
    }
    const l = link.current;
    if (l) {
      const attr = l.geometry.getAttribute('position') as THREE.BufferAttribute;
      attr.setXYZ(0, s.target.x, s.target.y, s.target.z);
      attr.setXYZ(1, s.flange.x, s.flange.y, s.flange.z);
      attr.needsUpdate = true;
      l.computeLineDistances();
      l.visible = !s.status.ok;
    }
  });

  const lineObj = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    return new THREE.Line(
      g,
      new THREE.LineDashedMaterial({ color: '#fb7185', dashSize: 0.02, gapSize: 0.015 }),
    );
  }, []);

  const top = MANUAL_BASE.z + 0.12;
  return (
    <group>
      {/* simple stand */}
      {[
        [-0.6, -0.6],
        [0.6, -0.6],
        [-0.6, 0.6],
        [0.6, 0.6],
      ].map(([x, y], i) => (
        <mesh key={i} position={[x!, y!, top / 2]} material={frameMat} castShadow>
          <boxGeometry args={[0.06, 0.06, top]} />
        </mesh>
      ))}
      {[-0.6, 0.6].map((y) => (
        <mesh key={y} position={[0, y, top]} material={frameMat} castShadow>
          <boxGeometry args={[1.26, 0.06, 0.06]} />
        </mesh>
      ))}
      {[-0.3, 0.3].map((x) => (
        <mesh key={x} position={[x, 0, top]} material={frameMat} castShadow>
          <boxGeometry args={[0.06, 1.26, 0.06]} />
        </mesh>
      ))}
      <DeltaRobotModel read={read} tool="suction" />
      <group position={[MANUAL_BASE.x, MANUAL_BASE.y, MANUAL_BASE.z]}>
        <WorkspaceEnvelope params={params} tool={0} />
        <mesh ref={ghost}>
          <sphereGeometry args={[0.03, 16, 12]} />
          <meshBasicMaterial color="#fb7185" transparent opacity={0.6} />
        </mesh>
        <primitive object={lineObj} ref={link} />
        {mode === 'ik' && (
          <PivotControls
            key={gizmoKey}
            offset={[start.x, start.y, start.z]}
            disableRotations
            disableScaling
            depthTest={false}
            scale={0.35}
            lineWidth={3}
            axisColors={['#fb7185', '#34d399', '#38bdf8']}
            onDrag={(local) => {
              const p = new THREE.Vector3().setFromMatrixPosition(local);
              useManual.getState().setTarget(vec3(start.x + p.x, start.y + p.y, start.z + p.z));
            }}
          />
        )}
      </group>
    </group>
  );
}
