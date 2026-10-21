/**
 * Regression tests for MJCF `fromto` geoms (capsule / cylinder).
 *
 * Regression: cylinder/capsule geoms with a `fromto` attribute were rendered
 * rotated 90 degrees off, because the alignment quaternion assumed the mesh's
 * long axis was +Y while createGeometryMesh() had already rotated the geometry
 * to +Z (MJCF's native geom axis). For a robot like the Unitree G1 that meant
 * thigh/shin/hip capsules showing up horizontal and the foot capsules vertical.
 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { DOMParser } from 'linkedom';
import * as THREE from 'three';
import { MJCFAdapter } from '../../src/adapters/MJCFAdapter.js';

globalThis.DOMParser = DOMParser;

const EPS = 1e-6;

test('capsule and cylinder geometries are long along +Z', async () => {
    const capsule = await MJCFAdapter.createGeometryMesh({
        type: 'capsule',
        size: { radius: 0.05, height: 0.4 }
    });
    assert.ok(capsule, 'capsule geometry should be created');

    capsule.geometry.computeBoundingBox();
    const capsuleSize = new THREE.Vector3();
    capsule.geometry.boundingBox.getSize(capsuleSize);

    // total length = cylinder length + 2 * radius, along Z
    assert.ok(Math.abs(capsuleSize.z - 0.5) < EPS, `capsule z extent: ${capsuleSize.z}`);
    assert.ok(Math.abs(capsuleSize.x - 0.1) < EPS, `capsule x extent: ${capsuleSize.x}`);
    assert.ok(Math.abs(capsuleSize.y - 0.1) < EPS, `capsule y extent: ${capsuleSize.y}`);

    const cylinder = await MJCFAdapter.createGeometryMesh({
        type: 'cylinder',
        size: { radius: 0.05, height: 0.4 }
    });
    assert.ok(cylinder, 'cylinder geometry should be created');

    cylinder.geometry.computeBoundingBox();
    const cylinderSize = new THREE.Vector3();
    cylinder.geometry.boundingBox.getSize(cylinderSize);

    assert.ok(Math.abs(cylinderSize.z - 0.4) < EPS, `cylinder z extent: ${cylinderSize.z}`);
    assert.ok(Math.abs(cylinderSize.x - 0.1) < EPS, `cylinder x extent: ${cylinderSize.x}`);
});

test('fromto frame maps the geometry long axis onto the fromto direction', () => {
    // A sample of real geoms from the Unitree G1 MJCF (mujoco_menagerie).
    const cases = [
        [[0.02, 0, 0], [0.02, 0, -0.08]], // left_hip_collision (straight down)
        [[0, 0, -0.03], [-0.06, 0, -0.17]], // left_thigh_collision (tilted)
        [[0.01, 0, 0], [0.01, 0, -0.15]], // left_shin_collision (straight down)
        [[-0.054, 0, -0.025], [0.132, 0, -0.025]], // left_foot4_collision (forward)
        [[-0.01, 0, -0.01], [0.08, 0, -0.01]], // left_elbow_yaw_collision (forward)
        [[0.01, 0, 0.08], [0.01, 0, 0.2]], // torso_collision (straight up)
        [[0.07, 0, 0], [0.15, -0.02, 0]] // left_hand_collision (slightly sideways)
    ];

    for (const [p1, p2] of cases) {
        const from = new THREE.Vector3(...p1);
        const to = new THREE.Vector3(...p2);
        const frame = MJCFAdapter.computeFromtoFrame(p1, p2);

        // center is the midpoint of the two points
        const expectedCenter = new THREE.Vector3().addVectors(from, to).multiplyScalar(0.5);
        assert.ok(new THREE.Vector3().fromArray(frame.center).distanceTo(expectedCenter) < EPS,
            `center for fromto ${p1} -> ${p2}`);

        // height is the distance between the two points
        assert.ok(Math.abs(frame.height - from.distanceTo(to)) < EPS,
            `height for fromto ${p1} -> ${p2}`);

        // the quaternion must rotate the geom's +Z axis onto the fromto direction
        // (rotating +Y instead would be the 90 degree regression)
        const axis = new THREE.Vector3(0, 0, 1)
            .applyQuaternion(new THREE.Quaternion().fromArray(frame.quaternion));
        const direction = new THREE.Vector3().subVectors(to, from).normalize();
        const angle = THREE.MathUtils.radToDeg(axis.angleTo(direction));
        assert.ok(angle < 1e-3, `axis error for fromto ${p1} -> ${p2}: ${angle} degrees`);
    }
});

test('anti-parallel fromto still points along the fromto direction', () => {
    // setFromUnitVectors() has a special case when the two vectors are opposite
    const frame = MJCFAdapter.computeFromtoFrame([0, 0, 0], [0, 0, -0.08]);
    const axis = new THREE.Vector3(0, 0, 1)
        .applyQuaternion(new THREE.Quaternion().fromArray(frame.quaternion));
    assert.ok(axis.distanceTo(new THREE.Vector3(0, 0, -1)) < EPS);
});

test('fromto attribute parsing tolerates runs of whitespace', () => {
    assert.deepEqual(
        MJCFAdapter.parseFromto('0.01 0 0 0.01 0 -0.15'),
        [0.01, 0, 0, 0.01, 0, -0.15]
    );

    // hand formatted / pretty printed attributes
    assert.deepEqual(
        MJCFAdapter.parseFromto('\n  -0.054\t0  -0.025\n  0.132   0 -0.025  '),
        [-0.054, 0, -0.025, 0.132, 0, -0.025]
    );

    // extra values are ignored, six are required
    assert.deepEqual(
        MJCFAdapter.parseFromto('1e-3 0 0 0 0 .2 0.5'),
        [0.001, 0, 0, 0, 0, 0.2]
    );
});

test('malformed fromto attributes are rejected instead of yielding NaN', () => {
    assert.equal(MJCFAdapter.parseFromto(null), null);
    assert.equal(MJCFAdapter.parseFromto(''), null);
    assert.equal(MJCFAdapter.parseFromto('0 0 0'), null, 'too few values');
    assert.equal(MJCFAdapter.parseFromto('0 0 0 0 0'), null, 'too few values');
    assert.equal(MJCFAdapter.parseFromto('0 0 0 0 0 x'), null, 'not a number');
    assert.equal(MJCFAdapter.parseFromto('0 0 0 0 0 0.2abc'), null, 'trailing garbage');
});

test('zero length fromto produces a valid identity transform', () => {
    const frame = MJCFAdapter.computeFromtoFrame([0.1, 0.2, 0.3], [0.1, 0.2, 0.3]);

    assert.equal(frame.height, 0);
    assert.deepEqual(frame.center, [0.1, 0.2, 0.3]);
    assert.ok(frame.quaternion.every(Number.isFinite), `quaternion: ${frame.quaternion}`);
    assert.deepEqual(frame.quaternion, [0, 0, 0, 1], 'identity quaternion');

    const axis = new THREE.Vector3(0, 0, 1)
        .applyQuaternion(new THREE.Quaternion().fromArray(frame.quaternion));
    assert.ok(axis.distanceTo(new THREE.Vector3(0, 0, 1)) < EPS);
});

for (const type of ['capsule', 'cylinder']) {
    test(`${type} visual and collision meshes retain fromto endpoints after full parsing`, async () => {
        const cases = [
            [0.02, 0, 0, 0.02, 0, -0.08],
            [-0.054, 0, -0.025, 0.132, 0, -0.025],
            [0.07, 0, 0, 0.15, -0.02, 0.03],
            [0.1, 0.2, 0.3, 0.1, 0.2, 0.3]
        ];
        const geoms = cases.map((points, i) => [
            `<geom name="visual_${i}" type="${type}" size=".01" fromto="${points.join('  \t')}"
                pos="9 8 7" euler="30 45 60"/>`,
            `<geom name="collision_${i}" type="${type}" size=".01" fromto="${points.join('\n ')}"
                pos="9 8 7" euler="30 45 60" group="3"/>`
        ].join('')).join('');
        const model = await MJCFAdapter.parse(`<mujoco><worldbody>
            <body name="arm" pos="1 2 3" euler="10 20 30">${geoms}</body>
            </worldbody></mujoco>`);
        const link = model.getLink('arm');
        assert.equal(link.visuals.length, cases.length);
        assert.equal(link.collisions.length, cases.length);
        model.threeObject.updateMatrixWorld(true);

        for (const [i, points] of cases.entries()) {
            const p1 = new THREE.Vector3(...points.slice(0, 3));
            const p2 = new THREE.Vector3(...points.slice(3, 6));
            const height = p1.distanceTo(p2);
            const expected = [p1, p2].map(point => link.threeObject.localToWorld(point));
            const meshes = [link.visuals[i].threeObject, link.collisions[i].threeObject.children[0]];
            for (const mesh of meshes) {
                for (const [end, z] of [-height / 2, height / 2].entries()) {
                    const actual = mesh.localToWorld(new THREE.Vector3(0, 0, z));
                    assert.ok(actual.distanceTo(expected[end]) < EPS,
                        `${mesh.name} endpoint ${end}: ${actual.toArray()} != ${expected[end].toArray()}`);
                }
                assert.ok(mesh.quaternion.toArray().every(Number.isFinite));
            }
        }
    });

    test(`${type} malformed fromto falls back to size and the explicit origin`, async () => {
        const model = await MJCFAdapter.parse(`<mujoco><worldbody><body name="arm">
            <geom type="${type}" size=".02 .15" fromto="0 0 0 0 0 bad" pos="1 2 3" euler="0 90 0"/>
            <geom type="${type}" size=".02 .15" pos="1 2 3" euler="0 90 0" group="3"/>
            </body></worldbody></mujoco>`);
        const link = model.getLink('arm');
        for (const geom of [...link.visuals, ...link.collisions]) {
            assert.equal(geom.geometry.fromto, undefined);
            assert.deepEqual(geom.geometry.size, { radius: .02, height: .3 });
            const mesh = geom.threeObject.isMesh ? geom.threeObject : geom.threeObject.children[0];
            assert.deepEqual(mesh.position.toArray(), [1, 2, 3]);
            const axis = new THREE.Vector3(0, 0, 1).applyQuaternion(mesh.quaternion);
            assert.ok(axis.distanceTo(new THREE.Vector3(1, 0, 0)) < EPS);
        }
    });
}
