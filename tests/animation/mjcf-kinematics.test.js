import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { DOMParser } from 'linkedom';
import { Vector3, Quaternion } from 'three';
import { MJCFAdapter } from '../../src/adapters/MJCFAdapter.js';
import { ConstraintManager } from '../../src/renderer/ConstraintManager.js';
import { MujocoSimulationManager } from '../../src/renderer/MujocoSimulationManager.js';
import { PoseController } from '../../src/animation/runtime/PoseController.js';

globalThis.DOMParser = DOMParser;
globalThis.XMLSerializer = class { serializeToString(doc) { return doc.toString(); } };

const close = (actual, expected, message = '', tolerance = 1e-9) => {
    assert.equal(actual.length, expected.length);
    actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < tolerance,
        `${message}: ${actual} != ${expected}`));
};
const position = (model, name) => model.getLink(name).threeObject.getWorldPosition(new Vector3()).toArray();
const parse = content => MJCFAdapter.parse(`<mujoco>${content}</mujoco>`);

test('MJCF inherits type, reference, axis and size from main and nested defaults', async () => {
    const model = await parse(`<compiler angle="radian"/>
      <default><joint type="slide" axis="1 0 0" ref="0.1" range="0 1"/>
        <geom type="box" size=".01 .02 .03"/>
        <default class="finger"><joint axis="0 2 0" range="0 .04"/></default>
        <default class="turn"><joint type="hinge" ref="0" range="-1 1"/></default>
      </default><worldbody><body name="base" childclass="finger"><joint name="grip"/>
        <geom/><body name="tip"><joint name="turn" class="turn"/><geom/></body>
      </body></worldbody>`);
    const grip = model.getJoint('grip');
    assert.equal(grip.type, 'prismatic');
    close(grip.axis.xyz, [0, 1, 0]);
    assert.equal(grip.ref, .1);
    assert.equal(grip.currentValue, .04, 'an out-of-range reference is clamped on import');
    assert.equal(grip.limits.upper, .04);
    assert.equal(model.getJoint('turn').type, 'revolute');
    close(model.getJoint('turn').axis.xyz, [1, 0, 0]);
    assert.deepEqual(model.getLink('base').visuals[0].geometry.size, { x: .02, y: .04, z: .06 });
});

test('degree limits and ref use radians internally; slide distances stay in meters', async () => {
    const model = await parse(`<worldbody><body name="arm"><joint name="root" range="-90 90" ref="30"/>
      <body name="tip" pos="1 0 0"><joint name="slide" type="slide" axis="1 0 0" ref=".2" range="0 .5"/></body>
      </body></worldbody>`);
    const root = model.getJoint('root');
    assert.equal(root.ref, Math.PI / 6);
    assert.equal(root.limits.upper, Math.PI / 2);
    assert.equal(model.getJoint('slide').ref, .2);
    assert.ok(root.threeObject, 'top-level moving joint must be attached to world');
    close(position(model, 'tip'), [1, 0, 0]);
    MJCFAdapter.setJointAngle(root, root.ref + Math.PI / 2);
    model.threeObject.updateMatrixWorld(true);
    close(position(model, 'tip'), [0, 1, 0]);
    MJCFAdapter.setJointAngle(root, root.ref);
    MJCFAdapter.setJointAngle(model.getJoint('slide'), .4);
    model.threeObject.updateMatrixWorld(true);
    close(position(model, 'tip'), [1.2, 0, 0]);
});

test('import clamps references to the nearest bound while keeping valid and unlimited joints unchanged', async () => {
    const model = await parse(`<worldbody><body name="arm">
      <joint name="low" ref="-60" range="-30 45"/>
      <joint name="high" type="slide" ref=".8" range=".1 .5"/>
      <joint name="valid" ref="15" range="-30 45"/>
      <joint name="unlimited" ref="90" limited="false" range="-30 45"/>
      </body></worldbody>`);
    assert.equal(model.getJoint('low').currentValue, -Math.PI / 6);
    assert.equal(model.getJoint('low').ref, -Math.PI / 3);
    assert.equal(model.getJoint('high').currentValue, .5);
    assert.equal(model.getJoint('valid').currentValue, Math.PI / 12);
    assert.equal(model.getJoint('unlimited').currentValue, Math.PI / 2);
    const referenceOnly = await MJCFAdapter.parse(`<mujoco><worldbody><body><joint name="j" type="slide" ref=".8" range="0 .5"/></body></worldbody></mujoco>`,
        null, null, { normalizeInitialPose: false });
    assert.equal(referenceOnly.getJoint('j').currentValue, .8);
});

test('unlimited hinges and explicit limited=false override inherited ranges', async () => {
    const model = await parse(`<compiler angle="radian"/><default><joint range="-1 1"/></default>
      <worldbody><body><joint name="spin" limited="false"/><body><joint name="bend"/></body></body></worldbody>`);
    assert.equal(model.getJoint('spin').type, 'continuous');
    assert.equal(model.getJoint('spin').limits, null);
    assert.equal(model.getJoint('bend').type, 'revolute');
    assert.ok(model.getJoint('spin').threeObject, 'unnamed bodies retain joint connections');
    const manual = await parse(`<compiler autolimits="false"/><worldbody><body><joint name="j" range="-90 90"/></body></worldbody>`);
    assert.equal(manual.getJoint('j').limits, null);
});

test('several joints on a body compose in XML order around their own anchors', async () => {
    const model = await parse(`<compiler angle="radian"/><worldbody><body name="arm" pos="1 0 0">
      <joint name="turn" pos=".5 0 0"/><joint name="slide" type="slide" axis="1 0 0"/>
      <body name="tip" pos="1 0 0"/></body></worldbody>`);
    MJCFAdapter.setJointAngle(model.getJoint('turn'), Math.PI / 2);
    MJCFAdapter.setJointAngle(model.getJoint('slide'), .3);
    model.threeObject.updateMatrixWorld(true);
    close(position(model, 'tip'), [1.5, .8, 0]);
    for (let i = 0; i < 20; i++) {
        MJCFAdapter.setJointAngle(model.getJoint('turn'), 0);
        MJCFAdapter.setJointAngle(model.getJoint('slide'), 0);
    }
    model.threeObject.updateMatrixWorld(true);
    close(position(model, 'tip'), [2, 0, 0]);
});

test('MJCF intrinsic, extrinsic and mixed Euler sequences respect compiler units', async () => {
    for (const sequence of ['xyz', 'XYZ', 'zYX']) {
        const model = await parse(`<compiler eulerseq="${sequence}"/><worldbody><body name="arm" euler="30 45 60"/></worldbody>`);
        const q = new Quaternion();
        for (const [i, axisName] of [...sequence].entries()) {
            const axis = new Vector3(); axis[axisName.toLowerCase()] = 1;
            const turn = new Quaternion().setFromAxisAngle(axis, [30, 45, 60][i] * Math.PI / 180);
            if (axisName === axisName.toLowerCase()) q.multiply(turn); else q.premultiply(turn);
        }
        close(model.getLink('arm').threeObject.getWorldQuaternion(new Quaternion()).toArray(), q.toArray());
    }
    for (const orientation of ['axisangle="0 0 2 90"', 'xyaxes="0 2 0 -1 0 0"']) {
        const model = await parse(`<worldbody><body name="arm" ${orientation}><body name="tip" pos="1 0 0"/></body></worldbody>`);
        close(position(model, 'tip'), [0, 1, 0]);
    }
    const model = await parse('<worldbody><body name="arm" zaxis="1 0 0"><body name="tip" pos="0 0 1"/></body></worldbody>');
    close(position(model, 'tip'), [1, 0, 0]);
});

test('plain primitive links are visible and fromto capsules align with both endpoints', async () => {
    const model = await parse(`<worldbody><body name="arm">
      <geom type="capsule" size=".02" fromto="1 2 3 2 4 6"/>
      <geom type="box" size=".1 .1 .1" group="3"/>
      </body></worldbody>`);
    const link = model.getLink('arm');
    assert.equal(link.visuals.length, 1);
    assert.equal(link.collisions.length, 1);
    const mesh = link.visuals[0].threeObject;
    const direction = new Vector3(0, 0, 1).applyQuaternion(mesh.quaternion);
    close(direction.toArray(), new Vector3(1, 2, 3).normalize().toArray());
    close(mesh.position.toArray(), [1.5, 3, 4.5]);
});

test('nested scene includes retain the robot, floor and include wrappers', async () => {
    const files = new Map([
        ['robots/arm/robot.xml', { text: async () => '<mujoco><include file="parts/body.xml"/></mujoco>' }],
        ['robots/arm/parts/body.xml', { text: async () => '<mujocoinclude><worldbody><body name="arm"><joint name="root"/></body></worldbody></mujocoinclude>' }]
    ]);
    const model = await MJCFAdapter.parse('<mujoco model="scene"><include file="robot.xml"/><worldbody><geom type="plane" size="0 0 .1"/></worldbody></mujoco>', files, 'robots/arm');
    assert.equal(model.name, 'scene');
    assert.equal(model.links.size, 2);
    assert.ok(model.getJoint('root').threeObject);
    assert.equal(model.getLink('worldbody').visuals[0].geometry.infinite, true);
    await assert.rejects(MJCFAdapter.parse('<mujoco><include file="absent.xml"/></mujoco>', files), /not found/);
});

test('gripper equalities move both fingers through the pose controller and emit updates', async () => {
    const model = await parse(`<compiler angle="radian"/><worldbody><body name="hand">
      <body name="left"><joint name="left" type="slide" axis="0 1 0" ref=".01" range="0 .05"/></body>
      <body name="right"><joint name="right" type="slide" axis="0 -1 0" ref=".02" range="0 .05"/></body>
      </body></worldbody><equality><joint joint1="left" joint2="right" polycoef="0 1 0 0 0"/></equality>`);
    const controller = new PoseController({ constraintManager: new ConstraintManager(null), redraw() {} });
    controller.setModel(model);
    const updates = []; controller.subscribe(event => updates.push(event));
    controller.setJointValue('left', .03);
    close([model.getJoint('left').currentValue, model.getJoint('right').currentValue], [.03, .04], '', 1e-7);
    close([updates.find(e => e.jointName === 'right').value], [.04], '', 1e-7);
    controller.setJointValue('right', .025);
    close([model.getJoint('left').currentValue, model.getJoint('right').currentValue], [.015, .025], '', 1e-7);
});

test('physics assets resolve by full model directory, including duplicate names', async () => {
    const writes = new Map();
    const manager = new MujocoSimulationManager(null);
    manager.ensureDir = () => {};
    manager.mujoco = { FS: { writeFile: (path, data) => writes.set(path, [...data]) } };
    const file = value => ({ arrayBuffer: async () => Uint8Array.of(value).buffer });
    const files = new Map([['robots/a/assets/base.stl', file(1)], ['robots/b/assets/base.stl', file(2)]]);
    await manager.writeAssetsToVFS('<mujoco><compiler assetdir="assets"/><asset><mesh file="base.stl"/></asset></mujoco>', 'robots/b/scene.xml', files);
    assert.deepEqual([...writes], [['/working/robots/b/assets/base.stl', [2]]]);
    await manager.writeAssetsToVFS('<mujoco><compiler texturedir="assets"/><asset><texture type="cube" fileleft="base.stl" fileright="base.stl"/></asset></mujoco>', 'robots/b/scene.xml', files);
    assert.deepEqual([...writes], [['/working/robots/b/assets/base.stl', [2]]]);
    await assert.rejects(manager.writeAssetsToVFS('<mujoco><asset><mesh file="missing.stl"/></asset></mujoco>', 'scene.xml', files), /not found/);
});

test('model attachments report missing dependencies instead of importing an empty robot', async () => {
    await assert.rejects(parse(`<asset><model name="foot" file="softfoot.xml"/></asset>
      <worldbody><body><attach model="foot" body="attachment_cube" prefix="sf_"/></body></worldbody>`),
    /attachment dependency not found.*softfoot.xml/);
});

// The geometry files are deliberately omitted in these offline tests. The
// unmodified upstream XML and MuJoCo mj_forward reference cover kinematics;
// rendering and WASM compilation are checked with the downloaded full models.
class KinematicsAdapter extends MJCFAdapter {
    static async createThreeObject(model) {
        for (const link of model.links.values()) {
            link.visuals = [];
            link.collisions = [];
        }
        await super.createThreeObject(model);
    }
}

const fixtureRoot = new URL('../fixtures/mjcf/', import.meta.url);
const reference = JSON.parse(readFileSync(new URL('reference.json', fixtureRoot), 'utf8'));
for (const [folder, expected] of Object.entries(reference.models)) {
    test(`${folder}: joint definitions and every body pose match MuJoCo mj_forward`, async () => {
        const xml = readFileSync(new URL(`${folder}/${expected.file}`, fixtureRoot), 'utf8');
        assert.equal(createHash('sha256').update(xml).digest('hex'), expected.sha256);
        const model = await KinematicsAdapter.parse(xml);
        assert.equal(model.joints.size, expected.joints.length);
        for (const spec of expected.joints) {
            const joint = model.getJoint(spec.name);
            assert.equal(joint.type, spec.type, spec.name);
            close(joint.axis.xyz, spec.axis, spec.name);
            if (spec.limits) close([joint.limits.lower, joint.limits.upper], spec.limits, spec.name);
            else assert.equal(joint.limits, null);
        }
        for (const pose of expected.poses) {
            for (const [name, value] of Object.entries(pose.joints)) MJCFAdapter.setJointAngle(model.getJoint(name), value);
            model.threeObject.updateMatrixWorld(true);
            for (const [name, body] of Object.entries(pose.bodies)) {
                close(position(model, name), body.pos, name);
                const q = model.getLink(name).threeObject.getWorldQuaternion(new Quaternion());
                const [w, x, y, z] = body.quat;
                assert.ok(1 - Math.abs(q.dot(new Quaternion(x, y, z, w))) < 1e-9, name);
            }
        }
    });
}
