import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { DOMParser } from 'linkedom';
import { Vector3, Quaternion, Group } from 'three';
import { MJCFAdapter } from '../../src/adapters/MJCFAdapter.js';
import { ConstraintManager } from '../../src/renderer/ConstraintManager.js';
import { PoseController } from '../../src/animation/runtime/PoseController.js';
import { JointDragControls } from '../../src/utils/JointDragControls.js';

globalThis.DOMParser = DOMParser;
globalThis.XMLSerializer = class { serializeToString(doc) { return doc.toString(); } };
class KinematicsAdapter extends MJCFAdapter {
    static async createThreeObject(model) {
        for (const link of model.links.values()) { link.visuals = []; link.collisions = []; }
        await super.createThreeObject(model);
    }
}
const load = async (folder, file) => KinematicsAdapter.parse(readFileSync(new URL(`../fixtures/mjcf/${folder}/${file}`, import.meta.url), 'utf8'));
const controllerFor = model => {
    const controller = new PoseController({ constraintManager: new ConstraintManager(null), redraw() {} });
    controller.setModel(model);
    controller.projectConstraints();
    return controller;
};
function assertClosed(model) {
    assert.ok(model.constraintStatus.converged, JSON.stringify(model.constraintStatus));
    assert.ok(Math.hypot(...model.constraintSolver.residual()) < 1e-7);
}
function assertWithinLimits(model) {
    for (const joint of model.joints.values()) if (joint.limits && ['revolute', 'prismatic'].includes(joint.type)) {
        assert.ok(joint.currentValue >= joint.limits.lower - 1e-12
            && joint.currentValue <= joint.limits.upper + 1e-12, `${joint.name} is outside its limits: ${joint.currentValue}`);
    }
}

const sources = JSON.parse(readFileSync(new URL('../fixtures/mjcf/parallel-sources.json', import.meta.url), 'utf8'));
for (const [folder, source] of Object.entries(sources)) {
    test(`${folder}: original fixture and every equality are retained`, async () => {
        const xml = readFileSync(new URL(`../fixtures/mjcf/${folder}/${source.file}`, import.meta.url), 'utf8');
        assert.equal(createHash('sha256').update(xml).digest('hex'), source.sha256);
        const doc = new DOMParser().parseFromString(xml, 'text/xml');
        const model = await KinematicsAdapter.parse(xml);
        assert.equal(model.constraints.size, doc.querySelector('mujoco > equality').children.length);
        controllerFor(model);
        assertClosed(model);
        assertWithinLimits(model);
    });
}

const extraSweeps = [
    ['agility_cassie', 'cassie.xml', {
        'left-knee': [-.8, -.84, -.9, -.84, -.8, -Math.PI / 4],
        'right-knee': [-.8, -.9, -Math.PI / 4]
    }],
    ['robotiq_2f85_v4', '2f85.xml', {
        left_driver_joint: [.05, .2, .5, .79, .2, 0], right_driver_joint: [.1, .4, .7, 0]
    }],
    ['toddlerbot_2xc', 'toddlerbot_2xc.xml', {
        neck_pitch_act: [.05, .2, -.1, 0], neck_yaw_drive: [.1, -.2, 0],
        left_hip_yaw_drive: [.1, -.1, 0], right_hip_yaw_drive: [-.1, .1, 0],
        left_shoulder_yaw_drive: [.1, -.1, 0], right_elbow_yaw_drive: [-.1, .1, 0]
    }],
    ['iit_softfoot', 'softfoot.xml', {
        posterior_arch_to_heel: [.02, .08, -.03, 0], heel_phalanx_1_to_virtual_1_1: [.02, .1, -.04, 0]
    }]
];
for (const [folder, file, sweeps] of extraSweeps) {
    test(`${folder}: forward/reverse sweeps and reset preserve the assembly`, async () => {
        const model = await load(folder, file);
        const controller = controllerFor(model);
        const initial = controller.getPose();
        for (const [name, steps] of Object.entries(sweeps)) {
            controller.applyPose(initial, { source: 'reset', ignoreLimits: true });
            for (const value of steps) {
                assert.equal(controller.setJointValue(name, value), value, name);
                assertClosed(model);
                if (value !== initial[name]) {
                    assert.ok(model.constraintSolver.entries.some(({ joint, size }) =>
                        joint.name !== name && (size > 1 ? joint.threeObject.quaternion.angleTo(new Quaternion()) > 1e-5
                            : Math.abs(joint.currentValue - initial[joint.name]) > 1e-5)), 'a dependent coordinate must actually move');
                }
            }
        }
        controller.applyPose(initial, { source: 'reset', ignoreLimits: true });
        assertClosed(model);
        for (const [name, value] of Object.entries(initial)) assert.equal(model.getJoint(name).currentValue, value);
    });
}

test('Robotiq 2F-85: either driver keeps both four-bars closed throughout a sweep', async () => {
    const model = await load('robotiq_2f85', '2f85.xml');
    const controller = controllerFor(model);
    const resetPose = controller.getPose();
    const initial = new Map(model.constraintSolver.constraints.filter(c => c.constraint.type === 'connect')
        .map(c => [c.constraint.name, c.second.anchor.clone()]));
    for (const name of ['right_driver_joint', 'left_driver_joint']) {
        for (const value of [.05, .2, .5, .8, .5, .2, 0]) {
            assert.equal(controller.setJointValue(name, value), value);
            assertClosed(model);
            assert.ok(Math.abs(model.getJoint('right_driver_joint').currentValue - model.getJoint('left_driver_joint').currentValue) < 1e-7);
        }
    }
    for (const item of model.constraintSolver.constraints.filter(c => c.constraint.type === 'connect')) {
        assert.ok(item.second.anchor.distanceTo(initial.get(item.constraint.name)) < 1e-14, 'anchors must not be rebound after editing');
        assert.ok(item.first.anchor.distanceTo(item.second.anchor) > .01, 'two bodies require different local anchor coordinates');
    }
    controller.applyPose(resetPose, { source: 'reset', ignoreLimits: true });
    assertClosed(model);
});

test('Cassie: editing either leg preserves the pelvis, opposite leg and every fixed connection', async () => {
    for (const side of ['left', 'right']) for (const name of ['knee', 'tarsus', 'shin']) {
        const model = await load('agility_cassie', 'cassie.xml');
        const controller = controllerFor(model);
        model.threeObject.updateMatrixWorld(true);
        const opposite = side === 'left' ? 'right' : 'left';
        const unchanged = new Map([...model.links].filter(([name]) => name === 'cassie-pelvis' || name.startsWith(`${opposite}-`))
            .map(([name, link]) => [name, link.threeObject.matrixWorld.clone()]));
        const fixed = [...model.links.values()].filter(link => {
            const parent = link.userData.parentName;
            return parent && ![...model.joints.values()].some(joint => joint.child === link.name);
        }).map(link => ({ link, relative: model.getLink(link.userData.parentName).threeObject.matrixWorld.clone().invert().multiply(link.threeObject.matrixWorld) }));
        const joint = model.getJoint(`${side}-${name}`);
        const requested = joint.currentValue + .1;
        assert.equal(controller.setJointValue(joint.name, requested), requested);
        assertClosed(model);
        model.threeObject.updateMatrixWorld(true);
        for (const [name, before] of unchanged) {
            const after = model.getLink(name).threeObject.matrixWorld;
            after.elements.forEach((value, index) => assert.ok(Math.abs(value - before.elements[index]) < 1e-12, `${joint.name} moved ${name}`));
        }
        for (const { link, relative } of fixed) {
            const after = model.getLink(link.userData.parentName).threeObject.matrixWorld.clone().invert().multiply(link.threeObject.matrixWorld);
            after.elements.forEach((value, index) => assert.ok(Math.abs(value - relative.elements[index]) < 1e-12, `fixed connection changed: ${link.name}`));
        }
        for (const entry of model.constraintSolver.entries.filter(entry => entry.joint.type === 'free')) {
            assert.deepEqual(model.constraintSolver.variables.slice(entry.offset, entry.offset + entry.size), [0, 0, 0, 0, 0, 0]);
        }
    }
});

test('Cassie: import closes the out-of-range feet within limits and either foot moves through its operating range', async () => {
    for (const side of ['left', 'right']) {
        const model = await load('agility_cassie', 'cassie.xml');
        const controller = controllerFor(model);
        const initial = controller.getPose();
        const foot = model.getJoint(`${side}-foot`);
        assert.equal(foot.ref, 0, 'normalization must preserve the MJCF reference used for kinematics');
        assert.notEqual(foot.currentValue, foot.ref);
        assert.equal(model.getJoint(`${side}-knee`).currentValue, -Math.PI / 4, 'valid actuators retain their reference');
        assertWithinLimits(model);
        const unchanged = model.constraintSolver.entries.filter(e => !e.joint.name.startsWith(`${side}-`))
            .map(e => ({ entry: e, values: model.constraintSolver.variables.slice(e.offset, e.offset + e.size) }));
        for (const value of [-.8, -1, -1.5, -2, -2.4, -1.2, -.65, initial[foot.name]]) {
            assert.equal(controller.setJointValue(foot.name, value), value);
            assertClosed(model);
            assertWithinLimits(model);
            for (const { entry, values } of unchanged) {
                assert.deepEqual(model.constraintSolver.variables.slice(entry.offset, entry.offset + entry.size), values,
                    `${foot.name} moved ${entry.joint.name}`);
            }
        }
        const beforeRejected = controller.getPose();
        assert.equal(controller.setJointValue(foot.name, foot.limits.upper), beforeRejected[foot.name],
            'a scalar limit alone does not guarantee a feasible four-bar assembly');
        assert.equal(model.constraintStatus.converged, false);
        assert.deepEqual(controller.getPose(), beforeRejected);
        controller.applyPose(initial, { source: 'reset', ignoreLimits: true });
        assertClosed(model);
        assertWithinLimits(model);
        assert.deepEqual(controller.getPose(), initial);
    }
});

test('Cassie: ordinary knee motion preserves spring deflection and the knee-spring to shin transform', async () => {
    for (const side of ['left', 'right']) {
        const model = await load('agility_cassie', 'cassie.xml');
        const controller = controllerFor(model);
        const shin = model.getJoint(`${side}-shin`), heel = model.getJoint(`${side}-heel-spring`);
        assert.equal(shin.stiffness, 1500);
        assert.equal(heel.stiffness, 1250);
        model.threeObject.updateMatrixWorld(true);
        const relative = () => model.getLink(`${side}-knee-spring`).threeObject.matrixWorld.clone().invert()
            .multiply(model.getLink(`${side}-shin`).threeObject.matrixWorld);
        const before = relative();
        for (const value of [-.65, -.9, -1.2, -1.5, -2, -.9, -Math.PI / 4]) {
            assert.equal(controller.setJointValue(`${side}-knee`, value), value);
            assertClosed(model);
            assertWithinLimits(model);
            assert.equal(shin.currentValue, 0);
            assert.equal(heel.currentValue, 0);
            model.threeObject.updateMatrixWorld(true);
            relative().elements.forEach((value, index) => assert.ok(Math.abs(value - before.elements[index]) < 1e-12));
        }
        assert.equal(controller.setJointValue(shin.name, .1), .1, 'explicit spring edits remain available');
        assertClosed(model);
    }
});

test('a passive spring is released when it is required to close the mechanism', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><compiler angle="radian"/>
      <default><joint type="slide" axis="1 0 0"/></default><worldbody>
      <body name="drive"><joint name="drive"/></body>
      <body name="spring"><joint name="spring" stiffness="100" range="-1 1"/></body>
      </worldbody><equality><connect body1="drive" body2="spring" anchor="0 0 0"/></equality></mujoco>`);
    const controller = controllerFor(model);
    assert.equal(controller.setJointValue('drive', .3), .3);
    assertClosed(model);
    assert.ok(Math.abs(model.getJoint('spring').currentValue - .3) < 1e-7);
});

test('an impossible bounded initial assembly reports an import error', async () => {
    await assert.rejects(MJCFAdapter.parse(`<mujoco><worldbody><body name="arm">
      <joint name="slide" type="slide" axis="1 0 0" range="1 2"/>
      </body></worldbody><equality><connect body1="arm" anchor="0 0 0"/></equality></mujoco>`),
    /initial pose cannot satisfy joint limits and equality constraints/);
});

test('closure preserves shared scalar ancestors and unrelated mechanisms', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><compiler angle="radian"/><worldbody>
      <body name="base"><freejoint name="base_free"/><body name="carrier"><joint name="carrier"/>
      <body name="drive"><joint name="drive"/><body name="a" pos="1 0 0"/></body>
      <body name="follower"><joint name="follower"/><body name="b" pos="1 0 0"/></body>
      </body></body><body name="other"><joint name="other"/></body></worldbody>
      <equality><connect body1="a" body2="b" anchor="0 0 0"/></equality></mujoco>`);
    const controller = controllerFor(model);
    controller.setJointValue('drive', .3);
    assertClosed(model);
    assert.equal(model.getJoint('carrier').currentValue, 0);
    assert.equal(model.getJoint('other').currentValue, 0);
    assert.ok(Math.abs(model.getJoint('follower').currentValue - .3) < 1e-7);
    assert.deepEqual(model.constraintSolver.variables.slice(0, 6), [0, 0, 0, 0, 0, 0]);
});

test('Stewart platform: six site welds close while the other five actuator lengths stay fixed', async () => {
    const model = await load('stewart_platform', 'stewart_platform.xml');
    const controller = controllerFor(model);
    const lengths = Array.from({ length: 6 }, (_, i) => model.getJoint(`joint_lu_${i + 1}`).currentValue);
    const initial = model.getLink('upper').threeObject.getWorldPosition(new Vector3());
    for (const value of [.02, .05, .1, -.05, 0]) {
        controller.setJointValue('joint_lu_1', value);
        assertClosed(model);
        assert.ok(Math.abs(model.getJoint('joint_lu_1').currentValue - value) < 1e-12);
        for (let i = 1; i < 6; i++) assert.ok(Math.abs(model.getJoint(`joint_lu_${i + 1}`).currentValue - lengths[i]) < 1e-12);
        if (value !== 0) assert.ok(model.getLink('upper').threeObject.getWorldPosition(new Vector3()).distanceTo(initial) > .005);
    }
    const world = new Group(); world.rotation.x = -Math.PI / 2; world.position.set(2, 3, 4); world.add(model.threeObject); world.updateMatrixWorld(true);
    controller.setJointValue('joint_lu_1', .03);
    assertClosed(model);
});

test('closed-chain projection does not traverse visual subtrees or submit render frames', async () => {
    const model = await load('iit_softfoot', 'softfoot.xml');
    const visual = new Group();
    model.getLink('heel').threeObject.add(visual);
    let visits = 0;
    visual.updateMatrixWorld = () => { visits++; };
    visual.updateWorldMatrix = () => { visits++; };
    const controller = controllerFor(model);
    visits = 0;
    controller.setJointValue('posterior_arch_to_heel', .04);
    assertClosed(model);
    assert.equal(visits, 0, 'geometry must update in the render pass, not each Jacobian sample');
});

test('animation and reset apply all requested coordinates before projecting the closed chain', async () => {
    const model = await load('stewart_platform', 'stewart_platform.xml');
    const controller = controllerFor(model);
    const initial = controller.getPose();
    for (const value of [.03, .06, -.03]) {
        controller.applyPose(Object.fromEntries(Array.from({ length: 6 }, (_, i) => [`joint_lu_${i + 1}`, value])));
        assertClosed(model);
        for (let i = 1; i <= 6; i++) assert.ok(Math.abs(model.getJoint(`joint_lu_${i}`).currentValue - value) < 1e-12);
    }
    controller.applyPose(initial, { source: 'reset', ignoreLimits: true });
    assertClosed(model);
});

test('unreachable edits restore the complete previous pose and report failure', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><compiler angle="radian"/><worldbody>
      <body name="arm"><joint name="drive"/><body name="end" pos="1 0 0"/></body>
      </worldbody><equality><connect body1="end" anchor="0 0 0"/></equality></mujoco>`);
    const controller = controllerFor(model);
    const events = []; controller.subscribe(event => events.push(event));
    assert.equal(controller.setJointValue('drive', .5), 0);
    assert.equal(model.getJoint('drive').currentValue, 0);
    assert.equal(model.constraintStatus.converged, false);
    assert.ok(events.some(event => event.type === 'constraintsSolved' && !event.status.converged));
    assert.ok(Math.hypot(...model.constraintSolver.residual()) < 1e-7);
});

test('passive ball joints rotate without exposing a fake scalar joint control', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><compiler angle="radian"/><worldbody>
      <body name="drive"><joint name="drive" axis="0 0 1"/><body name="target" pos="1 0 0"/></body>
      <body name="rod"><joint name="ball" type="ball"/><body name="end" pos="1 0 0"/></body>
      </worldbody><equality><connect body1="target" body2="end" anchor="0 0 0"/></equality></mujoco>`);
    const controller = controllerFor(model);
    controller.setJointValue('drive', .4);
    assertClosed(model);
    assert.equal(controller.getPose().ball, undefined);
    assert.ok(model.getJoint('ball').threeObject.quaternion.angleTo(new Quaternion()) > .3);
});

test('inactive constraints are skipped and nonlinear joint equalities respect reference offsets', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><compiler angle="radian"/><worldbody>
      <body name="a"><joint name="a" type="slide" ref=".1"/></body>
      <body name="b"><joint name="b" type="slide" ref=".2"/></body>
      </worldbody><equality><joint joint1="a" joint2="b" polycoef="0 0 2 0 0"/>
      <connect body1="a" anchor="1 0 0" active="false"/></equality></mujoco>`);
    const controller = controllerFor(model);
    controller.setJointValue('b', .5);
    assertClosed(model);
    assert.ok(Math.abs(model.getJoint('a').currentValue - .28) < 1e-7);
});

test('viewport dragging finds MJCF joints through pivot and fixed-body groups', async () => {
    const model = await load('robotiq_2f85', '2f85.xml');
    const controls = new JointDragControls(null, null, null, model);
    controls.hovered = model.getLink('right_pad');
    controls.onDragStart = () => {};
    controls.setGrabbed(true);
    assert.equal(controls.manipulating.name, 'right_follower_joint');
});

test('body weld relpose enforces both anchor location and relative rotation', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><compiler angle="radian"/><worldbody>
      <body name="drive"><joint name="drive" axis="0 0 1"/></body>
      <body name="platform" pos="1 0 0"><freejoint name="free"/></body>
      </worldbody><equality><weld body1="drive" body2="platform" anchor=".2 .1 0"
      relpose="1 0 0 1 0 0 0"/></equality></mujoco>`);
    const controller = controllerFor(model);
    controller.setJointValue('drive', .3);
    assertClosed(model);
    const first = model.getLink('drive').threeObject;
    const second = model.getLink('platform').threeObject;
    const anchor1 = first.localToWorld(new Vector3(1, 0, 0));
    const anchor2 = second.localToWorld(new Vector3(.2, .1, 0));
    assert.ok(anchor1.distanceTo(anchor2) < 1e-7);
    assert.ok(first.getWorldQuaternion(new Quaternion()).angleTo(second.getWorldQuaternion(new Quaternion())) < 1e-6);
});

test('unsupported equalities remain visible as unsupported instead of being silently discarded', async () => {
    const model = await MJCFAdapter.parse(`<mujoco><worldbody><body name="body"><joint name="j"/></body></worldbody>
      <equality><tendon tendon1="unimplemented"/></equality></mujoco>`);
    controllerFor(model);
    assert.deepEqual(model.constraintStatus.unsupported, ['tendon']);
});
