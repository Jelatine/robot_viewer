import test from 'node:test';
import assert from 'node:assert/strict';
import { DOMParser } from 'linkedom';
import { Vector3 } from 'three';
import { MJCFAdapter } from '../../src/adapters/MJCFAdapter.js';
import { PoseController } from '../../src/animation/runtime/PoseController.js';
import { ConstraintManager } from '../../src/renderer/ConstraintManager.js';
import { getMujocoRuntime } from '../../src/utils/MujocoRuntime.js';

globalThis.DOMParser = DOMParser;
globalThis.XMLSerializer = class { serializeToString(doc) { return doc.toString(); } };
class KinematicsAdapter extends MJCFAdapter {
    static async createThreeObject(model) {
        for (const link of model.links.values()) { link.visuals = []; link.collisions = []; }
        await super.createThreeObject(model);
    }
}
const files = entries => new Map(Object.entries(entries).map(([name, content]) => [name, new File([content], name.split('/').pop())]));
const close = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-12, `${actual} != ${expected}`);
const hand = `<mujoco><compiler angle="radian"/>
  <default><joint type="slide" axis="0 1 0" range="-.2 .4"/>
    <geom type="box" size=".01 .02 .03" mass=".1"/></default>
  <worldbody><body name="hand" pos=".123456789012345 0 0">
    <body name="a"><joint name="drive"/><geom/><site name="tip" pos=".012345678901234 0 0"/></body>
    <body name="b"><joint name="follow"/><geom/></body>
  </body></worldbody>
  <equality><joint name="gear" joint1="follow" joint2="drive" polycoef="0 .987654321012345 0 0 0"/></equality>
  <actuator><position name="motor" joint="drive" kp="100"/></actuator>
  <sensor><jointpos name="encoder" joint="drive"/></sensor>
  </mujoco>`;

test('repeated attachments isolate defaults, references and joint coupling per instance', async () => {
    const xml = `<mujoco><compiler angle="degree"/>
      <default><joint type="hinge" axis="1 0 0" range="-90 90"/></default>
      <asset><model name="gripper" file="hands/hand.xml"/></asset><worldbody>
      <body name="left" pos="1 0 0"><attach model="gripper" body="hand" prefix="l_"/></body>
      <body name="right" pos="-1 0 0"><attach model="gripper" body="hand" prefix="r_"/></body>
      </worldbody></mujoco>`;
    const model = await KinematicsAdapter.parse(xml, files({ 'pkg/hands/hand.xml': hand }), 'pkg');
    assert.equal(model.joints.size, 4);
    assert.equal(model.constraints.size, 2);
    for (const prefix of ['l_', 'r_']) {
        const drive = model.getJoint(`${prefix}drive`);
        assert.equal(drive.type, 'prismatic');
        assert.deepEqual(drive.axis.xyz, [0, 1, 0]);
        close(drive.limits.lower, -.2);
        close(drive.limits.upper, .4);
        assert.equal(drive.actuated, true);
        close(model.sites.get(`${prefix}tip`).origin.xyz[0], .012345678901234);
        close(model.getConstraint(`${prefix}gear`).polycoef[1], .987654321012345);
    }
    close(model.getLink('l_hand').threeObject.getWorldPosition(new Vector3()).x, 1.123456789012345);
    const controller = new PoseController({ constraintManager: new ConstraintManager(null), redraw() {} });
    controller.setModel(model);
    controller.setJointValue('l_drive', .1);
    assert.ok(model.constraintStatus.converged);
    assert.ok(Math.abs(model.getJoint('l_follow').currentValue - .0987654321012345) < 1e-7);
    assert.equal(model.getJoint('r_drive').currentValue, 0);
    assert.equal(model.getJoint('r_follow').currentValue, 0);
    const compiled = new DOMParser().parseFromString(model.userData.compiledMJCF, 'text/xml');
    assert.equal(compiled.querySelectorAll('sensor > jointpos').length, 2);
    assert.equal(compiled.querySelector('attach'), null);
});

test('nested model attachments resolve includes and dependency directories', async () => {
    const source = files({
        'pkg/hands/hand.xml': '<mujoco><include file="parts.xml"/></mujoco>',
        'pkg/hands/parts.xml': hand,
        'pkg/modules/module.xml': `<mujoco><asset><model name="hand" file="../hands/hand.xml"/></asset>
          <worldbody><body name="module" pos=".25 0 0"><attach model="hand" body="hand" prefix="inner_"/></body></worldbody></mujoco>`
    });
    const model = await KinematicsAdapter.parse(`<mujoco><asset><model name="module" file="modules/module.xml"/></asset>
      <worldbody><attach model="module" body="module" prefix="outer_"/></worldbody></mujoco>`, source, 'pkg');
    assert.ok(model.getJoint('outer_inner_drive').actuated);
    assert.equal(model.getConstraint('outer_inner_gear').joint2, 'outer_inner_drive');
    close(model.getLink('outer_inner_hand').threeObject.getWorldPosition(new Vector3()).x, .373456789012345);
});

test('attachments inside frames retain the precise body transform and scalar reference', async () => {
    const child = `<mujoco><compiler angle="radian"/><worldbody>
      <body name="arm" pos=".123456789012345 0 0"><joint name="j" ref=".123456789012345" range="-.5 .8"/>
        <geom type="sphere" size=".01" mass=".1"/></body></worldbody></mujoco>`;
    const model = await KinematicsAdapter.parse(`<mujoco><asset><model name="arm" file="arm.xml"/></asset>
      <worldbody><frame pos="1 2 3" euler="0 0 90"><attach model="arm" body="arm" prefix="a_"/></frame></worldbody></mujoco>`, files({ 'arm.xml': child }));
    const p = model.getLink('a_arm').threeObject.getWorldPosition(new Vector3());
    close(p.x, 1); close(p.y, 2.123456789012345); close(p.z, 3);
    close(model.getJoint('a_j').ref, .123456789012345);
});

test('whole-model attachment retains frame defaults and multiple top-level bodies', async () => {
    const source = hand.replace('</worldbody>', '<body name="other"><geom/></body></worldbody>');
    const model = await KinematicsAdapter.parse(`<mujoco><asset><model name="hand" file="hand.xml"/></asset>
      <worldbody><attach model="hand" prefix="full_"/></worldbody></mujoco>`, files({ 'hand.xml': source }));
    assert.ok(model.getLink('full_other'));
    assert.equal(model.getJoint('full_drive').type, 'prismatic');
    close(model.getJoint('full_drive').limits.upper, .4);
});

test('attached free and ball joints retain their frames and recompile for simulation', async () => {
    const child = `<mujoco><worldbody><body name="platform" pos=".123456789012345 0 1">
      <freejoint name="free"/><geom type="box" size=".1 .1 .1" mass="1"/>
      <body name="rod" pos="0 0 .2"><joint name="ball" type="ball" limited="true" range="0 45"/>
      <geom type="capsule" size=".01 .1" mass=".1"/></body></body></worldbody></mujoco>`;
    const model = await KinematicsAdapter.parse(`<mujoco><asset><model name="child" file="child.xml"/></asset>
      <worldbody><attach model="child" body="platform" prefix="p_"/></worldbody></mujoco>`, files({ 'child.xml': child }));
    assert.equal(model.getJoint('p_free').type, 'free');
    assert.equal(model.getJoint('p_ball').type, 'ball');
    close(model.getJoint('p_ball').limits.upper, Math.PI / 4);
    close(model.getLink('p_platform').threeObject.getWorldPosition(new Vector3()).x, .123456789012345);
    const mujoco = await getMujocoRuntime();
    const path = '/working/attachment-recompile.xml';
    mujoco.FS.writeFile(path, model.userData.compiledMJCF);
    let native;
    try {
        native = mujoco.MjModel.loadFromXML(path);
        assert.equal(native.nq, 11);
        close(native.qpos0[0], .123456789012345);
    } finally { native?.delete(); mujoco.FS.unlink(path); }
});

test('newer attachment syntax reports the runtime limitation before native parsing', async () => {
    await assert.rejects(KinematicsAdapter.parse(`<mujoco><worldbody><body name="loop">
      <attach body="loop" prefix="copy_"/></body></worldbody></mujoco>`), /self-attachment.*bundled MuJoCo/);
    await assert.rejects(KinematicsAdapter.parse(`<mujoco><worldbody>
      <attach model="child" frame="f" prefix="copy_"/></worldbody></mujoco>`), /frame selection.*bundled MuJoCo/);
});

test('missing files and cyclic model dependencies fail cleanly without retaining virtual files', async () => {
    const main = `<mujoco><asset><model name="child" file="child.xml"/></asset>
      <worldbody><attach model="child" body="body" prefix="c_"/></worldbody></mujoco>`;
    const child = `<mujoco><asset><model name="other" file="other.xml"/></asset>
      <worldbody><body name="body"><attach model="other" body="body" prefix="o_"/></body></worldbody></mujoco>`;
    await assert.rejects(KinematicsAdapter.parse(main), /dependency not found/);
    await assert.rejects(KinematicsAdapter.parse(main, files({ 'child.xml': child, 'other.xml': child.replace('other.xml', 'child.xml') })), /Cyclic MJCF attachment/);
    const mujoco = await getMujocoRuntime();
    assert.deepEqual(mujoco.FS.readdir('/').filter(name => name.startsWith('mjcf_import_')), []);
});

test('meshes with the same basename keep each model directory and full scale precision', async () => {
    const mesh = 'v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n';
    const child = `<mujoco><compiler meshdir="meshes"/><asset><mesh file="shared.obj" scale=".123456789012345 .2 .3"/></asset>
      <worldbody><body name="meshbody"><geom type="mesh" mesh="shared"/></body></worldbody></mujoco>`;
    const model = await KinematicsAdapter.parse(`<mujoco><asset><model name="a" file="a/model.xml"/><model name="b" file="b/model.xml"/></asset>
      <worldbody><attach model="a" body="meshbody" prefix="a_"/><attach model="b" body="meshbody" prefix="b_"/></worldbody></mujoco>`, files({
        'pkg/a/model.xml': child, 'pkg/b/model.xml': child,
        'pkg/a/meshes/shared.obj': mesh, 'pkg/b/meshes/shared.obj': mesh.replace('v 1 0 0', 'v 2 0 0')
    }), 'pkg');
    const doc = new DOMParser().parseFromString(model.userData.compiledMJCF, 'text/xml');
    const assets = MJCFAdapter.parseAssets(doc, null, null, 'pkg');
    assert.equal(assets.get('a_shared').path, 'pkg/a/meshes/shared.obj');
    assert.equal(assets.get('b_shared').path, 'pkg/b/meshes/shared.obj');
    close(assets.get('a_shared').scale[0], .123456789012345);
});
