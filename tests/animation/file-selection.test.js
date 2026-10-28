import test from 'node:test';
import assert from 'node:assert/strict';
import { FileHandler } from '../../src/controllers/FileHandler.js';

const handler = new FileHandler();
const model = name => ({ name, category: 'model' });

test('folder imports open robot XML before scene wrappers, regardless of file order', () => {
    for (const robot of ['softfoot.xml', 'stewart_platform.xml', 'toddlerbot_2xc.xml', 'cassie.xml', '2f85.xml']) {
        const scene = model('scene.xml'), target = model(robot);
        assert.equal(handler.selectInitialModel([scene, target]), target);
        assert.equal(handler.selectInitialModel([target, scene]), target);
    }
    const target = model('arm.xml');
    assert.equal(handler.selectInitialModel([model('scene_pos.xml'), model('arm_scene.xml'), target]), target);
});

test('canonical robot selection and scene-only or mesh-only fallbacks remain available', () => {
    const robot = model('robot.xml');
    assert.equal(handler.selectInitialModel([model('ball.xml'), model('scene.xml'), robot]), robot);
    const scene = model('scene.xml');
    assert.equal(handler.selectInitialModel([scene]), scene);
    const mesh = { name: 'part.stl', category: 'mesh' };
    assert.equal(handler.selectInitialModel([mesh, scene]), scene);
    assert.equal(handler.selectInitialModel([mesh]), mesh);
    assert.equal(handler.selectInitialModel([]), null);
});
