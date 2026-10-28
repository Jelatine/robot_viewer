import test from 'node:test';
import assert from 'node:assert/strict';
import { parseHTML } from 'linkedom';
import { Object3D, PerspectiveCamera, Scene, Vector3 } from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { SceneManager } from '../../src/renderer/SceneManager.js';
import { ViewGizmo } from '../../src/ui/ViewGizmo.js';

function createManager(document) {
    const canvas = document.createElement('canvas');
    const camera = new PerspectiveCamera(75, 1.5, 0.1, 1000);
    camera.position.set(3, 4, 5);
    const controls = new OrbitControls(camera, canvas);
    controls.target.set(0.5, -0.25, 1.5);
    controls.update();
    const manager = Object.assign(Object.create(SceneManager.prototype), {
        scene: new Scene(), camera, controls,
        currentModel: { threeObject: new Object3D() }
    });
    manager.scene.add(manager.currentModel.threeObject);
    return manager;
}

test('all six axis views follow every up-axis and preserve a panned target and zoom', () => {
    const { document } = parseHTML('<html><body></body></html>');
    for (const up of ['+X', '-X', '+Y', '-Y', '+Z', '-Z']) {
        const manager = createManager(document);
        manager.setUp(up);
        const target = manager.controls.target.clone();
        const distance = manager.camera.position.distanceTo(target);
        for (const axis of ['+X', '-X', '+Y', '-Y', '+Z', '-Z']) {
            // Start from an oblique view so the click does not toggle a view.
            manager.camera.position.copy(target).add(new Vector3(2, 3, 4).setLength(distance));
            manager.controls.update();
            assert.equal(manager.alignCameraToAxis(axis), true);
            // Exercise the real OrbitControls over subsequent application ticks.
            for (let i = 0; i < 5; i++) manager.controls.update();
            const expected = new Vector3();
            expected[axis[1].toLowerCase()] = axis[0] === '+' ? 1 : -1;
            expected.applyQuaternion(manager.world.quaternion);
            const actual = manager.camera.position.clone().sub(target).normalize();
            assert(actual.dot(expected) > 1 - 1e-10, `${up} / ${axis}: ${actual.toArray()}`);
            assert(manager.controls.target.equals(target));
            assert(Math.abs(manager.camera.position.distanceTo(target) - distance) < 1e-10);
            assert(manager.camera.up.equals(new Vector3(0, 1, 0)));

            manager.alignCameraToAxis(axis);
            const reverse = manager.camera.position.clone().sub(target).normalize();
            assert(reverse.dot(expected) < -1 + 1e-10, `${up} / ${axis}: reverse`);
        }
        manager.controls.dispose();
    }
});

test('standalone meshes use their displayed frame even after a rotated robot was loaded', () => {
    const { document } = parseHTML('<html><body></body></html>');
    const manager = createManager(document);
    manager.setUp('+Z');
    manager.scene.add(manager.currentModel.threeObject);
    manager.alignCameraToAxis('+Z');
    assert(manager.camera.position.clone().sub(manager.controls.target).normalize().z > 1 - 1e-10);
    for (const invalid of ['', 'front', 'XX', '+W', null]) {
        const position = manager.camera.position.clone();
        assert.equal(manager.alignCameraToAxis(invalid), false);
        assert(manager.camera.position.equals(position));
    }
    manager.controls.dispose();
});

test('orientation widget updates depth, active view, keyboard buttons and translated labels', () => {
    const { document, window } = parseHTML('<html><body><div id="viewer"></div></body></html>');
    const previousDocument = globalThis.document;
    const previousWindow = globalThis.window;
    globalThis.document = document;
    globalThis.window = window;
    try {
        let language = 'en';
        window.i18n = { t: key => `${language}:${key}` };
        const manager = createManager(document);
        manager.setUp('+Z');
        const gizmo = new ViewGizmo(manager, document.getElementById('viewer'));
        assert.equal(gizmo.element.querySelectorAll('button').length, 6);
        assert.equal(gizmo.element.querySelectorAll('line').length, 3);
        const x = gizmo.element.querySelector('[data-axis="+X"]');
        const negX = gizmo.element.querySelector('[data-axis="-X"]');
        x.click();
        gizmo.update();
        assert.equal(x.getAttribute('aria-pressed'), 'true');
        assert.equal(negX.getAttribute('aria-pressed'), 'false');
        assert(Number(x.style.zIndex) > Number(negX.style.zIndex));
        assert.match(x.title, /en:flipViewAxis/);
        x.click();
        gizmo.update();
        assert.equal(negX.getAttribute('aria-pressed'), 'true');
        language = 'zh';
        gizmo.refreshLanguage();
        assert.match(negX.title, /zh:alignViewAxis/);
        assert.equal(gizmo.element.getAttribute('aria-label'), 'zh:viewOrientation');
        gizmo.setVisible(false);
        assert.equal(gizmo.element.hidden, true);
        gizmo.setVisible(true);
        assert.equal(gizmo.element.hidden, false);
        manager.controls.dispose();
    } finally {
        if (previousDocument === undefined) delete globalThis.document;
        else globalThis.document = previousDocument;
        if (previousWindow === undefined) delete globalThis.window;
        else globalThis.window = previousWindow;
    }
});
