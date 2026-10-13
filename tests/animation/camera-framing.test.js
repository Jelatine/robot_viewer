import test from 'node:test';
import assert from 'node:assert/strict';
import { Box3, PerspectiveCamera, Vector3 } from 'three';
import { SceneManager } from '../../src/renderer/SceneManager.js';

function createManager(aspect) {
    const camera = new PerspectiveCamera(75, aspect, 0.1, 1000);
    const target = new Vector3(0.5, 1, -0.5);
    return Object.assign(Object.create(SceneManager.prototype), {
        camera,
        currentModel: { joints: new Map([['joint', {}]]) },
        controls: {
            target,
            update() {
                camera.lookAt(target);
                camera.updateMatrixWorld();
            }
        },
        renderer: { setSize() {}, setPixelRatio() {} },
        render() { camera.updateMatrixWorld(); }
    });
}

function assertBoundsVisible(manager, box) {
    for (const x of [box.min.x, box.max.x]) {
        for (const y of [box.min.y, box.max.y]) {
            for (const z of [box.min.z, box.max.z]) {
                const point = new Vector3(x, y, z).project(manager.camera);
                assert(Math.abs(point.x) <= 1 && Math.abs(point.y) <= 1,
                    `Model corner is outside viewport at aspect ${manager.camera.aspect}: ${point.toArray()}`);
            }
        }
    }
}

test('camera fits wide, tall and deep models in phone and desktop viewports', () => {
    for (const aspect of [0.38, 0.46, 1, 2.2]) {
        for (const size of [new Vector3(4, 1, 1), new Vector3(1, 4, 1), new Vector3(1, 1, 4)]) {
            const manager = createManager(aspect);
            const center = manager.controls.target.clone();
            const box = new Box3().setFromCenterAndSize(center, size);
            manager.fitCameraToModel(box, center, size);
            assertBoundsVisible(manager, box);
        }
    }
});

test('drawer and orientation resizing keeps the model visible and restores the camera', () => {
    const previousWindow = globalThis.window;
    globalThis.window = { devicePixelRatio: 2 };
    try {
        const manager = createManager(390 / 320);
        const center = manager.controls.target.clone();
        const size = new Vector3(4, 1, 1);
        const box = new Box3().setFromCenterAndSize(center, size);
        manager.fitCameraToModel(box, center, size);
        const originalPosition = manager.camera.position.clone();
        for (const [width, height] of [[390, 720], [844, 260], [390, 320]]) {
            manager.handleResize(width, height);
            assertBoundsVisible(manager, box);
            assert(manager.controls.target.equals(center));
        }
        assert(manager.camera.position.distanceTo(originalPosition) < 1e-10);
    } finally {
        if (previousWindow === undefined) delete globalThis.window;
        else globalThis.window = previousWindow;
    }
});
