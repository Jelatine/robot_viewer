// Share one lazily loaded runtime between attachment import and simulation.
let runtime;

export function getMujocoRuntime() {
    if (!runtime) {
        runtime = import('mujoco-js/dist/mujoco_wasm.js').then(async ({ default: load }) => {
            const mujoco = await load();
            mujoco.FS.mkdir('/working');
            return mujoco;
        }).catch(error => {
            runtime = null;
            throw error;
        });
    }
    return runtime;
}
