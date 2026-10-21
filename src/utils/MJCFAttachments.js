import { Matrix4, Quaternion, Vector3 } from 'three';
import { cleanFilePath, resolveFileFromMap } from './FileUtils.js';
import { getMujocoRuntime } from './MujocoRuntime.js';

let nextImport = 0;
const serialize = doc => new XMLSerializer().serializeToString(doc);
const numbers = (values, start, count) => Array.from(values.slice(start, start + count)).join(' ');
const relativePath = (path, directory) => {
    const target = cleanFilePath(path).split('/');
    const parent = cleanFilePath(directory).split('/').filter(Boolean);
    while (parent.length && target.length && parent[0] === target[0]) { parent.shift(); target.shift(); }
    return [...parent.map(() => '..'), ...target].join('/');
};

/** MuJoCo owns attachment semantics: prefixing, defaults, references, subtree
 * selection and nested attachments. Only composed models load the compiler.
 * Its XML writer rounds numbers to six significant digits, so restore the
 * compiled double-precision frames before handing them to the pose editor.
 */
export async function compileMJCFWithAttachments(content, fileMap, basePath, adapter) {
    const mujoco = await getMujocoRuntime();
    const root = `/mjcf_import_${++nextImport}`;
    const written = new Set();
    const directories = new Set([root]);
    const prepared = new Set();
    mujoco.FS.mkdir(root);
    const write = (path, data) => {
        const parts = path.split('/');
        for (let i = 2; i < parts.length; i++) {
            const dir = parts.slice(0, i).join('/');
            if (!directories.has(dir)) { mujoco.FS.mkdir(dir); directories.add(dir); }
        }
        mujoco.FS.writeFile(path, data);
        written.add(path);
    };
    const dependency = (path, directory) => {
        const file = resolveFileFromMap(path, fileMap, { baseDir: directory });
        if (!file) throw new Error(`MJCF attachment dependency not found: ${path}`);
        const key = cleanFilePath(Array.from(fileMap).find(([, value]) => value === file)[0]);
        return { file, key, path: `${root}/${key}` };
    };
    const prepare = async (xml, directory, stack) => {
        const expanded = await adapter.processIncludes(xml, fileMap, directory);
        const doc = new DOMParser().parseFromString(expanded, 'text/xml');
        if (doc.querySelector('parsererror')) throw new Error('MJCF attachment XML parsing failed.');
        adapter.mergeTopLevelSections(doc);
        // These newer syntax variants are absent from the bundled 3.3 schema.
        // Catch them before its parser raises an opaque C++ exception.
        for (const attach of doc.querySelectorAll('attach')) {
            if (!attach.getAttribute('model')) throw new Error('MJCF self-attachment is not supported by the bundled MuJoCo runtime; use an asset model.');
            if (attach.hasAttribute('frame')) throw new Error('MJCF attachment frame selection is not supported by the bundled MuJoCo runtime; select a body or the whole model.');
            if (!attach.hasAttribute('prefix')) throw new Error('MJCF attachment requires a prefix.');
        }
        const compiler = doc.querySelector('mujoco > compiler');
        const assetdir = compiler?.getAttribute('assetdir') || '';
        const meshdir = compiler?.getAttribute('meshdir') ?? assetdir;
        const texturedir = compiler?.getAttribute('texturedir') ?? assetdir;
        for (const asset of doc.querySelectorAll('asset > *')) {
            // Texture cube faces are separate file attributes, not names.
            for (const attribute of ['file', 'fileleft', 'fileright', 'fileup', 'filedown', 'filefront', 'fileback']) {
                if (!asset.hasAttribute(attribute)) continue;
                let filename = asset.getAttribute(attribute);
                if (asset.tagName !== 'model' && compiler?.getAttribute('strippath') === 'true') filename = filename.replace(/\\/g, '/').split('/').pop();
                const assetDirectory = asset.tagName === 'model' ? '' : asset.tagName === 'texture' ? texturedir : meshdir;
                const path = adapter.resolveAssetFilePath(filename, directory, assetDirectory);
                const found = dependency(path, '');
                if (asset.tagName === 'model') {
                    if (stack.has(found.key)) throw new Error(`Cyclic MJCF attachment: ${found.key}`);
                    if (!prepared.has(found.key)) {
                        const nested = new Set(stack); nested.add(found.key);
                        const childDirectory = found.key.includes('/') ? found.key.slice(0, found.key.lastIndexOf('/')) : '';
                        write(found.path, await prepare(await found.file.text(), childDirectory, nested));
                        prepared.add(found.key);
                    }
                } else if (!written.has(found.path)) {
                    const data = found.file.arrayBuffer ? new Uint8Array(await found.file.arrayBuffer()) : await found.file.text();
                    write(found.path, data);
                }
                asset.setAttribute(attribute, found.path);
            }
        }
        for (const attribute of ['meshdir', 'texturedir', 'assetdir', 'strippath']) compiler?.removeAttribute(attribute);
        return serialize(doc);
    };
    let model;
    try {
        const source = await prepare(content, basePath || '', new Set());
        const input = `${root}/scene.xml`, output = `${root}/expanded.xml`;
        write(input, source);
        try {
            model = mujoco.MjModel.loadFromXML(input);
        } catch (error) {
            throw new Error(`MJCF attachment compilation failed: ${error.message || 'check model names, selected bodies and MJCF syntax.'}`, { cause: error });
        }
        if (!model) throw new Error('MuJoCo could not compile the attached MJCF model. Check its bodies, references and inertial properties.');
        write(output, '');
        if (!mujoco.mj_saveLastXML(output, model)) throw new Error('Could not expand MJCF attachments.');
        const doc = new DOMParser().parseFromString(mujoco.FS.readFile(output, { encoding: 'utf8' }), 'text/xml');
        // Frames may own childclass. Materialize their defaults before
        // removing frames whose transforms the compiler already accumulated.
        adapter.expandDefaults(doc);
        restoreCompiledFrames(doc, model);
        for (const asset of doc.querySelectorAll('asset > *')) for (const attribute of ['file', 'fileleft', 'fileright', 'fileup', 'filedown', 'filefront', 'fileback']) {
            const path = asset.getAttribute(attribute);
            if (path?.startsWith(`${root}/`)) asset.setAttribute(attribute, relativePath(path.slice(root.length + 1), basePath || ''));
        }
        return serialize(doc);
    } finally {
        model?.delete();
        for (const path of written) mujoco.FS.unlink(path);
        for (const path of [...directories].reverse()) mujoco.FS.rmdir(path);
    }
}

function restoreCompiledFrames(doc, model) {
    const world = doc.querySelector('worldbody');
    const bodies = [world, ...world.querySelectorAll('body')];
    if (bodies.length !== model.nbody) throw new Error('MJCF attachment body count changed during expansion.');
    const owner = element => {
        let parent = element.parentElement;
        while (parent && parent.tagName !== 'body' && parent.tagName !== 'worldbody') parent = parent.parentElement;
        return parent;
    };
    const pose = (element, pos, quat, index) => {
        for (const attribute of ['euler', 'axisangle', 'xyaxes', 'zaxis', 'fromto']) element.removeAttribute(attribute);
        element.setAttribute('pos', numbers(pos, 3 * index, 3));
        if (quat) element.setAttribute('quat', numbers(quat, 4 * index, 4));
    };
    const groups = [
        ['joint, freejoint', 'jnt_bodyid'], ['geom', 'geom_bodyid'], ['site', 'site_bodyid'],
        ['camera', 'cam_bodyid'], ['light', 'light_bodyid']
    ];
    for (let bodyId = 0; bodyId < bodies.length; bodyId++) {
        const body = bodies[bodyId];
        if (bodyId) {
            pose(body, model.body_pos, model.body_quat, bodyId);
            const inertial = Array.from(body.children).find(element => element.tagName === 'inertial');
            if (inertial) {
                pose(inertial, model.body_ipos, model.body_iquat, bodyId);
                inertial.removeAttribute('fullinertia');
                inertial.setAttribute('mass', String(model.body_mass[bodyId]));
                inertial.setAttribute('diaginertia', numbers(model.body_inertia, bodyId * 3, 3));
            }
        }
        for (const [selector, field] of groups) {
            const elements = Array.from(body.querySelectorAll(selector)).filter(element => owner(element) === body);
            const ids = Array.from(model[field]).flatMap((value, index) => value === bodyId ? [index] : []);
            if (elements.length !== ids.length) throw new Error(`MJCF attachment ${selector} count changed during expansion.`);
            elements.forEach((element, index) => {
                const id = ids[index];
                if (field === 'jnt_bodyid') {
                    element.setAttribute('pos', numbers(model.jnt_pos, id * 3, 3));
                    element.setAttribute('axis', numbers(model.jnt_axis, id * 3, 3));
                    element.setAttribute('limited', model.jnt_limited[id] ? 'true' : 'false');
                    element.setAttribute('range', numbers(model.jnt_range, id * 2, 2));
                    if (model.jnt_type[id] >= 2) element.setAttribute('ref', String(model.qpos0[model.jnt_qposadr[id]]));
                } else if (field === 'geom_bodyid') {
                    pose(element, model.geom_pos, model.geom_quat, id);
                    const mesh = model.geom_type[id] === 7 ? model.geom_dataid[id] : -1;
                    if (mesh >= 0) {
                        // Native mesh geoms include the mesh's centering/PCA
                        // transform. Three.js loads the original vertices.
                        const matrix = (pos, quat, i) => new Matrix4().compose(new Vector3(...pos.slice(i * 3, i * 3 + 3)),
                            new Quaternion(quat[i * 4 + 1], quat[i * 4 + 2], quat[i * 4 + 3], quat[i * 4]), new Vector3(1, 1, 1));
                        const transform = matrix(model.geom_pos, model.geom_quat, id).multiply(matrix(model.mesh_pos, model.mesh_quat, mesh).invert());
                        const p = new Vector3(), q = new Quaternion(); transform.decompose(p, q, new Vector3());
                        element.setAttribute('pos', p.toArray().join(' '));
                        element.setAttribute('quat', [q.w, q.x, q.y, q.z].join(' '));
                    } else {
                        const count = [3, 3, 1, 2, 3, 2, 3][model.geom_type[id]] || 3;
                        element.setAttribute('size', numbers(model.geom_size, id * 3, count));
                    }
                } else if (field === 'site_bodyid') {
                    pose(element, model.site_pos, model.site_quat, id);
                    element.setAttribute('size', numbers(model.site_size, id * 3, 3));
                }
                else if (field === 'cam_bodyid') pose(element, model.cam_pos, model.cam_quat, id);
                else {
                    pose(element, model.light_pos, null, id);
                    element.setAttribute('dir', numbers(model.light_dir, id * 3, 3));
                }
            });
        }
    }
    // Compiled poses are already expressed in the enclosing body's frame.
    for (const frame of Array.from(world.querySelectorAll('frame')).reverse()) {
        while (frame.firstChild) frame.parentNode.insertBefore(frame.firstChild, frame);
        frame.remove();
    }
    Array.from(doc.querySelectorAll('asset > mesh')).forEach((element, id) => {
        element.setAttribute('scale', numbers(model.mesh_scale, id * 3, 3));
    });
    Array.from(doc.querySelector('mujoco > equality')?.children || []).forEach((element, id) => {
        if (element.tagName === 'joint') element.setAttribute('polycoef', numbers(model.eq_data, id * 11, 5));
        if (element.hasAttribute('site1')) return;
        if (element.tagName === 'connect' || element.tagName === 'weld') element.setAttribute('anchor', numbers(model.eq_data, id * 11, 3));
        if (element.tagName === 'weld') {
            element.setAttribute('relpose', numbers(model.eq_data, id * 11 + 3, 7));
            element.setAttribute('torquescale', String(model.eq_data[id * 11 + 10]));
        }
    });
}
