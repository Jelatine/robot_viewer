import * as THREE from 'three';
import { isScalarJoint } from '../models/UnifiedRobotModel.js';

const rotation = values => {
    const axis = new THREE.Vector3(...values);
    const angle = axis.length();
    return angle < 1e-15 ? new THREE.Quaternion() : new THREE.Quaternion().setFromAxisAngle(axis.divideScalar(angle), angle);
};
const norm = values => Math.hypot(...values);

// Partial-pivot Gaussian elimination for the small damped normal system.
function linearSolve(matrix, rhs) {
    const n = rhs.length;
    const rows = matrix.map((row, i) => [...row, rhs[i]]);
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k + 1; i < n; i++) if (Math.abs(rows[i][k]) > Math.abs(rows[pivot][k])) pivot = i;
        if (Math.abs(rows[pivot][k]) < 1e-18) return null;
        [rows[k], rows[pivot]] = [rows[pivot], rows[k]];
        for (let i = k + 1; i < n; i++) {
            const factor = rows[i][k] / rows[k][k];
            for (let j = k; j <= n; j++) rows[i][j] -= factor * rows[k][j];
        }
    }
    const result = Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
        let value = rows[i][n];
        for (let j = i + 1; j < n; j++) value -= rows[i][j] * result[j];
        result[i] = value / rows[i][i];
    }
    return result;
}

/** Position-level equality projection for the interactive MJCF pose editor.
 * Bind body anchors at qpos0, before any pose edit or scene-axis conversion.
 * Solve all active connect/weld/joint equations together; physics remains
 * MuJoCo's responsibility. No rendering or DOM work belongs in this solver.
 */
export class ClosedChainSolver {
    constructor(model, setScalar) {
        this.model = model;
        this.setScalar = setScalar;
        this.variables = [];
        this.entries = [];
        this.tolerance = 1e-7;
        this.lastValid = null;
        model.threeObject.updateMatrixWorld(true);
        for (const joint of model.joints.values()) {
            if (!joint.threeObject) continue;
            if (isScalarJoint(joint)) {
                this.entries.push({ joint, offset: this.variables.length, size: 1 });
                this.variables.push(joint.currentValue);
            } else if (joint.type === 'ball' || joint.type === 'free') {
                const size = joint.type === 'free' ? 6 : 3;
                this.entries.push({ joint, offset: this.variables.length, size,
                    position: joint.threeObject.position.clone(), quaternion: joint.threeObject.quaternion.clone() });
                this.variables.push(...Array(size).fill(0));
            }
        }
        this.constraints = [];
        this.unsupported = [];
        for (const constraint of model.constraints.values()) {
            if (constraint.active === false) continue;
            if (constraint.type === 'joint') {
                const first = model.getJoint(constraint.joint1);
                const second = model.getJoint(constraint.joint2);
                if (!isScalarJoint(first) || (constraint.joint2 && !isScalarJoint(second))) {
                    throw new Error(`Invalid joint equality: ${constraint.name}`);
                }
                this.constraints.push({ constraint, first, second });
            } else if (constraint.type === 'connect' || constraint.type === 'weld') {
                this.constraints.push(this.bindBodyConstraint(constraint));
            } else {
                this.unsupported.push(constraint.type);
            }
        }
        const entryByObject = new Map(this.entries.map(entry => [entry.joint.threeObject, entry]));
        const entryByJoint = new Map(this.entries.map(entry => [entry.joint, entry]));
        const ancestors = object => {
            const result = new Set();
            for (let node = object; node && node !== model.threeObject; node = node.parent) {
                const entry = entryByObject.get(node);
                if (entry) result.add(entry);
            }
            return result;
        };
        this.constraintEntries = new Set();
        for (const item of this.constraints) {
            if (item.constraint.type === 'joint') {
                item.entries = new Set([entryByJoint.get(item.first), entryByJoint.get(item.second)].filter(Boolean));
            } else {
                const first = ancestors(item.first.object), second = ancestors(item.second.object);
                // Moving a common ancestor carries both closure endpoints
                // together. Its motion cannot close the chain, and a numerical
                // Jacobian can otherwise spend those redundant DOFs drifting
                // Cassie's pelvis or a joint above the entire mechanism.
                item.entries = new Set([...first].filter(entry => !second.has(entry))
                    .concat([...second].filter(entry => !first.has(entry))));
            }
            for (const entry of item.entries) this.constraintEntries.add(entry);
        }
        // Projection needs only joint/constraint frames. Geometry, collision,
        // axis and inertia helpers are updated once by the next render.
        const required = new Set();
        const include = object => {
            while (object && object !== model.threeObject && !required.has(object)) {
                required.add(object); object = object.parent;
            }
        };
        for (const entry of this.entries) include(entry.joint.threeObject);
        for (const item of this.constraints) if (item.first.object) { include(item.first.object); include(item.second.object); }
        this.frames = [];
        const collect = object => {
            for (const child of object.children) if (required.has(child)) { this.frames.push(child); collect(child); }
        };
        collect(model.threeObject);
        const residual = norm(this.residual());
        if (residual <= this.tolerance) this.lastValid = [...this.variables];
        this.status = { converged: residual <= this.tolerance, residual, iterations: 0, unsupported: this.unsupported };
        model.constraintStatus = this.status;
    }

    body(name) {
        const object = this.model.getLink(name || 'worldbody')?.threeObject;
        if (!object) throw new Error(`Equality body not found: ${name}`);
        return object;
    }

    bindBodyConstraint(constraint) {
        const siteFrame = name => {
            const site = this.model.sites.get(name);
            if (!site) throw new Error(`Equality site not found: ${name}`);
            const q = site.origin.quat;
            return { object: this.body(site.body), anchor: new THREE.Vector3(...site.origin.xyz),
                quaternion: q ? new THREE.Quaternion(q.x, q.y, q.z, q.w) : new THREE.Quaternion() };
        };
        if (constraint.site1 || constraint.site2) {
            return { constraint, first: siteFrame(constraint.site1), second: siteFrame(constraint.site2), relative: new THREE.Quaternion() };
        }
        const first = { object: this.body(constraint.body1), anchor: new THREE.Vector3(), quaternion: new THREE.Quaternion() };
        const second = { object: this.body(constraint.body2), anchor: new THREE.Vector3(), quaternion: new THREE.Quaternion() };
        if (constraint.type === 'connect') {
            first.anchor.set(...(constraint.anchor || [0, 0, 0]));
            second.anchor.copy(first.anchor);
            first.object.localToWorld(second.anchor);
            second.object.worldToLocal(second.anchor);
        } else {
            second.anchor.set(...(constraint.anchor || [0, 0, 0]));
            first.anchor.copy(second.anchor);
            second.object.localToWorld(first.anchor);
            first.object.worldToLocal(first.anchor);
        }
        const relative = first.object.getWorldQuaternion(new THREE.Quaternion()).invert()
            .multiply(second.object.getWorldQuaternion(new THREE.Quaternion()));
        if (constraint.type === 'weld' && constraint.relpose && norm(constraint.relpose.slice(3)) > 0) {
            first.anchor.set(...constraint.relpose.slice(0, 3));
            const [w, x, y, z] = constraint.relpose.slice(3);
            relative.set(x, y, z, w).normalize();
        }
        return { constraint, first, second, relative };
    }

    // Keep residuals in model coordinates, independent of the viewer's +Z/+Y
    // selection and its position in the scene.
    residual() {
        const root = this.model.threeObject;
        this.updateFrames();
        const inverse = root.matrixWorld.clone().invert();
        const result = [];
        for (const item of this.constraints) {
            const { constraint, first, second } = item;
            if (constraint.type === 'joint') {
                const x = second ? second.currentValue - (second.ref || 0) : 0;
                const target = (constraint.polycoef || [0, 1]).reduceRight((value, c) => value * x + c, 0);
                result.push(first.currentValue - (first.ref || 0) - target);
                continue;
            }
            const a = first.anchor.clone().applyMatrix4(first.object.matrixWorld).applyMatrix4(inverse);
            const b = second.anchor.clone().applyMatrix4(second.object.matrixWorld).applyMatrix4(inverse);
            result.push(...a.sub(b).toArray());
            if (constraint.type === 'weld') {
                const q1 = first.object.getWorldQuaternion(new THREE.Quaternion()).multiply(first.quaternion);
                const q2 = second.object.getWorldQuaternion(new THREE.Quaternion()).multiply(second.quaternion);
                const error = item.relative.clone().invert().multiply(q1.invert().multiply(q2)).normalize();
                if (error.w < 0) error.set(-error.x, -error.y, -error.z, -error.w);
                const vectorLength = Math.hypot(error.x, error.y, error.z);
                const scale = vectorLength < 1e-12 ? 2 : 2 * Math.atan2(vectorLength, error.w) / vectorLength;
                const weight = constraint.torquescale ?? 1;
                result.push(error.x * scale * weight, error.y * scale * weight, error.z * scale * weight);
            }
        }
        return result;
    }

    apply(values) {
        const previous = this.variables;
        this.variables = [...values];
        for (const entry of this.entries) {
            const { joint, offset, size } = entry;
            if (size === 1) {
                if (joint.currentValue !== values[offset]) this.setScalar(joint, values[offset]);
            } else if (values.slice(offset, offset + size).some((value, i) => value !== previous[offset + i])) {
                const object = joint.threeObject;
                object.position.copy(entry.position);
                if (size === 6) object.position.add(new THREE.Vector3(...values.slice(offset, offset + 3)));
                object.quaternion.copy(entry.quaternion).multiply(rotation(values.slice(offset + size - 3, offset + size)));
            }
        }
        this.updateFrames();
    }

    updateFrames() {
        this.model.threeObject.updateWorldMatrix(true, false);
        for (const frame of this.frames) frame.updateWorldMatrix(false, false);
    }

    clamp(values, unlocked) {
        for (const entry of this.entries) {
            const { joint, offset, size } = entry;
            if (!unlocked.has(offset)) continue;
            if (size === 1 && joint.limits && !this.model.userData?.ignoreLimits) {
                values[offset] = Math.max(joint.limits.lower, Math.min(joint.limits.upper, values[offset]));
            } else if (joint.type === 'ball' && joint.limits) {
                const length = norm(values.slice(offset, offset + 3));
                if (length > joint.limits.upper) for (let i = 0; i < 3; i++) values[offset + i] *= joint.limits.upper / length;
            }
        }
        return values;
    }

    project(pinned, affected = this.constraintEntries, maxIterations = 60) {
        const unlocked = [];
        for (const entry of this.entries) {
            if (!affected.has(entry) || pinned.has(entry.joint.name)) continue;
            for (let i = 0; i < entry.size; i++) unlocked.push(entry.offset + i);
        }
        const unlockedSet = new Set(unlocked);
        const columnByOffset = new Map(unlocked.map((offset, column) => [offset, column]));
        const bounds = this.entries.filter(entry => entry.size === 1 && entry.joint.limits && unlockedSet.has(entry.offset))
            .map(entry => ({ offset: entry.offset, column: columnByOffset.get(entry.offset), limits: entry.joint.limits }));
        let values = [...this.variables];
        let residual = this.residual();
        let damping = 1e-7;
        let iterations = 0;
        for (; iterations < maxIterations && norm(residual) > this.tolerance && unlocked.length; iterations++) {
            const columns = unlocked.map(index => {
                const perturbed = [...values];
                const step = 1e-6;
                perturbed[index] += step;
                this.apply(perturbed);
                const next = this.residual();
                return next.map((value, row) => (value - residual[row]) / step);
            });
            this.apply(values);
            const active = new Set(columns.map((_, index) => index));
            let delta;
            // A joint already at a bound cannot contribute an outward step.
            // Remove those columns and solve again, instead of repeatedly
            // clipping a step whose remaining coordinates cannot close.
            while (active.size) {
                const matrix = residual.map(() => new Float64Array(residual.length));
                for (let row = 0; row < residual.length; row++) {
                    for (let col = 0; col <= row; col++) {
                        let value = row === col ? damping : 0;
                        for (const k of active) value += columns[k][row] * columns[k][col];
                        matrix[row][col] = matrix[col][row] = value;
                    }
                }
                const dual = linearSolve(matrix, residual.map(value => -value));
                if (!dual) break;
                delta = columns.map((column, index) => active.has(index)
                    ? column.reduce((sum, value, row) => sum + value * dual[row], 0) : 0);
                const blocked = new Set();
                if (!this.model.userData?.ignoreLimits) for (const { offset, column, limits } of bounds) {
                    if (!active.has(column)) continue;
                    if ((values[offset] <= limits.lower + 1e-12 && delta[column] < 0)
                        || (values[offset] >= limits.upper - 1e-12 && delta[column] > 0)) blocked.add(column);
                }
                if (!blocked.size) break;
                for (const column of blocked) active.delete(column);
                delta = null;
            }
            if (!delta) break;
            const maxStep = Math.max(1, ...delta.map(value => Math.abs(value) / .2));
            let improved = false;
            for (const fraction of [1, .5, .25, .125, .0625]) {
                const trial = [...values];
                unlocked.forEach((index, col) => { trial[index] += delta[col] * fraction / maxStep; });
                this.apply(this.clamp(trial, unlockedSet));
                const next = this.residual();
                if (norm(next) < norm(residual)) {
                    values = [...this.variables]; residual = next; improved = true; damping = Math.max(1e-10, damping / 3); break;
                }
            }
            if (!improved) {
                this.apply(values);
                damping *= 10;
                if (damping > 1) break;
            }
        }
        this.apply(values);
        return { converged: norm(residual) <= this.tolerance, residual: norm(residual), iterations };
    }

    projectPreservingSprings(pinned, affected, springIterations = 8) {
        // A pose edit carries no load that calls for bending a passive spring.
        // Keep its configured deflection if ordinary linkage joints can close
        // the mechanism. Release it only when closure actually requires it.
        const springs = this.entries.filter(entry => affected.has(entry)
            && entry.joint.stiffness > 0 && !entry.joint.actuated && !pinned.has(entry.joint.name));
        if (!springs.length) return this.project(pinned, affected);
        const start = [...this.variables];
        // This is a preference, so bound its cost before the full solve.
        const status = this.project(new Set([...pinned, ...springs.map(entry => entry.joint.name)]), affected, springIterations);
        if (status.converged) return status;
        this.apply(start);
        return this.project(pinned, affected);
    }

    initializeWithinLimits(adjusted) {
        // Bind equality anchors at qpos0 first, then find a bounded
        // assembly starting with every invalid scalar at its nearest limit.
        // Already valid actuators keep their imported coordinates.
        const pinned = new Set(this.entries.filter(entry => entry.joint.actuated && !adjusted.includes(entry.joint))
            .map(entry => entry.joint.name));
        this.apply(this.clamp([...this.variables], new Set(this.entries.map(entry => entry.offset))));
        const status = this.projectPreservingSprings(pinned, this.constraintEntries, 60);
        status.unsupported = this.unsupported;
        this.status = this.model.constraintStatus = status;
        if (status.converged) this.lastValid = [...this.variables];
        return status;
    }

    solve(changed = null) {
        const driven = changed ? (Array.isArray(changed) ? changed : [changed]) : [];
        const pinned = new Set(driven.map(joint => joint.name));
        // An edit may propagate through connected closure/coupling equations,
        // but must not borrow a DOF from an unrelated leg or mechanism.
        const affected = driven.length
            ? new Set(this.entries.filter(entry => driven.includes(entry.joint)))
            : this.constraintEntries;
        if (driven.length) {
            let expanded;
            do {
                expanded = false;
                for (const item of this.constraints) {
                    if (![...item.entries].some(entry => affected.has(entry))) continue;
                    for (const entry of item.entries) if (!affected.has(entry)) {
                        affected.add(entry); expanded = true;
                    }
                }
            } while (expanded);
        }
        const before = this.entries.map(entry => entry.joint.currentValue);
        const requested = [...this.variables];
        for (const entry of this.entries) if (entry.size === 1) requested[entry.offset] = entry.joint.currentValue;
        // Other actuated coordinates remain fixed when an actuator is driven.
        // Joint-coupled actuators belong to the same input and can follow it.
        if (driven.some(joint => joint.actuated)) {
            const coupled = new Set(pinned);
            for (let i = 0; i < this.constraints.length; i++) for (const item of this.constraints) {
                if (item.constraint.type !== 'joint' || !item.second) continue;
                if (coupled.has(item.first.name) || coupled.has(item.second.name)) {
                    coupled.add(item.first.name); coupled.add(item.second.name);
                }
            }
            for (const entry of this.entries) if (entry.joint.actuated && !coupled.has(entry.joint.name)) pinned.add(entry.joint.name);
        }
        const previous = this.lastValid || [...this.variables];
        // Continue from the last assembly configuration to avoid branch jumps.
        const distance = Math.max(0, ...this.entries.filter(e => driven.includes(e.joint))
            .map(e => Math.abs(requested[e.offset] - previous[e.offset])));
        // A complete pose/reset can be valid at its destination while a
        // straight interpolation of all its passive coordinates is invalid.
        const steps = this.lastValid && driven.length === 1 ? Math.min(40, Math.max(1, Math.ceil(distance / .08))) : 1;
        let status;
        for (let step = 1; step <= steps; step++) {
            const values = this.lastValid ? [...this.variables] : [...requested];
            for (const entry of this.entries) if (pinned.has(entry.joint.name) && entry.size === 1) {
                values[entry.offset] = step === steps ? requested[entry.offset]
                    : previous[entry.offset] + (requested[entry.offset] - previous[entry.offset]) * step / steps;
            }
            this.apply(values);
            status = this.projectPreservingSprings(pinned, affected);
            if (!status.converged) break;
        }
        if (status.converged) this.lastValid = [...this.variables];
        else this.apply(previous);
        status.unsupported = this.unsupported;
        this.status = status;
        this.model.constraintStatus = status;
        const updated = new Set();
        this.entries.forEach((entry, i) => {
            if (entry.size === 1 && Math.abs(entry.joint.currentValue - before[i]) > 1e-12) updated.add(entry.joint);
        });
        return updated;
    }
}
