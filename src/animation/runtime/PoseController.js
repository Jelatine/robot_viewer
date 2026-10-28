import { isScalarJoint } from '../../models/UnifiedRobotModel.js';
import { ModelLoaderFactory } from '../../loaders/ModelLoaderFactory.js';

/**
 * The single write path for robot poses.
 *
 * UI controls, viewport dragging and animation playback all go through this
 * controller so an editor can distinguish previews from committed user edits.
 */
export class PoseController {
    constructor(sceneManager = null) {
        this.sceneManager = sceneManager;
        this.model = null;
        this.listeners = new Set();
    }

    setModel(model) {
        this.model = model || null;
        this.emit({ type: 'modelChanged', model: this.model });
    }

    subscribe(listener) {
        this.listeners.add(listener);
        return () => this.listeners.delete(listener);
    }

    emit(event) {
        this.listeners.forEach((listener) => listener(event));
    }

    getJointValue(jointName) {
        return this.model?.joints?.get(jointName)?.currentValue ?? 0;
    }

    getPose() {
        const pose = {};
        this.model?.joints?.forEach((joint, name) => {
            if (isScalarJoint(joint)) {
                pose[name] = joint.currentValue ?? 0;
            }
        });
        return pose;
    }

    setJointValue(jointName, value, options = {}) {
        const {
            source = 'user',
            commit = false,
            ignoreLimits = false,
            applyConstraints = true,
            render = true,
            measure = true
        } = options;

        const joint = this.model?.joints?.get(jointName);
        if (!isScalarJoint(joint) || !Number.isFinite(value)) {
            return null;
        }

        let appliedValue = value;
        const limitsAreIgnored = ignoreLimits || this.sceneManager?.ignoreLimits;
        if (!limitsAreIgnored && joint.type !== 'continuous' && joint.limits) {
            const lower = Number.isFinite(joint.limits.lower) ? joint.limits.lower : -Infinity;
            const upper = Number.isFinite(joint.limits.upper) ? joint.limits.upper : Infinity;
            appliedValue = Math.max(lower, Math.min(upper, appliedValue));
        }

        ModelLoaderFactory.setJointAngle(this.model, jointName, appliedValue, limitsAreIgnored, !this.model.constraintSolver);
        joint.currentValue = appliedValue;

        if (applyConstraints && this.sceneManager?.constraintManager) {
            this.projectConstraints([joint], { commit });
            appliedValue = joint.currentValue;
        }

        if (render) {
            this.sceneManager?.redraw();
        }
        if (measure && this.sceneManager?.onMeasurementUpdate) {
            this.sceneManager.onMeasurementUpdate();
        }

        this.emit({
            type: 'jointChanged',
            model: this.model,
            joint,
            jointName,
            value: appliedValue,
            source,
            commit
        });

        return appliedValue;
    }

    commitJointValue(jointName, { source = 'user' } = {}) {
        const joint = this.model?.joints?.get(jointName);
        if (!isScalarJoint(joint)) return;

        this.emit({
            type: 'jointChanged',
            model: this.model,
            joint,
            jointName,
            value: joint.currentValue ?? 0,
            source,
            commit: true
        });
    }

    projectConstraints(joints = null, { commit = false } = {}) {
        const updated = this.sceneManager?.constraintManager?.applyConstraints(this.model, joints);
        updated?.forEach(joint => this.emit({ type: 'jointChanged', model: this.model, joint,
            jointName: joint.name, value: joint.currentValue, source: 'constraint', commit }));
        if (this.model?.constraintStatus) this.emit({ type: 'constraintsSolved', status: this.model.constraintStatus });
    }

    applyPose(values, { source = 'playback', ignoreLimits = false } = {}) {
        if (!this.model || !values) return;

        const joints = [];
        Object.entries(values).forEach(([jointName, value]) => {
            const applied = this.setJointValue(jointName, value, {
                source,
                commit: false,
                ignoreLimits,
                applyConstraints: false,
                render: false,
                measure: false
            });
            if (applied !== null) joints.push(this.model.getJoint(jointName));
        });
        this.projectConstraints(joints);

        this.sceneManager?.redraw();
        if (this.sceneManager?.onMeasurementUpdate) {
            this.sceneManager.onMeasurementUpdate();
        }
    }
}
