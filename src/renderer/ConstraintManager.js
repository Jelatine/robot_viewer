import * as THREE from 'three';
import { ModelLoaderFactory } from '../loaders/ModelLoaderFactory.js';
import { ClosedChainSolver } from '../utils/ClosedChainSolver.js';

/**
 * ConstraintManager - Handles parallel mechanism constraint visualization and solving
 */
export class ConstraintManager {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.constraintVisuals = [];
        this.showConstraints = true;
    }

    /**
     * Visualize parallel mechanism constraints
     */
    visualizeConstraints(model, world) {
        // Clean up previous constraint visualizations
        this.constraintVisuals.forEach(visual => {
            if (visual.parent) {
                visual.parent.remove(visual);
            }
        });
        this.constraintVisuals = [];

        if (!model.constraints || model.constraints.size === 0) {
            return; // No constraints, skip
        }
        model.constraints.forEach((constraint, name) => {
            try {
                if (constraint.active === false) return;
                if (constraint.type === 'connect' || constraint.type === 'weld') {
                    // connect and weld constraints: connect two bodies
                    this.visualizeBodyConstraint(model, constraint, world);
                } else if (constraint.type === 'joint') {
                    // joint constraint: joint coupling, shown as info marker
                    this.visualizeJointConstraint(model, constraint, world);
                } else if (constraint.type === 'distance') {
                    // distance constraint: maintain distance
                    this.visualizeDistanceConstraint(model, constraint, world);
                }
            } catch (error) {
                console.error(`Failed to visualize constraint ${name}:`, error);
            }
        });    }

    /**
     * Visualize body constraint (connect/weld)
     */
    visualizeBodyConstraint(model, constraint, world) {
        const binding = model.constraintSolver?.constraints.find(item => item.constraint === constraint);
        const body1 = binding?.first.object;
        const body2 = binding?.second.object;

        if (!body1 || !body2) {
            console.warn(`Constraint ${constraint.name} bodies not found: ${constraint.body1}, ${constraint.body2}`);
            return;
        }

        // Create a dashed line connecting two bodies
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(6); // Two points, 3 coords each
        positions[0] = 0;
        positions[1] = 0;
        positions[2] = 0;
        positions[3] = 0;
        positions[4] = 0;
        positions[5] = 0;
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        // Use different colors to distinguish constraint types
        const color = constraint.type === 'weld' ? 0xff6600 : 0x00ffff; // weld=orange, connect=cyan
        const material = new THREE.LineDashedMaterial({
            color: color,
            linewidth: 2,
            dashSize: 0.05,
            gapSize: 0.03,
            depthTest: true,
            transparent: true,
            opacity: 0.8
        });

        const line = new THREE.Line(geometry, material);
        line.computeLineDistances(); // Must compute for dashed line display

        // Store constraint info
        line.userData.constraint = constraint;
        line.userData.body1 = body1;
        line.userData.body2 = body2;
        line.userData.isConstraintVisualization = true;

        // Add to scene (add to world or scene)
        const parent = world || this.sceneManager.scene;
        parent.add(line);
        this.constraintVisuals.push(line);

        // Create update function, update line position on each render
        line.onBeforeRender = () => {
            if (!line.userData.body1 || !line.userData.body2) return;

            // Get world coordinates of two bodies
            const pos1 = new THREE.Vector3();
            const pos2 = new THREE.Vector3();
            pos1.copy(binding.first.anchor);
            pos2.copy(binding.second.anchor);
            line.userData.body1.localToWorld(pos1);
            line.userData.body2.localToWorld(pos2);

            // If there's a world object, need to convert to world coordinate system
            if (world) {
                const worldMatrix = new THREE.Matrix4();
                worldMatrix.copy(world.matrixWorld).invert();
                pos1.applyMatrix4(worldMatrix);
                pos2.applyMatrix4(worldMatrix);
            }

            // Update line position
            const positions = line.geometry.attributes.position.array;
            positions[0] = pos1.x;
            positions[1] = pos1.y;
            positions[2] = pos1.z;
            positions[3] = pos2.x;
            positions[4] = pos2.y;
            positions[5] = pos2.z;
            line.geometry.attributes.position.needsUpdate = true;
            line.computeLineDistances();
        };    }

    /**
     * Visualize joint constraint (joint coupling)
     */
    visualizeJointConstraint(model, constraint, world) {
        const joint1 = model.getJoint(constraint.joint1);
        const joint2 = model.getJoint(constraint.joint2);

        if (!joint1 || !joint2) {
            console.warn(`Constraint ${constraint.name} joints not found: ${constraint.joint1}, ${constraint.joint2}`);
            return;
        }

        // Get links where the two joints are located
        const link1 = joint1.child ? this.findLinkObject(model.threeObject, joint1.child) : null;
        const link2 = joint2.child ? this.findLinkObject(model.threeObject, joint2.child) : null;

        if (!link1 || !link2) {
            console.warn(`Constraint ${constraint.name} joint links not found`);
            return;
        }

        // Create dashed line connection
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(6);
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        const material = new THREE.LineDashedMaterial({
            color: 0xffff00, // Yellow for joint constraint
            linewidth: 2,
            dashSize: 0.03,
            gapSize: 0.02,
            depthTest: true,
            transparent: true,
            opacity: 0.7
        });

        const line = new THREE.Line(geometry, material);
        line.computeLineDistances();

        line.userData.constraint = constraint;
        line.userData.link1 = link1;
        line.userData.link2 = link2;
        line.userData.isConstraintVisualization = true;

        const parent = world || this.sceneManager.scene;
        parent.add(line);
        this.constraintVisuals.push(line);

        // Update position
        line.onBeforeRender = () => {
            if (!line.userData.link1 || !line.userData.link2) return;

            const pos1 = new THREE.Vector3();
            const pos2 = new THREE.Vector3();
            line.userData.link1.getWorldPosition(pos1);
            line.userData.link2.getWorldPosition(pos2);

            if (world) {
                const worldMatrix = new THREE.Matrix4();
                worldMatrix.copy(world.matrixWorld).invert();
                pos1.applyMatrix4(worldMatrix);
                pos2.applyMatrix4(worldMatrix);
            }

            const positions = line.geometry.attributes.position.array;
            positions[0] = pos1.x;
            positions[1] = pos1.y;
            positions[2] = pos1.z;
            positions[3] = pos2.x;
            positions[4] = pos2.y;
            positions[5] = pos2.z;
            line.geometry.attributes.position.needsUpdate = true;
            line.computeLineDistances();
        };    }

    /**
     * Visualize distance constraint
     */
    visualizeDistanceConstraint(model, constraint, world) {
        // Similar to body constraint, but use different color
        const body1 = this.findLinkObject(model.threeObject, constraint.body1);
        const body2 = this.findLinkObject(model.threeObject, constraint.body2);

        if (!body1 || !body2) {
            console.warn(`Constraint ${constraint.name} bodies not found`);
            return;
        }

        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(6);
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        const material = new THREE.LineDashedMaterial({
            color: 0xff00ff, // Purple for distance constraint
            linewidth: 2,
            dashSize: 0.04,
            gapSize: 0.02,
            depthTest: true,
            transparent: true,
            opacity: 0.8
        });

        const line = new THREE.Line(geometry, material);
        line.computeLineDistances();

        line.userData.constraint = constraint;
        line.userData.body1 = body1;
        line.userData.body2 = body2;
        line.userData.isConstraintVisualization = true;

        const parent = world || this.sceneManager.scene;
        parent.add(line);
        this.constraintVisuals.push(line);

        line.onBeforeRender = () => {
            if (!line.userData.body1 || !line.userData.body2) return;

            const pos1 = new THREE.Vector3();
            const pos2 = new THREE.Vector3();
            line.userData.body1.getWorldPosition(pos1);
            line.userData.body2.getWorldPosition(pos2);

            if (world) {
                const worldMatrix = new THREE.Matrix4();
                worldMatrix.copy(world.matrixWorld).invert();
                pos1.applyMatrix4(worldMatrix);
                pos2.applyMatrix4(worldMatrix);
            }

            const positions = line.geometry.attributes.position.array;
            positions[0] = pos1.x;
            positions[1] = pos1.y;
            positions[2] = pos1.z;
            positions[3] = pos2.x;
            positions[4] = pos2.y;
            positions[5] = pos2.z;
            line.geometry.attributes.position.needsUpdate = true;
            line.computeLineDistances();
        };    }

    /**
     * Apply parallel mechanism constraints (closed-chain kinematics solving)
     * Uses iterative method to solve constraints
     */
    applyConstraints(model, changedJoint) {
        if (!model?.constraints?.size) return new Set();
        if (!model.constraintSolver) {
            model.constraintSolver = new ClosedChainSolver(model,
                (joint, value) => ModelLoaderFactory.setJointAngle(model, joint.name, value, true, false));
        }
        return model.constraintSolver.solve(changedJoint);
    }

    /**
     * Find link object in scene graph
     */
    findLinkObject(root, linkName) {
        let found = null;
        root.traverse((child) => {
            if (child.name === linkName || child.name === `link_${linkName}` || child.name === `body_${linkName}`) {
                found = child;
            }
        });
        return found;
    }

    /**
     * Clear all constraint visualizations
     */
    clear() {
        this.constraintVisuals.forEach(visual => {
            if (visual.parent) visual.parent.remove(visual);
        });
        this.constraintVisuals = [];
    }
}

