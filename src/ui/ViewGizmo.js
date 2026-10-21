import { Quaternion, Vector3 } from 'three';

const AXES = [
    ['X', new Vector3(1, 0, 0)],
    ['Y', new Vector3(0, 1, 0)],
    ['Z', new Vector3(0, 0, 1)]
];
const SVG_NS = 'http://www.w3.org/2000/svg';

/** Blender-style orientation widget; shares the scene's on-demand render cycle. */
export class ViewGizmo {
    constructor(sceneManager, container) {
        this.sceneManager = sceneManager;
        this.lastOrientation = null;
        this.endpoints = [];
        this.element = document.createElement('div');
        this.element.className = 'view-gizmo';
        this.element.setAttribute('role', 'group');

        const lines = document.createElementNS(SVG_NS, 'svg');
        lines.setAttribute('viewBox', '0 0 100 100');
        lines.setAttribute('aria-hidden', 'true');
        this.element.appendChild(lines);

        for (const [axis, vector] of AXES) {
            const line = document.createElementNS(SVG_NS, 'line');
            line.setAttribute('x1', '50');
            line.setAttribute('y1', '50');
            line.setAttribute('class', `view-gizmo-line view-gizmo-${axis.toLowerCase()}`);
            lines.appendChild(line);

            for (const sign of [1, -1]) {
                const button = document.createElement('button');
                const signedAxis = `${sign > 0 ? '+' : '-'}${axis}`;
                button.type = 'button';
                button.className = `view-gizmo-axis view-gizmo-${axis.toLowerCase()}${sign < 0 ? ' is-negative' : ''}`;
                button.dataset.axis = signedAxis;
                const label = document.createElement('span');
                label.textContent = sign > 0 ? axis : `−${axis}`;
                button.appendChild(label);
                button.addEventListener('click', () => sceneManager.alignCameraToAxis(signedAxis));
                this.element.appendChild(button);
                this.endpoints.push({ button, vector: vector.clone().multiplyScalar(sign), line: sign > 0 ? line : null });
            }
        }

        container.appendChild(this.element);
        // Floating panels can be moved and resized. Keep the widget in the
        // available viewport beside them without polling layout each frame.
        this.container = container;
        this.panels = [...container.querySelectorAll('.floating-panel, #code-editor-panel')];
        if (typeof ResizeObserver !== 'undefined') {
            this.resizeObserver = new ResizeObserver(() => this.updatePosition());
            this.resizeObserver.observe(container);
            this.panelObserver = new MutationObserver(() => this.updatePosition());
            for (const panel of this.panels) {
                this.panelObserver.observe(panel, { attributes: true, attributeFilter: ['style', 'class'] });
            }
        }
        this.refreshLanguage();
        this.update();
    }

    refreshLanguage() {
        const t = key => window.i18n?.t(key) || key;
        this.element.setAttribute('aria-label', t('viewOrientation'));
        this.hint = t('alignViewAxis');
        this.flipHint = t('flipViewAxis');
        this.lastOrientation = null;
        this.update();
    }

    setVisible(visible) {
        this.element.hidden = !visible;
        if (visible) {
            this.updatePosition();
            this.update();
        }
    }

    updatePosition() {
        if (!this.container.getBoundingClientRect) return;
        if (window.innerWidth <= 1023) {
            this.element.style.removeProperty('right');
            return;
        }
        const container = this.container.getBoundingClientRect();
        const top = container.top + 76;
        let right = container.right - 16;
        // Work from the rightmost panel towards the left so adjacent panels
        // can also be avoided after the first shift.
        const panels = this.panels.filter(panel => window.getComputedStyle(panel).display !== 'none')
            .map(panel => panel.getBoundingClientRect()).sort((a, b) => b.right - a.right);
        for (const panel of panels) {
            if (panel.top < top + 120 && panel.bottom > top && panel.left < right && panel.right > right - 120) {
                right = panel.left - 12;
            }
        }
        // A maximized panel owns the viewport; leave the widget underneath it.
        if (right - 120 < container.left) right = container.right - 16;
        this.element.style.right = `${container.right - right}px`;
    }

    update() {
        if (this.element.hidden) return;
        const orientation = new Quaternion().copy(this.sceneManager.camera.quaternion).invert()
            .multiply(this.sceneManager.getViewAxesQuaternion());
        if (this.lastOrientation?.angleTo(orientation) < 1e-7) return;
        this.lastOrientation = orientation;

        for (const { button, vector, line } of this.endpoints) {
            const point = vector.clone().applyQuaternion(orientation);
            const x = 50 + point.x * 34;
            const y = 50 - point.y * 34;
            const aligned = point.z > 1 - 1e-8;
            button.style.left = `${x}%`;
            button.style.top = `${y}%`;
            button.style.zIndex = String(Math.round((point.z + 1) * 100) + 1);
            button.classList.toggle('is-back', point.z < -1e-6);
            button.setAttribute('aria-pressed', String(aligned));
            const label = `${this.hint} ${button.dataset.axis}${aligned ? ` · ${this.flipHint}` : ''}`;
            button.title = label;
            button.setAttribute('aria-label', label);
            if (line) {
                line.setAttribute('x2', String(x));
                line.setAttribute('y2', String(y));
            }
        }
    }
}
