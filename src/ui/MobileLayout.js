/** Mobile panels share one drawer; desktop visibility is restored on return. */
export class MobileLayout {
    constructor(animationWorkspace) {
        this.animationWorkspace = animationWorkspace;
        this.media = window.matchMedia('(max-width: 1023px)');
        this.entries = [
            ['floating-files-panel', 'toggle-files-panel'],
            ['floating-joints-panel', 'toggle-joints-panel'],
            ['floating-model-tree', 'toggle-model-tree'],
            ['floating-help-panel', 'help-button'],
            ['code-editor-panel', 'open-editor-btn'],
            ['animation-workspace', 'toggle-animation-panel']
        ].map(([id, buttonId]) => ({
            panel: document.getElementById(id),
            button: document.getElementById(buttonId)
        })).filter(({ panel }) => panel);
        this.visible = new Set();
        this.observer = new MutationObserver((records) => this.syncPanels(records));
        this.toolbar = document.getElementById('top-control-bar');
        this.toolbarObserver = new ResizeObserver(() => this.updateToolbarInset());
        this.toolbarObserver.observe(this.toolbar);
        this.drawerObserver = new ResizeObserver(() => this.updateCanvasInset());
        this.entries.forEach(({ panel }) => this.drawerObserver.observe(panel));
        this.media.addEventListener('change', () => this.updateLayout());
        this.updateLayout();
    }

    updateToolbarInset() {
        if (this.media.matches) {
            document.documentElement.style.setProperty('--mobile-toolbar-bottom', `${this.toolbar.getBoundingClientRect().bottom + 8}px`);
        } else {
            document.documentElement.style.removeProperty('--mobile-toolbar-bottom');
        }
    }

    updateCanvasInset() {
        const rootStyle = document.documentElement.style;
        if (!this.media.matches) {
            rootStyle.removeProperty('--mobile-drawer-inset');
            rootStyle.removeProperty('--mobile-drawer-right');
            return;
        }
        const drawer = this.entries.find((entry) => this.isVisible(entry));
        const bounds = drawer?.panel.getBoundingClientRect();
        const sideDrawer = window.innerHeight <= 500;
        const inset = bounds && !sideDrawer ? window.innerHeight - bounds.top + 8 : 0;
        const rightInset = bounds && sideDrawer ? window.innerWidth - bounds.left + 8 : 0;
        rootStyle.setProperty('--mobile-drawer-inset', `${Math.max(0, inset)}px`);
        rootStyle.setProperty('--mobile-drawer-right', `${Math.max(0, rightInset)}px`);
    }

    isVisible({ panel }) {
        return !panel.hidden && getComputedStyle(panel).display !== 'none';
    }

    setVisible(entry, visible) {
        const { panel, button } = entry;
        clearTimeout(panel._visibilityTimer);
        if (panel.id === 'animation-workspace') {
            if (this.animationWorkspace.visible !== visible) {
                if (visible) this.animationWorkspace.open();
                else this.animationWorkspace.close();
            }
        } else if (panel.id === 'code-editor-panel') {
            panel.classList.toggle('visible', visible);
        } else {
            panel.style.display = visible ? 'flex' : 'none';
            panel.style.opacity = '';
        }
        button?.classList.toggle('active', visible);
        button?.setAttribute('aria-expanded', String(visible));
    }

    updateLayout() {
        this.observer.disconnect();
        if (this.media.matches) {
            this.desktopVisibility = this.entries.map((entry) => this.isVisible(entry));
            const initial = this.entries.find((entry) => this.isVisible(entry));
            this.entries.forEach((entry) => this.setVisible(entry, entry === initial));
        } else if (this.desktopVisibility) {
            this.entries.forEach((entry, index) => this.setVisible(entry, this.desktopVisibility[index]));
            this.desktopVisibility = null;
        }
        this.visible = new Set(this.entries.filter((entry) => this.isVisible(entry)));
        this.entries.forEach((entry) => {
            const { panel, button } = entry;
            button?.setAttribute('aria-controls', panel.id);
            button?.setAttribute('aria-expanded', String(this.visible.has(entry)));
            this.observer.observe(panel, { attributes: true, attributeFilter: ['style', 'class', 'hidden'] });
        });
        this.updateToolbarInset();
        this.updateCanvasInset();
    }

    syncPanels(records) {
        const opened = this.entries.filter((entry) => this.isVisible(entry) && !this.visible.has(entry));
        if (this.media.matches && opened.length) {
            const lastOpened = [...records].reverse().map(({ target }) => opened.find(({ panel }) => panel === target)).find(Boolean);
            const active = lastOpened || opened[opened.length - 1];
            this.entries.forEach((entry) => {
                if (entry !== active && this.isVisible(entry)) this.setVisible(entry, false);
            });
        }
        this.visible = new Set(this.entries.filter((entry) => this.isVisible(entry)));
        this.entries.forEach((entry) => entry.button?.setAttribute('aria-expanded', String(this.visible.has(entry))));
        this.updateCanvasInset();
    }
}
