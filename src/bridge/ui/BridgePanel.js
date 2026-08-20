/**
 * BridgePanel - connection UI for the local joint stream.
 *
 * Built in JS rather than in index.html so the whole feature stays in one
 * folder; it still reuses the floating-panel markup so PanelManager can drag
 * and resize it like every other panel.
 */
import { RemoteBridge, BRIDGE_DEFAULT_URL } from '../RemoteBridge.js';
import './bridge-panel.css';

const STORAGE_KEY = 'robot-viewer.bridge';

export class BridgePanel {
    constructor({ poseController, panelManager }) {
        this.bridge = new RemoteBridge(poseController);
        this.panelManager = panelManager;

        this.createUI();
        this.bindEvents();
        this.restoreSettings();

        this.bridge.subscribe((event) => {
            if (event.type === 'status') this.renderStatus();
            else if (event.type === 'stats') this.renderStats();
            else if (event.type === 'mapping') this.renderMapping();
        });

        this.renderStatus();
        this.renderStats();
        this.renderMapping();
    }

    createUI() {
        const host = document.getElementById('floating-joints-panel')?.parentElement || document.body;

        this.root = document.createElement('div');
        this.root.id = 'floating-bridge-panel';
        this.root.className = 'floating-panel';
        this.root.style.display = 'none';
        this.root.innerHTML = `
            <div class="floating-panel-header">
                <span data-i18n="bridgeTitle">Local Bridge</span>
                <button class="panel-close-btn" data-panel="floating-bridge-panel">✕</button>
            </div>
            <div class="floating-panel-content bridge-content">
                <div class="bridge-status">
                    <span class="bridge-dot"></span>
                    <span class="bridge-status-text"></span>
                </div>

                <label class="bridge-field">
                    <span data-i18n="bridgeUrl">Server address</span>
                    <input type="text" class="bridge-input" data-bridge="url" spellcheck="false"
                           placeholder="${BRIDGE_DEFAULT_URL}">
                </label>

                <label class="bridge-field">
                    <span data-i18n="bridgeToken">Token (optional)</span>
                    <input type="password" class="bridge-input" data-bridge="token" spellcheck="false"
                           autocomplete="off">
                </label>

                <div class="bridge-actions">
                    <button class="control-button bridge-connect" data-bridge-action="toggle"></button>
                    <label class="bridge-checkbox">
                        <input type="checkbox" data-bridge="auto-reconnect" checked>
                        <span data-i18n="bridgeAutoReconnect">Auto reconnect</span>
                    </label>
                </div>

                <div class="bridge-stats">
                    <div><span data-i18n="bridgeRateIn">In</span><b class="bridge-rate-in">0</b> Hz</div>
                    <div><span data-i18n="bridgeRateOut">Applied</span><b class="bridge-rate-out">0</b> Hz</div>
                    <div><span data-i18n="bridgeDropped">Coalesced</span><b class="bridge-dropped">0</b></div>
                </div>

                <div class="bridge-mapping"></div>
            </div>
        `;
        host.appendChild(this.root);

        this.urlInput = this.root.querySelector('[data-bridge="url"]');
        this.tokenInput = this.root.querySelector('[data-bridge="token"]');
        this.autoReconnectInput = this.root.querySelector('[data-bridge="auto-reconnect"]');
        this.connectButton = this.root.querySelector('.bridge-connect');
        this.statusText = this.root.querySelector('.bridge-status-text');
        this.statusDot = this.root.querySelector('.bridge-dot');
        this.mappingBox = this.root.querySelector('.bridge-mapping');

        this.panelManager?.registerPanel('floating-bridge-panel');
        window.i18n?.updatePageLanguage();
    }

    bindEvents() {
        // Opening, closing and the position reset come from UIController's shared
        // panel wiring, so this panel animates exactly like Files and Structure.
        this.toggleButton = document.getElementById('toggle-bridge-panel');
        this.connectButton.addEventListener('click', () => this.toggleConnection());

        this.urlInput.addEventListener('keydown', (event) => {
            if (event.key === 'Enter') this.toggleConnection();
        });
        this.autoReconnectInput.addEventListener('change', () => {
            this.bridge.autoReconnect = this.autoReconnectInput.checked;
            this.saveSettings();
        });
    }

    toggleConnection() {
        if (this.bridge.status === 'disconnected' || this.bridge.status === 'error') {
            this.bridge.autoReconnect = this.autoReconnectInput.checked;
            this.bridge.connect(this.urlInput.value.trim() || BRIDGE_DEFAULT_URL, this.tokenInput.value);
            this.saveSettings();
        } else {
            this.bridge.disconnect();
        }
    }

    // ------------------------------------------------------------- rendering

    t(key, fallback) {
        return window.i18n?.t(key) || fallback;
    }

    renderStatus() {
        const { status, statusDetail } = this.bridge;
        const labels = {
            disconnected: this.t('bridgeDisconnected', 'Disconnected'),
            connecting: this.t('bridgeConnecting', 'Connecting…'),
            connected: this.t('bridgeConnected', 'Connected'),
            error: this.t('bridgeError', 'Error')
        };
        this.statusText.textContent = statusDetail
            ? `${labels[status]} · ${statusDetail}`
            : labels[status];
        this.statusDot.dataset.status = status;
        this.connectButton.textContent = (status === 'connected' || status === 'connecting')
            ? this.t('bridgeDisconnect', 'Disconnect')
            : this.t('bridgeConnect', 'Connect');
        this.connectButton.classList.toggle('active', status === 'connected');
        this.toggleButton?.classList.toggle('bridge-live', status === 'connected');
    }

    renderStats() {
        const { rateIn, rateOut, dropped } = this.bridge.stats;
        this.root.querySelector('.bridge-rate-in').textContent = rateIn;
        this.root.querySelector('.bridge-rate-out').textContent = rateOut;
        this.root.querySelector('.bridge-dropped').textContent = dropped;
    }

    renderMapping() {
        const { matched, unknown, undriven, total } = this.bridge.mapping;
        if (!this.bridge.remoteJoints.length) {
            this.mappingBox.innerHTML =
                `<div class="bridge-hint">${this.t('bridgeAwaitingHello', 'Waiting for the server to declare its joints.')}</div>`;
            return;
        }

        const rows = [
            `<div class="bridge-map-row ok">
                <span>${this.t('bridgeMatched', 'Driven joints')}</span>
                <b>${matched.length} / ${total}</b>
             </div>`
        ];
        if (unknown.length) {
            rows.push(`<div class="bridge-map-row warn">
                <span>${this.t('bridgeUnknown', 'Not in model')}</span>
                <b title="${unknown.join(', ')}">${unknown.length}</b>
             </div>
             <div class="bridge-names">${unknown.join(', ')}</div>`);
        }
        if (undriven.length) {
            rows.push(`<div class="bridge-map-row muted">
                <span>${this.t('bridgeUndriven', 'Not streamed')}</span>
                <b title="${undriven.join(', ')}">${undriven.length}</b>
             </div>`);
        }
        this.mappingBox.innerHTML = rows.join('');
    }

    // -------------------------------------------------------------- settings

    saveSettings() {
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify({
                url: this.urlInput.value.trim(),
                autoReconnect: this.autoReconnectInput.checked
            }));
        } catch {
            // Private browsing or a full quota - the panel still works, it just forgets.
        }
    }

    restoreSettings() {
        let saved = {};
        try {
            saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
        } catch {
            saved = {};
        }
        this.urlInput.value = saved.url || BRIDGE_DEFAULT_URL;
        this.autoReconnectInput.checked = saved.autoReconnect !== false;
        this.bridge.autoReconnect = this.autoReconnectInput.checked;
    }

    /** Re-render the labels this panel builds in JS, which data-i18n cannot reach. */
    refreshLanguage() {
        this.renderStatus();
        this.renderMapping();
    }
}
