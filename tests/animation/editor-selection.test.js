import test from 'node:test';
import assert from 'node:assert/strict';
import { EditorSelection } from '../../src/animation/core/EditorSelection.js';

test('repeated joint previews preserve track selection without notifying canvases again', () => {
    const selection = new EditorSelection();
    let updates = 0;
    selection.subscribe(() => updates++);
    selection.selectTrack('drive');
    for (let i = 0; i < 50; i++) selection.selectTrack('drive');
    assert.equal(updates, 1);
    selection.selectKeyframe({ trackId: 'drive', keyframeId: 'key' });
    selection.selectTrack('drive');
    assert.equal(selection.keyframes.size, 0);
    assert.equal(updates, 3);
    selection.selectTrack('drive', { toggle: true });
    assert.equal(selection.trackIds.size, 0);
    selection.selectTrack('other', { additive: true });
    selection.selectTrack('drive', { additive: true });
    selection.selectTrack('drive');
    assert.deepEqual([...selection.trackIds], ['drive']);
});
