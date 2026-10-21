![screenshot](./docs/screenshot.png)

---

# Robot Viewer

[![Version](https://img.shields.io/badge/version-v1.5.0-blue.svg)](https://github.com/fan-ziqi/robot_viewer)
[![License](https://img.shields.io/badge/license-Apache--2.0-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-web-orange.svg)](https://github.com/fan-ziqi/robot_viewer)
[![JavaScript](https://img.shields.io/badge/language-JavaScript-f1e05a.svg)](https://github.com/fan-ziqi/robot_viewer)
[![Three.js](https://img.shields.io/badge/Three.js-0.163.0-black.svg)](https://threejs.org/)
[![Vite](https://img.shields.io/badge/Vite-4.5.0-646cff.svg)](https://vitejs.dev/)
[![Demo](https://img.shields.io/badge/Demo-Live-brightgreen.svg)](http://viewer.robotsfan.com/)

**Robot Viewer** is a web-based 3D viewer for robot models and scenes. Built on top of [Three.js](https://threejs.org/), it provides an intuitive interface for visualizing, editing, and simulating robots directly in the browser without any installation required. It supports serial and branched robots, as well as MJCF parallel mechanisms with closed-chain joint coordination.

**Live Demo** (All processing happens in your browser - your models never leave your device):

[![Try it now](https://img.shields.io/badge/🌐_Try_it_now-viewer.robotsfan.com-brightgreen?style=for-the-badge)](http://viewer.robotsfan.com/)

## Key Features

- **Format Support**:
  - **URDF**: Unified Robot Description Format
  - **Xacro**: ROS Xacro format with macro expansion and conditional logic support
  - **MJCF**: Mujoco XML format
  - **USD**: Universal Scene Description (partial support)
- **Robot Types**: Serial, branched and MJCF parallel mechanisms with `connect`, `weld` and joint equality constraints
- **Visualization Tools**: Visual/collision geometry, inertia tensors, center of mass, coordinate frames, joint axes, shadows, ground grid visibility, coordinate system orientation
- **Interactive Controls**: Drag joints in real-time, adjust model poses with linked motion for parallel mechanisms
- **Axis Views**: Blender-style orientation gizmo in the upper-right corner. Click an X/Y/Z endpoint to view from that positive or negative model axis; click the aligned endpoint again to flip to the opposite side. The view keeps its orbit target and distance and follows the selected up-axis. Available in the Three.js viewer (URDF, Xacro, MJCF and meshes).
- **Animation Editor (Experimental)**: Multi-clip dope sheet and Bézier graph editor with auto-key, multi-selection, undo/redo, play/record ranges, event and media tracks, pose snapshots, audio-reactive keys, live-input recording, autosave, and portable `.robotanim.json` projects
- **Measurement Tools**: Measure distances between joints and links with 3D visualization, display X/Y/Z axis projections and total distance, support ground height measurement
- **Code Editor**: Built-in CodeMirror editor with syntax highlighting and live preview
- **Physics Simulation**: Integrated MuJoCo engine for dynamics simulation (MJCF models)
- **Scene Management**: File tree and scene graph visualization with hierarchical structure

## Getting Started

This project uses **pnpm**, but you can also use **npm** or **yarn**.

Clone the repository and install dependencies:

```bash
git clone https://github.com/fan-ziqi/robot_viewer.git
cd robot_viewer
pnpm install
```

Start the development server:

```bash
pnpm run dev
```

Build for production:

```bash
pnpm run build
```

Output will be in the `dist/` directory.

## MJCF Parallel Mechanisms

Example mechanisms include the [Robotiq 2F-85 four-bar gripper](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85) and a [Stewart platform with six closed branches](https://github.com/duccuongvu/stewart_platform_mujoco). Obtain the XML and assets from the source repository, then drag the model folder into the viewer. It opens the robot model XML by default; select `scene.xml` in the file tree when you want the complete scene. Keep the XML and `assets/` together when importing your own models.

Move a Robotiq driver joint or a Stewart `joint_lu_*` slider. The selected coordinate stays fixed at your requested value while passive joints follow to close all branches. The solver changes only joints in the affected closed chain; shared ancestors and unrelated branches keep their poses. Other actuators stay fixed unless coupled to the selected actuator by a joint equality. Sliders, viewport dragging, animation and reset use the same constraint solver. Unreachable edits restore the previous valid pose and show a message in the joint panel.

Supported MJCF constraints are body/site `connect`, body/site `weld` (including relative poses), and polynomial joint equalities. Passive ball joints and floating platforms participate in closure; scalar sliders and animation tracks control hinge/slide joints. The pose editor performs kinematic closure; use **Simulate** for MuJoCo dynamics. Tendon equalities, contact-based closure and arbitrary mechanism-specific inverse kinematics are outside this support.

The importer also handles nested includes, default classes and `childclass`, multiple joints per body, joint anchors/references, degree/radian units and MJCF frame orientations. Controls use radians for rotation and meters for translation.

Additional regression models include Cassie, Robotiq 2F-85 v4, ToddlerBot 2XC and IIT SoftFoot. SoftFoot's original `scene.xml` is supported, including its loading slide and attached foot.

MJCF [`<attach>` model composition](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-attach) uses the bundled MuJoCo compiler to preserve prefixes, defaults, assets, actuators, sensors and equality references. It supports external body subtrees or whole models, nested/repeated attachments, and placement inside frames. Keep all referenced XML and asset files together; assembly imports require a model that compiles in MuJoCo. Newer self-attachment and named-frame selection syntax are outside the bundled runtime's support and report explicit errors.

On import, joints outside their limits are clamped to the nearest limit. Closed mechanisms then project the assembly within those limits, keeping already valid actuators fixed; linked joints may need to move together to achieve closure. Passive spring joints retain their current deflection whenever closure permits. Spring meshes remain rigid, so directly changing a spring joint can create a visible gap rather than a bent spring.

Run the regression suite with:

```bash
pnpm test
```

Tests cover all six parallel models, forward/reverse motion and reset, complete equality import, actuator preservation, passive ball joints and rejection of unreachable poses. Serial regressions compare every body of Panda, UR5e and Lite6 against MuJoCo `mj_forward` in five poses each. Original XML, licenses, source revisions and references are in `tests/fixtures/mjcf/`. Mesh rendering and physics compilation require the full downloaded examples.

## Contributing

We welcome contributions from the community! Whether you're fixing bugs, adding features, or improving documentation, your help is appreciated.

- **Bug Reports**: Open an [issue](https://github.com/fan-ziqi/robot_viewer/issues) with details
- **Feature Requests**: Discuss ideas in [Discussions](https://github.com/fan-ziqi/robot_viewer/discussions)
- **Pull Requests**: Submit PRs with clear descriptions and tests

## License

This project is licensed under the [Apache License 2.0](LICENSE).

## Acknowledgements

Robot Viewer builds upon the excellent work of the open-source robotics community. This project integrates several powerful open-source projects:

- **[urdf-loader](https://github.com/gkjohnson/urdf-loaders)** - Robust URDF loading for Three.js
- **[xacro-parser](https://github.com/gkjohnson/xacro-parser)** - ROS Xacro file format parser for Javascript
- **[mujoco_wasm](https://github.com/zalo/mujoco_wasm)** - MuJoCo physics engine compiled to WebAssembly
- **[usd-viewer](https://github.com/needle-tools/usd-viewer)** - OpenUSD viewer with rich USDStage support
- **[mechaverse](https://github.com/jurmy24/mechaverse)** - Universal 3D viewer for robot models, providing valuable design inspiration

Special thanks to all the maintainers and contributors of these projects for their foundational work.

Parts of this project were developed with the assistance of [Cursor](https://cursor.sh).
