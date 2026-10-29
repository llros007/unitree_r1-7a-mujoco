# Plant geometry and contact assumptions

The plant meshes and texture are CC0 assets. Source and applicable resource
licenses are listed in [THIRD_PARTY_LICENSES.md](../../../THIRD_PARTY_LICENSES.md).

The release includes only four evaluated OBJ meshes, one 4K PNG diffuse
texture and `model.json`. Blender modifiers, world transforms and UVs are
baked into the OBJ files. Unused normal/roughness EXRs, the original Blender
file, the duplicate JPG and the original ZIP are not distributed.

Uniform scale: 0.35738467845019384. Pot outer diameter: **0.060 m**; pot
height: **0.05542122675375947 m**. The origin is the pot bounding-box center.
`model.json` records the original bounds, conversion scale and mesh parts.
Both entry points use the same asset and collision geometry.

Pot collision uses a convex hull, including a solid approximation of the
opening. Plant, dirt and ground are visual geometry. Mass is an estimated
80 g; inertia approximates a uniform cylinder at the scaled dimensions.
Friction, rigidity and compliance are simulation approximations. Five
MuJoCo no-slip refinement iterations reduce numerical friction creep.
No attachment or object pose correction is used during grabbing.

Optional regeneration requires Blender and the original download. Extract
`potted_plant_04_4k.blend` and `textures/potted_plant_04_diff_4k.jpg` from the
ZIP into this directory, then run from the repository root:

```bash
blender -b -t 2 --python yolo_grasp/convert_plant04.py
```

Those original input files are ignored by Git. Normal simulation startup
requires neither Blender nor the original download.
