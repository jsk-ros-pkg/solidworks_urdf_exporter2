# quadruped — the tutorial robot

A four-legged robot made of three primitive parts, built for the
[tutorial](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/tutorial/)
([日本語](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/tutorial/)).
It has 9 links and 8 revolute joints, a hip and a knee per leg.

| Path | What it is |
| --- | --- |
| `cad/` | the SolidWorks files (open `quadruped.SLDASM`; SolidWorks 2024 SP5+) |
| `build_cad.py` | builds `cad/` from scratch over the SolidWorks API |
| `graph.json`, `meshes/` | what `sw2urdf` extracted from `cad/` |
| `quadruped.joints.yaml` | the joint config, edited in step 4 of the tutorial |

`urdf/`, `package.xml` and `CMakeLists.txt` are not committed; rebuild them
without SolidWorks:

```bash
python -m sw2robot.exporter.build examples/quadruped \
    --config examples/quadruped/quadruped.joints.yaml
```

Re-extract from the CAD (Windows + SolidWorks), keeping the edited config:

```bash
sw2urdf examples/quadruped/cad/quadruped.SLDASM -o examples -n quadruped \
    --config examples/quadruped/quadruped.joints.yaml
```

Rebuild the CAD and the tutorial's images (Windows + SolidWorks):

```bash
uv run python examples/quadruped/build_cad.py --overwrite \
    --screenshots docs/images --record recording
# then copy recording/{body,thigh,shin,assembly}.gif to docs/images/build_*.gif
```
