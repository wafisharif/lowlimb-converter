# Station 1b: element-support reports

`python -m lowlimb_converter.support <model.osim> --geometry <dir> --md report.md --json report.json` lists every
element of an OpenSim model with one status (converted, approximated, skipped, ignored, unsupported) and predicts
whether the converter's XML step will succeed. Every rule cites the vendored source it comes from
(`lowlimb_converter/support.py`, `RULES`). Gates S1–S3 are in `station1/GATES.md`; results are in
`station1/FINDINGS.md`.

## Models and where they came from

| Report | Model file | Source | SHA-256 (first 16) | Geometry folder used |
| --- | --- | --- | --- | --- |
| `gait10dof18musc.*` | gait10dof18musc.osim | MyoConverter @ cadf380, `models/osim/Gait10dof18musc/` | 8b7a6f6e509641e8 | that model's `Geometry/` |
| `gait2354.*` | gait2354.osim | MyoConverter @ cadf380, `models/osim/Gait2354Simbody/` | a895cf566552e98b | that model's `Geometry/` |
| `Rajagopal2016.*` | Rajagopal2016.osim | [opensim-org/opensim-models](https://github.com/opensim-org/opensim-models) @ d9b05d4, `Models/Rajagopal/` | 3f5c5f23e486073f | merged (below) |
| `RajagopalLaiUhlrich2023.*` | RajagopalLaiUhlrich2023.osim | same repo and commit | 8f30d0b64750b87e | merged (below) |

**Merged geometry for the Rajagopal models.** The converter takes one geometry folder. 72 of the 81 meshes these
models use live in the repo-wide `Geometry/` folder, not `Models/Rajagopal/Geometry/`. We copied the repo-wide folder,
then copied `Models/Rajagopal/Geometry/` over it, so model-specific files win (OpenSim also looks in the model's own
folder first). Five files exist in both with different contents, and the model-specific version was used:
`l_fibula.vtp l_pelvis.vtp r_fibula.vtp r_pelvis.vtp sacrum.vtp`. With only `Models/Rajagopal/Geometry/`, conversion
fails at the first missing mesh, which the report predicts (`s3_results.txt`, third case).

**Licenses.** The gait models are CC BY 3.0 (their `notes.txt` in MyoConverter). We found no license file for the
Rajagopal models in opensim-models; only these reports about them are committed here, not the model files.
Check the license before redistributing a converted Rajagopal model.

## Files

- `<model>.md`: human-readable summary (non-converted elements grouped by reason).
- `<model>.json`: every element with section, type, path, status, reason and rule.
- `s3_results.txt`: gate S3 output, predicted vs actual XML conversion step (local paths removed).
