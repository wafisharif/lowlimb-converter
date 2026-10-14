"""Sphinx configuration for the osim2mujoco documentation (built on Read the Docs, see .readthedocs.yaml)."""
import os
import sys

sys.path.insert(0, os.path.abspath(".."))   # the osim2mujoco package, for the API reference

from osim2mujoco import __version__   # noqa: E402

project = "osim2mujoco"
author = "Wafi Sharif"
copyright = "2026, Wafi Sharif"
release = __version__
version = __version__

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.extlinks",
    "sphinx_copybutton",
]

source_suffix = {".md": "markdown"}
root_doc = "index"
exclude_patterns = ["_build"]
myst_enable_extensions = ["colon_fence", "deflist"]
myst_heading_anchors = 3

# Repository files (findings, gates, results) are linked on GitHub rather than copied into the site.
REPO = "https://github.com/wafisharif/osim2mujoco"
extlinks = {
    "repo": (f"{REPO}/blob/main/%s", "%s"),
    "tree": (f"{REPO}/tree/main/%s", "%s"),
}

# The API reference documents the package without installing the simulators: these are imported lazily or only
# inside functions, and mocked here so the build needs no OpenSim or MuJoCo.
autodoc_mock_imports = ["opensim", "mujoco", "pyvista", "trimesh", "scipy", "matplotlib", "fpdf", "networkx",
                        "sklearn", "seaborn", "myoconverter"]
autodoc_member_order = "bysource"
autodoc_default_options = {"members": False}

html_theme = "furo"
html_title = f"osim2mujoco {version}"
html_static_path = ["_static"]
html_theme_options = {
    "source_repository": REPO,
    "source_branch": "main",
    "source_directory": "docs/",
}
