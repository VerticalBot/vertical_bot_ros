"""Sphinx configuration for VerticalBot Studio documentation (Read the Docs)."""
import os, re

project = "VerticalBot Studio"
author = "VerticalBot"
copyright = "VerticalBot"
# version from studio/package.json
try:
    with open(os.path.join(os.path.dirname(__file__), "..", "studio", "package.json"), encoding="utf-8") as f:
        m = re.search(r'"version"\s*:\s*"([^"]+)"', f.read())
        release = m.group(1) if m else "0.1"
except OSError:
    release = "0.1"
version = release

extensions = ["myst_parser", "sphinx_copybutton", "sphinx.ext.autosectionlabel"]
myst_enable_extensions = ["colon_fence", "deflist", "tasklist", "substitution", "attrs_inline"]
myst_heading_anchors = 3
autosectionlabel_prefix_document = True
suppress_warnings = ["autosectionlabel.*", "myst.header"]
source_suffix = {".md": "markdown", ".rst": "restructuredtext"}
master_doc = "index"
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
language = "en"

html_theme = "sphinx_rtd_theme"
html_title = "VerticalBot Studio documentation"
html_theme_options = {
    "navigation_depth": 4,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "titles_only": False,
    "logo_only": False,
}
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_show_sourcelink = True
html_context = {
    "display_github": True,
    "github_user": "VerticalBot",
    "github_repo": "vertical_bot_ros",
    "github_version": "main",
    "conf_py_path": "/docs/",
}
