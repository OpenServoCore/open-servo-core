"""Live OCP viewer preview: python preview.py stenciljig.knob [more.modules]

Calls every build*() in the listed modules and pushes the parts to the
OCP CAD Viewer (start the viewer panel in VS Code first), then re-runs
on any .py save under mechanical/. Each refresh is a fresh subprocess,
so edits to any imported file are picked up. --once builds and exits.
"""
import importlib
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).parent
SKIP = {".venv", "__pycache__", "build", "reference"}


def _parts(mod_name):
    mod = importlib.import_module(mod_name)
    fns = sorted(n for n in vars(mod)
                 if n == "build" or n.startswith("build_"))
    return [vars(mod)[n]() for n in fns if callable(vars(mod)[n])]


def _show(mod_names):
    from ocp_vscode import Camera, set_defaults, show
    set_defaults(reset_camera=Camera.KEEP)
    parts = [p for m in mod_names for p in _parts(m)]
    names = [getattr(p, "label", "") or f"part{i}"
             for i, p in enumerate(parts)]
    show(*parts, names=names)


def _mtimes():
    return {p: p.stat().st_mtime for p in ROOT.rglob("*.py")
            if not SKIP.intersection(p.parts)}


def main():
    args = sys.argv[1:]
    if args and args[0] == "--once":
        _show(args[1:])
        return
    if not args:
        sys.exit(__doc__)
    seen = None
    while True:
        now = _mtimes()
        if now != seen:
            seen = now
            r = subprocess.run(
                [sys.executable, __file__, "--once", *args], cwd=ROOT)
            print("refreshed" if r.returncode == 0 else "BUILD FAILED",
                  time.strftime("%H:%M:%S"), flush=True)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
