"""Build all carrier parts: python -m carrier (from mechanical/)."""
import pathlib

from build123d import export_step, export_stl

from blueprint import blueprint

from . import body, cup, assembly

out = pathlib.Path(__file__).parent / "build"
out.mkdir(exist_ok=True)
for mod, name in ((body, "carrier_body"), (cup, "carrier_cup"),
                  (assembly, "carrier_assembly")):
    part = mod.build()
    export_step(part, str(out / f"{name}.step"))
    export_stl(part, str(out / f"{name}.stl"), tolerance=0.01)
    blueprint(part, str(out / f"{name}_bp.svg"), name=name)
    print(f"{name} -> {out}")
