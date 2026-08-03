"""Build all encbench parts: python -m encbench (from mechanical/)."""
import runpy

runpy.run_module("encbench.bench", run_name="__main__")
