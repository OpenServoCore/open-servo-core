"""Run from mechanical/: .venv/bin/python -m testharness."""
import argparse
from pathlib import Path

from build123d import export_step, export_stl

from .harness import Spec, build


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--servo-width', type=float, default=Spec.servo_w)
    parser.add_argument('--servo-length', type=float, default=Spec.servo_l)
    parser.add_argument('--servo-clearance', type=float, default=Spec.servo_clear)
    parser.add_argument('--count', type=int, default=Spec.count)
    args = parser.parse_args()
    spec = Spec(count=args.count, servo_w=args.servo_width,
                servo_l=args.servo_length, servo_clear=args.servo_clearance)
    part = build(spec)
    if not part.is_valid or len(part.solids()) != 1:
        raise RuntimeError("Harness must be one valid solid")
    out = Path(__file__).parent / 'build'
    out.mkdir(exist_ok=True)
    name = 'testharness' if spec.count == Spec.count else f'testharness_{spec.count}'
    export_step(part, out / f'{name}.step')
    export_stl(part, out / f'{name}.stl', tolerance=0.01)
    print(f'{name}: {spec.width:.1f} x {spec.north - spec.south:.1f} mm -> {out}')


if __name__ == '__main__':
    main()
