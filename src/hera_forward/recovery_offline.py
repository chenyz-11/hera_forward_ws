#!/usr/bin/env python3
"""Generate an offline dropout/recovery candidate from a recorded pose YAML."""
import argparse
import json

from hera_forward.forward_logic import load_initial_poses
from hera_forward.recovery_scenario import make_scenario


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-poses', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = make_scenario(load_initial_poses(args.initial_poses))
    result['disclaimer'] = ('Offline geometric candidate only; velocity changes are instantaneous. '
                            'Platform stop, motion feasibility and execution are not verified.')
    result['initial_poses_file'] = args.initial_poses
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(f"dropout=ugv{result['dropout']} rescuer=ugv{result['rescuer']} "
          f"minimum_separation={result['minimum_separation'][0]:.3f}m "
          f"duration={result['times_s'][-1]:.3f}s output={args.output}")


if __name__ == '__main__':
    main()
