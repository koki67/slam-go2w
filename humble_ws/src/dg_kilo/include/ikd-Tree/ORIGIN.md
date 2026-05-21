# ikd-Tree — Provenance

## Upstream

- **Repository**: `hku-mars/ikd-Tree`
- **License**: Apache-2.0
- **Vendored via**: `humble_ws/src/fast_lio_ros2/include/ikd-Tree/` (same commit)

## Attribution

This is a verbatim copy of the ikd-Tree library from the FAST-LIO2 workspace.
No modifications have been made. The original authors are:
  Yixi Cai, Wei Xu, Fu Zhang — HKU MARS Lab.

See https://github.com/hku-mars/ikd-Tree for the upstream repository.

## Rationale

ikd-Tree is required by DG-KILO's map management (scan-to-map NN search,
ground-plane 5-NN lookup, sliding-window box deletion). Rather than adding a
new dependency entry pointing at the upstream, it is vendored here to stay
in sync with the version already in this workspace (fast_lio_ros2).
