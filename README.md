# hector_stability_metrics

A header only collection of implementations of stability measures.

Functions where the fast implementation is not auto-differentiable are namespaced
in `non_differentiable` with a more computationally expensive differentiable
alternative implementation in the namespace `differentiable`.

## Metrics

Headers are in `hector_stability_metrics/include/hector_stability_metrics/metrics/`.
Each takes a support polygon (points ordered clockwise viewed from above) and
the center of mass, and fills one stability value per edge. `...Value` returns
the minimum over all edges, `...LeastStableEdgeIndex` the index of that edge.

- `computeStaticStabilityMargin`: Static Stability Margin (McGhee, Frank, 1968).
- `non_differentiable::computeForceAngleStabilityMeasure`: Force-Angle Stability
  Measure (Papadopoulos, Rey, 1996). Takes the sum of external forces acting on
  the center of mass.
- `computeNormalizedEnergyStabilityMargin`: Normalized Energy Stability Margin
  (Hirose et al.). Signed by default so that unstable edges are negative; pass
  `math::constantOneSignum` as `signFunction` for the original unsigned form.
- `computeNormalizedDynamicEnergyStabilityMargin`: Normalized Dynamic Energy
  Stability Margin (Garcia, De Santos, 2005). Takes the normalized resultant
  `(g - a_com) / |g|`, which is `(0, 0, -1)` in the static case, where it
  reduces to the NESM. Signed like the NESM.
