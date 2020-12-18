# hector_stability_metrics

A header only collection of implementations of stability measures.

Functions where the fast implementation is not auto-differentiable are namespaced
in `non_differentiable` with a more computationally expensive differentiable
alternative implementation in the namespace `differentiable`.
