# flo-imops

`flo-imops` performs FLO's experimental Mono8 threshold-and-moment detection.
It receives frames from the embedded Strand Camera host, while `flo`
owns delivery of resulting centroids to the FLO controller and preview
annotations.

The SIMD kernels come from Strand-Braid's standalone `imops` crate. This crate
contains the transport-independent FLO processing types and has no networking
or camera-runtime responsibilities.
