# IDCBS
## Disjoint/non-disjoint (E)CBSH and ID(E)CBS, integrated with LPA* and other incremental techniques

Different #define directives allow building an executable that runs either ID(E)CBS or (E)CBS, with or without incremental techniques in the low level, on a single MAPF instance.

The resulting executable can optionally be put into a Docker image using the provided Dockerfile.

This project currently links against Gurobi, so a Gurobi license is needed. Gurobi integration can be turned off using a #define directive.

Then, the included Python scripts can be used to run the executable or the Docker image on a set of MAPF instances.
