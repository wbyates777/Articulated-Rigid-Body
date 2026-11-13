# Articulated Rigid Body and libccd

This subdirectory contains a local copy of **[libccd](https://github.com/danfis/libccd)**, integrated directly into **ARB**.


## Why Embed `libccd`?
libccd is an established library for collision detection, but its official build scripts and CMake configuration have remained largely unmaintained 
for several years. Relying on an external legacy build system introduces CMake version conflicts, dependency bottlenecks, and failures across automated CI pipelines.
Embedding `libccd` directly into the repository solves these issues. In addition, it facilitates future code enhancements, performance profiling, etc.

## Directory Structure
* `CCD/`: Contains the core C source files alongside a lowercase `ccd/` public header subfolder, ensuring strict compatibility with existing internal include paths (`#include <ccd/ccd.h>`).

## License
This embedded copy retains its original licensing terms (BSD-3-Clause) included here. 


