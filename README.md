# PCLLite

A minimum version of [r9y9/PCL.jl](https://github.com/r9y9/PCL.jl)

PCLLite is designed to be minimum. PCLLite doesn't provide any glue functions
and types to PCL in contrast to PCL.jl. Instead, PCLLite provides a minimum
setup to interact with PCL using Cxx.jl; i.e only adds necessary header search
paths to clang, and includes PCL base headers into the current module.


## How it works

Once you have `using PCLLite; using Cxx`, you can call PCL functions as needed:

```jl
using PCLLite
using Cxx

cxx"""
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
"""

cloud = icxx"pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    pcl::PointCloud<pcl::PointXYZ>);"
pass = icxx"pcl::PassThrough<pcl::PointXYZ>();"
```
