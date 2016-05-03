__precompile__(false)

module PCLLite

const VERBOSE = Bool(parse(Int, get(ENV, "PCLLITEJL_VERBOSE", "0")))

const BOOST_INCLUDE_DIR = get(ENV, "BOOST_INCLUDE_DIR", "/usr/local/include")
const FLANN_INCLUDE_DIR = get(ENV, "FLANN_INCLUDE_DIR", "/usr/local/include")
const EIGEN_INCLUDE_DIR = get(ENV, "EIGEN_INCLUDE_DIR",
    "/usr/local/include/eigen3")

searchdir(path, key) = filter(x->contains(x, key), readdir(path))

const VTK_INCLUDE_DIR_PARENT = get(ENV, "VTK_INCLUDE_DIR_PARENT",
    "/usr/local/include")

# Search GUI backend
vtk_dirs = searchdir(VTK_INCLUDE_DIR_PARENT, "vtk-")
const VTK_INCLUDE_DIR = get(ENV, "VTK_INCLUDE_DIR",
    isempty(vtk_dirs) ? "" : joinpath(VTK_INCLUDE_DIR_PARENT, vtk_dirs[1]))

const has_vtk_backend = isfile(joinpath(VTK_INCLUDE_DIR, "vtkVersion.h"))
if has_vtk_backend
    VERBOSE && info("vtk include directory found: $VTK_INCLUDE_DIR")
end

using BinDeps

# Load dependency
deps = joinpath(Pkg.dir("PCLLite"), "deps", "deps.jl")
if isfile(deps)
    include(deps)
else
    error("PCLLite not properly installed. Please run Pkg.build(\"PCLLite\")")
end

VERBOSE && info("Loading Cxx.jl...")
using Cxx
using CxxStd

macro timevb(expr)
    if VERBOSE
        return Expr(:macrocall, Symbol("@time"), expr)
    else
        return expr
    end
end

VERBOSE && info("dlopen...")
for lib in [
        libpcl_common,
        libpcl_io,
        libpcl_features,
        libpcl_filters,
        libpcl_kdtree,
        libpcl_keypoints,
        libpcl_segmentation,
        libpcl_visualization,
        libpcl_recognition,
        libpcl_registration,
        libpcl_octree,
        libpcl_surface,
        libpcl_people,
        libpcl_search,
        libpcl_tracking,
        ]
    p = Libdl.dlopen_e(lib, Libdl.RTLD_GLOBAL)
    p == C_NULL && warn("Failed to load: $lib")
end

# Make sure vtk libraries are loaded before calling @cxx vtkVersion::xxx()
if has_vtk_backend
    cxxinclude(joinpath(VTK_INCLUDE_DIR, "vtkVersion.h"))
    global const vtk_version = bytestring(@cxx vtkVersion::GetVTKVersion())
    VERBOSE && info("vtk version: $vtk_version")
end


const system_include_top = "/usr/local/include"
const local_include_top = joinpath(Pkg.dir("PCL", "deps", "usr", "include"))

function get_pcl_version(top)
    dirs = searchdir(top, "pcl-")
    isempty(dirs) && error("could not find pcl directory")
    pcl_dir = dirs[1]
    return pcl_dir[5:end]
end

topdir_to_be_included = local_include_top

if isdir(local_include_top)
    VERBOSE && info("Including headers from local path: $local_include_top")
    pcl_version = get_pcl_version(local_include_top)
elseif !isempty(searchdir(joinpath(system_include_top), "pcl-"))
    VERBOSE && info("Including headers from system path: $system_include_top")
    pcl_version = get_pcl_version(system_include_top)
    topdir_to_be_included = system_include_top
else
    error("Cannot find PCL headers")
end

function add_header_dirs(top)
    # Boost (required)
    addHeaderDir(BOOST_INCLUDE_DIR, kind=C_System)

    # Eigen (required)
    addHeaderDir(EIGEN_INCLUDE_DIR, kind=C_System)

    # FLANN (required)
    addHeaderDir(FLANN_INCLUDE_DIR, kind=C_System)

    # PCL top directory
    addHeaderDir(top, kind=C_System)
    addHeaderDir(joinpath(top, "pcl"), kind=C_System)
end

function include_headers(top)
    # This is necesarry, but not sure why...
    cxxinclude("iostream")

    # top level
    VERBOSE && info("Include pcl top-level headers")
    @timevb for name in ["pcl_base.h", "point_cloud.h", "point_types.h"]
        cxxinclude(joinpath(top, "pcl", name))
    end

    # boost
    cxxinclude("boost/version.hpp")
end

VERBOSE && info("pcl_version: $pcl_version")
add_header_dirs(joinpath(topdir_to_be_included, "pcl-$pcl_version"))
include_headers(joinpath(topdir_to_be_included, "pcl-$pcl_version"))

# Check boost version
const _BOOST_VERSION = icxx"BOOST_VERSION;"
const BOOST_VERSION_MAJOR = trunc(Int, _BOOST_VERSION / 100000)
const BOOST_VERSION_MINOR = trunc(Int, _BOOST_VERSION / 100 % 1000)
const BOOST_VERSION_PATCH = trunc(Int, _BOOST_VERSION % 100)
const BOOST_VERSION = join([BOOST_VERSION_MAJOR, BOOST_VERSION_MINOR, BOOST_VERSION_PATCH], ".")
VERBOSE && info("boost version: $BOOST_VERSION")

# Check FLANN vesion
# make sure FLANN_INCLUDE_DIR is addded as kind=C_System
cxxinclude(joinpath(FLANN_INCLUDE_DIR, "flann/flann.h"))
cxx"""
namespace pcllite {
std::string getFLANNVersion() { return FLANN_VERSION_; }
}
"""
getFLANNVersion() = bytestring(@cxx pcllite::getFLANNVersion())
VERBOSE && info("FLANN version: $(getFLANNVersion())")


end # module PCLLite
