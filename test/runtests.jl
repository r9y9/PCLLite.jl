module PassThrough

using PCLLite
using Cxx
using Base.Test

cxx"""
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
"""

cloud = icxx"""
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    """
cloud_filtered = icxx"""
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
"""

# fill in the cloud data
icxx"$cloud->width = 5;"
icxx"$cloud->height = 1;"
icxx"$cloud->points.resize($cloud->width * $cloud->height);"

size = convert(Int, icxx"$cloud->points.size();")
for i in 0:size-1
    icxx"""
    $cloud->points[$i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    $cloud->points[$i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    $cloud->points[$i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    """
end

icxx"""
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < $cloud->points.size (); ++i)
    std::cerr << "    " << $cloud->points[i].x << " "
                        << $cloud->points[i].y << " "
                        << $cloud->points[i].z << std::endl;
"""

# Create the filtering object
pass = icxx"pcl::PassThrough<pcl::PointXYZ>();"
icxx"""
$pass.setInputCloud($cloud);
$pass.setFilterFieldName("z");
$pass.setFilterLimits(0.0, 1.0);
$pass.filter(*($cloud_filtered));
"""

icxx"""
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < $cloud_filtered->points.size (); ++i)
    std::cerr << "    " << $cloud_filtered->points[i].x << " "
                        << $cloud_filtered->points[i].y << " "
                        << $cloud_filtered->points[i].z << std::endl;
"""

Base.length{T}(cloud::cxxt"boost::shared_ptr<pcl::PointCloud<$T>>") =
    icxx"$cloud->points.size();"
@test length(cloud_filtered) <= length(cloud)


end # module PassThrough
