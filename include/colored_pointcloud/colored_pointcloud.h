#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointXYZRGBI
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    float intensity;     //intensity
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (uint8_t,r,r)
                                  (uint8_t,g,g)
                                  (uint8_t,b,b)
                                  (float, rgb, rgb)
                                  (float,intensity,intensity)
)

struct PointXYZIRT
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    uint8_t intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (uint8_t,intensity,intensity)
                                  (uint16_t,ring,ring)
                                  (double,timestamp,timestamp)
)
typedef pcl::PointCloud<PointXYZIRT> rslidarPointCloud;