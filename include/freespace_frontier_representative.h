#ifndef _OCTOMAP_FRONTIERS3D_FREESPACE_FRONTIERS_REPRESENTATIVE_H_
#define _OCTOMAP_FRONTIERS3D_FREESPACE_FRONTIERS_REPRESENTATIVE_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace octomap_frontiers3d
{
    class FreeSpaceFrontierRepresentative{

    public:
        uint32_t nvx;
        geometry_msgs::Point p;
        geometry_msgs::Vector3 n;

        FreeSpaceFrontierRepresentative();
        virtual ~FreeSpaceFrontierRepresentative();
    };
}

#endif

// # the centroid of the freespace frontier voxel cluster
// geometry_msgs/Point p
// # the normal to the freespace frontier voxel surface at centroid
// geometry_msgs/Vector3 n
// # number of voxels in the freespace frontier cluster
// uint32 nvx