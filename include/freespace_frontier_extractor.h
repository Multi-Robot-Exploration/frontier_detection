#ifndef _OCTOMAP_FRONTIERS3D_FREESPACE_FRONTIERS_EXTRACTOR_H_
#define _OCTOMAP_FRONTIERS3D_FREESPACE_FRONTIERS_EXTRACTOR_H_

//#include <octomap_frontiers3d/FreeSpaceFrontierRepresentative.h>
//#include <octomap_frontiers3d/util/octomap_frontiers3d_util.h>
#include <freespace_frontier_representative.h>
#include <octomap/octomap.h>
#include <pcl/pcl_base.h>
#include <pcl/common/pca.h>
#include <pcl/search/kdtree.h>
#include <frontier_detection/frontier_srv.h>
#include <ros/ros.h>

/*!
  \namespace octomap_frontier3D
  \brief Namespace for extracting frontiers from 3D OctoMap  
 */
namespace octomap_frontiers3d
{

/*!
  \class FreeSpaceFrontierExtractor
  \brief Impements the algorithm that extracts the free-space frontiers from an OctoMap
 */
class FreeSpaceFrontierExtractor
{
  public:
    typedef pcl::IndicesPtr FVClusterIndicesPtr;

    typedef std::vector<pcl::PointXYZ> *PointXYZListPtr;

    template <typename pointtype>
    pointtype Point3DOctomapToPCL(const octomap::point3d &point)
    {
        return pointtype(point.x(), point.y(), point.z());
    }

    bool extractorCallback(frontier_detection::frontier_srv::Request &req,
                           frontier_detection::frontier_srv::Response &res);

    /*!
      \brief Costructor
      
      \param[in] octree The pointer to octree storing the 3D occupancy map
     */
    FreeSpaceFrontierExtractor(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private,
                               double cluster_radius = 0.25,
                               double neighborhood_radius = 0.3);

    /*!
      \brief Deconstructor
     */
    virtual ~FreeSpaceFrontierExtractor();

    void SetOcTree(octomap::OcTree **octree);

    /*!
      \brief Extracts the free space frontier representatives and optionally all free space frontier voxels
      
      \param[out] fs_frep vector of detected free space frontier reprsentatives
      \param[out] freespace_frontier_voxels shared pointer to vector of detected free space frontier voxels
     */
    void Extract(std::vector<octomap_frontiers3d::FreeSpaceFrontierRepresentative> &fs_freps,
                 PointXYZListPtr freespace_frontier_voxels = PointXYZListPtr());

    /*!
      \brief Set the radius of the detected free space frontier voxel clusters
      
      \param[in] cluster_radius The radius of the cluster
     */
    void SetClusterRadius(double cluster_radius);

    /*!
      \brief Get the defined radius of the clusters
     */
    double GetClusterRadius();

    /*!
      \brief Set the radius defining the neighborhood of a voxel
      
      \param[in] neighborhood_radius 
     */
    void SetVoxelNeighborhoodRadius(double neighborhood_radius);

    /*!
      \brief Get the radius set to define the neighborhood of a voxel
     */
    double GetVoxelNeighborhoodRadius();

    octomap::KeySet *GetFreeVoxelKeys()
    {
        return free_voxels_;
    }

  private:
    octomap::OcTree *octree_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    double cluster_radius_;      /**< The radius of the ball enclosing the cluster */
    double resolution_;          /**< The resolution of the voxel grid */
    double neighborhood_radius_; /**< The voxel neighborhood radius */

    int min_no_of_voxels_in_cluster_; /**< The minimum number of voxels to be in a cluster, calculated based on cluster radius and voxel resolution */

    octomap::KeySet freespace_frontier_voxel_keys_; /**< The set of detected free space frontier voxels */
    std::vector<octomap_frontiers3d::FreeSpaceFrontierRepresentative> fs_freps_;
    PointXYZListPtr fs_frontier_voxels_;
    octomap::KeySet *free_voxels_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr freespace_frontier_voxel_centers_cloud_; /**< Pointcloud represetnation of the detected free space frontier voxels */

    geometry_msgs::Vector3 cube_scale_;

    ros::ServiceServer extractorService_;

    /*!
      \brief Detect frontier voxels
      
      Iterates through all the free voxels of the 3D Map and detects
      free space frontier voxels. Their keys are stored in freespace_frontier_voxel_keys.
      A point cloud is also generated
     */
    void DetectFrontierVoxels();

    /*!
      \brief Cluster the frontier voxels

      \param[out] clusters Vectors of detected cluster indices
      \param[out] fs_freps Extracted frontier representatives
     */
    void ClusterFrontierVoxels(std::vector<FVClusterIndicesPtr> &clusters,
                               std::vector<octomap_frontiers3d::FreeSpaceFrontierRepresentative> &fs_freps);

    /*!
      \brief Checks if the voxel cluster has reached the specified radius

      \param[in] cluster_frontier Frontier/boundary voxels of the cluster
      \return true if cluster has reached the required radius
     */
    bool VoxelClusterReachedRadiusLimit(const octomap::KeySet &cluster_frontier, const octomap::KeySet &cluster);

    /*!
      \brief Compute the principal components of the cluster's boundary/frontier points

      \param[in] cluster_frontier octomap keys of the cluster's current boundary points
      \param[out] mean Mean of the cluster
      \param[out] evs Eigen values of the principal components, in descending order
      \param[out] pcs Principal components, in discending order
     */
    void ComputeClusterPrincipalComponents(const octomap::KeySet &cluster_frontier, Eigen::Vector3f &mean, Eigen::Vector3f &evs, Eigen::Matrix3f &pcs);

    /*!
      \brief Generate the free space frontier representative
     */
    octomap_frontiers3d::FreeSpaceFrontierRepresentative GenerateFreeSpaceFrontierRepresentative(FVClusterIndicesPtr &cluster_indices, pcl::search::KdTree<pcl::PointXYZ>::Ptr &nn_tree);

    /*!
      \brief Update the cluster's size threshold

      The threshold is computed as number of voxels. This is computed as 50% of the number of voxels required to cover the surface area of the cluster. This area is approximated using the cluster's radius.
     */
    void UpdateClusterSizeThreshold();

    /*!
      \brief Orient the normal vector of the free space frontier cluster to direct towards the empty space
     */
    void OrientFreeSpaceFrontierNormalTowardsEmptySpace(octomap_frontiers3d::FreeSpaceFrontierRepresentative &fs_frep);

    uint32_t ExtractFreeVoxelKeys(octomap::OcTree *octree, octomap::KeySet &free_voxels);

    void ExtractAllBasicVoxels(octomap::OcTreeKey key, uint32_t depth, uint32_t max_tree_depth, octomap::KeySet &key_set);
};

} // ~namespace

#endif
