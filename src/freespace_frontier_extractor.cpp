#include <freespace_frontier_extractor.h>
#include <freespace_frontier_representative.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include <iostream>

using namespace octomap_frontiers3d;

//constructor
FreeSpaceFrontierExtractor::FreeSpaceFrontierExtractor(const ros::NodeHandle &nh,
                                                       const ros::NodeHandle &nh_private,
                                                       double cluster_radius,
                                                       double neighborhood_radius)

    : nh_(nh),
      nh_private_(nh_private),
      cluster_radius_(cluster_radius),
      neighborhood_radius_(neighborhood_radius),
      free_voxels_(NULL)

{
    //UpdateClusterSizeThreshold();

    extractorService_ = nh_.advertiseService("freespace_frontier_detector",
                                             &octomap_frontiers3d::FreeSpaceFrontierExtractor::extractorCallback,
                                             this);
}

//destructor
FreeSpaceFrontierExtractor::~FreeSpaceFrontierExtractor()
{
    if (free_voxels_ != NULL)
        delete free_voxels_;
}

bool FreeSpaceFrontierExtractor::extractorCallback(frontier_detection::frontier_srv::Request &req,
                                                   frontier_detection::frontier_srv::Response &res)
{
    ros::Time computationTime = ros::Time::now();
    // Check that planner is ready to compute path.
    if (!ros::ok())
    {
        ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
        return true;
    }

    /////////////////////////////////////////
    ///add the code here.
    /////////////////////////////////////////

    ROS_INFO("frontier extraction computation lasted %2.3fs", (ros::Time::now() - computationTime).toSec());
    return true;
}

//setter api
void FreeSpaceFrontierExtractor::SetOcTree(octomap::OcTree **octree)
{
    octree_ = *octree;
    resolution_ = octree_->getResolution();
    UpdateClusterSizeThreshold();
}

// api method
void FreeSpaceFrontierExtractor::Extract(std::vector<octomap_frontiers3d::FreeSpaceFrontierRepresentative> &fs_freps,
                                         PointXYZListPtr freespace_frontier_voxels)
{
    if (!octree_)
        return;

    DetectFrontierVoxels();
    std::vector<FVClusterIndicesPtr> clusters;
    ClusterFrontierVoxels(clusters, fs_freps);
    fs_freps_ = fs_freps;
    if (freespace_frontier_voxels)
        BOOST_FOREACH (pcl::PointXYZ &p, freespace_frontier_voxel_centers_cloud_->points)
            freespace_frontier_voxels->push_back(p);
}

// internal method
void FreeSpaceFrontierExtractor::DetectFrontierVoxels()
{
    uint32_t tree_depth = octree_->getTreeDepth();

    if (free_voxels_)
        delete free_voxels_;
    free_voxels_ = new octomap::KeySet();
    ExtractFreeVoxelKeys(octree_, *free_voxels_);

    freespace_frontier_voxel_centers_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (octomap::KeySet::iterator it = free_voxels_->begin();
         it != free_voxels_->end(); it++)
    {
        octomap::OcTreeKey key = *it;
        bool is_frontier = false;
        // iterate through all the 26 neighboring voxels
        for (int i = -1; i <= 1 && !is_frontier; i++)
        {
            for (int j = -1; j <= 1 && !is_frontier; j++)
            {
                for (int k = -1; k <= 1 && !is_frontier; k++)
                {
                    if (i == 0 && j == 0 && k == 0)
                        continue;

                    octomap::OcTreeKey nkey;
                    nkey.k[0] = key.k[0] + i;
                    nkey.k[1] = key.k[1] + j;
                    nkey.k[2] = key.k[2] + k;

                    if (!octree_->search(nkey, tree_depth))
                    {
                        is_frontier = true;
                        freespace_frontier_voxel_keys_.insert(key);
                        pcl::PointXYZ vcp = Point3DOctomapToPCL<pcl::PointXYZ>(octree_->keyToCoord(key));
                        freespace_frontier_voxel_centers_cloud_->points.push_back(vcp);
                    }
                }
            }
        }
    }

    freespace_frontier_voxel_centers_cloud_->is_dense = true;
    freespace_frontier_voxel_centers_cloud_->height = 1;
    freespace_frontier_voxel_centers_cloud_->width = freespace_frontier_voxel_centers_cloud_->points.size();
}

//internal method
void FreeSpaceFrontierExtractor::ClusterFrontierVoxels(std::vector<FVClusterIndicesPtr> &clusters, std::vector<octomap_frontiers3d::FreeSpaceFrontierRepresentative> &fs_freps)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr nn_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    nn_tree->setInputCloud(freespace_frontier_voxel_centers_cloud_);
    //double cluster_radius_2 = cluster_radius_*cluster_radius_;

    octomap::KeySet key_marker;
    for (octomap::KeySet::iterator it = freespace_frontier_voxel_keys_.begin();
         it != freespace_frontier_voxel_keys_.end(); it++)
    {
        octomap::OcTreeKey key = *it;
        if (key_marker.find(key) != key_marker.end())
            continue; // key is already processed to be in a cluster

        // create free space frontier voxel cluster
        FVClusterIndicesPtr cluster_indices(new std::vector<int>);
        octomap::KeySet cluster_marker;
        octomap::KeySet cluster_frontier;
        std::queue<octomap::OcTreeKey> Q;

        // initialize clustering with seed voxel key
        Q.push(key);
        cluster_marker.insert(key);   // mark key as used
        cluster_frontier.insert(key); // add key to cluster's frontier (i.e. voxels at the boundary of cluster that are not expanded)
        octomap::point3d q = octree_->keyToCoord(key);
        pcl::PointXYZ qq = Point3DOctomapToPCL<pcl::PointXYZ>(q);
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        nn_tree->nearestKSearch(qq, 1, k_indices, k_sqr_distances);
        cluster_indices->push_back(k_indices[0]); // add voxel's PCL cloud index

        while (!Q.empty())
        {
            octomap::OcTreeKey ckey = Q.front();
            Q.pop();
            octomap::point3d p = octree_->keyToCoord(ckey);
            pcl::PointXYZ pp = Point3DOctomapToPCL<pcl::PointXYZ>(p);

            // expand this point using its neighbors
            int expanded_no = 0;
            int total_neighboring_frontier_voxels = 0;
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    for (int k = -1; k <= 1; k++)
                    {
                        if (i == 0 && j == 0)
                            continue;
                        octomap::OcTreeKey nkey; // generate neighboring voxel key
                        nkey.k[0] = ckey.k[0] + i;
                        nkey.k[1] = ckey.k[1] + j;
                        nkey.k[2] = ckey.k[2] + k;

                        if (freespace_frontier_voxel_keys_.find(nkey) == freespace_frontier_voxel_keys_.end())
                            continue;

                        // if the neighboring voxel is a freespace frontier and
                        // is not added to this cluter or any previous cluster
                        // then add to this cluster

                        // if voxel is neither added to this cluster nor to a different cluster
                        if (cluster_marker.find(nkey) == cluster_marker.end() &&
                            key_marker.find(nkey) == key_marker.end())
                        {
                            Q.push(nkey);
                            // add to cluster
                            cluster_marker.insert(nkey);
                            // since this is newly added, its at the frontier of the cluster
                            cluster_frontier.insert(nkey);
                            expanded_no++;

                            octomap::point3d np = octree_->keyToCoord(nkey);
                            pcl::PointXYZ npp = Point3DOctomapToPCL<pcl::PointXYZ>(np);
                            nn_tree->nearestKSearch(npp, 1, k_indices, k_sqr_distances); // find nkey's PCL index
                            cluster_indices->push_back(k_indices[0]);
                        }
                        total_neighboring_frontier_voxels++;
                    }
                }
            }
            if (expanded_no == 0) // if voxel is not expanded, no need to check for size of the cluster
                continue;

            if (expanded_no == total_neighboring_frontier_voxels)
                cluster_frontier.erase(ckey); // remove the voxel from the cluster's frontier

            if (cluster_frontier.size() > 3 && VoxelClusterReachedRadiusLimit(cluster_frontier, cluster_marker))
                break;
        }

        // mark all the voxels added to cluster
        key_marker.insert(cluster_marker.begin(), cluster_marker.end());

        if (cluster_indices->size() >= min_no_of_voxels_in_cluster_)
        {
            octomap_frontiers3d::FreeSpaceFrontierRepresentative fs_frep = GenerateFreeSpaceFrontierRepresentative(cluster_indices, nn_tree);
            fs_freps.push_back(fs_frep);
        }
    }
}

bool FreeSpaceFrontierExtractor::VoxelClusterReachedRadiusLimit(const octomap::KeySet &cluster_frontier, const octomap::KeySet &cluster)
{
    Eigen::Vector3f mean, evs;
    Eigen::Matrix3f pcs;
    ComputeClusterPrincipalComponents(cluster_frontier, mean, evs, pcs);

    Eigen::Vector3f pc_0 = pcs.col(0); // retrieve the 1st principal component
    // project each voxel point to this principal component and find the maximum
    // distance from mean of cluster to the frontier along principal component
    for (octomap::KeySet::const_iterator cfi = cluster_frontier.begin();
         cfi != cluster_frontier.end(); cfi++)
    {
        pcl::PointXYZ p = Point3DOctomapToPCL<pcl::PointXYZ>(octree_->keyToCoord(*cfi));
        double x = fabs((p.getVector3fMap() - mean).dot(pc_0));
        if (x > cluster_radius_) // if the projected distance is larger than cluster radius, stop growing the cluster
            return true;
    }

    return false;
}

void FreeSpaceFrontierExtractor::ComputeClusterPrincipalComponents(const octomap::KeySet &cluster_frontier, Eigen::Vector3f &mean, Eigen::Vector3f &evs, Eigen::Matrix3f &pcs)
{
    // retrieve the point vectors and the mean vector
    std::vector<Eigen::Vector3f> cfps;
    mean = Eigen::Vector3f::Zero();
    for (octomap::KeySet::const_iterator cfi = cluster_frontier.begin();
         cfi != cluster_frontier.end(); cfi++)
    {
        pcl::PointXYZ p = Point3DOctomapToPCL<pcl::PointXYZ>(octree_->keyToCoord(*cfi));
        cfps.push_back(p.getVector3fMap());
        mean += p.getVector3fMap();
    }
    mean /= cfps.size();

    // compute the covariance matrix
    Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
    BOOST_FOREACH (Eigen::Vector3f p, cfps)
        A += (p - mean) * ((p - mean).transpose());
    A /= cfps.size();

    // do eigen analysis of covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(A);

    // sort eigen vectors from first principal component
    Eigen::Matrix3f V;
    for (int i = 0; i < 3; i++)
        V.col(i) = es.eigenvectors().col(2 - i);

    pcs = V;
    evs = es.eigenvalues();
}

//api method
octomap_frontiers3d::FreeSpaceFrontierRepresentative
FreeSpaceFrontierExtractor::GenerateFreeSpaceFrontierRepresentative(FVClusterIndicesPtr &cluster_indices, pcl::search::KdTree<pcl::PointXYZ>::Ptr &nn_tree)
{
    octomap_frontiers3d::FreeSpaceFrontierRepresentative fs_frep;

    // compute the cluster's centroid
    Eigen::Vector3f pv_sum(0, 0, 0);
    BOOST_FOREACH (int idx, *cluster_indices)
    {
        pv_sum += freespace_frontier_voxel_centers_cloud_->points[idx].getVector3fMap();
    }
    pv_sum /= cluster_indices->size();
    fs_frep.p.x = pv_sum.x();
    fs_frep.p.y = pv_sum.y();
    fs_frep.p.z = pv_sum.z();

    // compute the centroid's normal
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // get the neighboring free space frontier voxels of cluster's centroid voxel
    pcl::PointXYZ cp;
    cp.x = fs_frep.p.x;
    cp.y = fs_frep.p.y;
    cp.z = fs_frep.p.z;
    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_distances;
    nn_tree->radiusSearch(cp, neighborhood_radius_, nn_indices, nn_sqr_distances);
    float nx, ny, nz, curvature;
    ne.computePointNormal(*freespace_frontier_voxel_centers_cloud_,
                          nn_indices,
                          nx, ny, nz, curvature);

    fs_frep.n.x = nx;
    fs_frep.n.y = ny;
    fs_frep.n.z = nz;

    // orient the normal vectors towards empty space
    OrientFreeSpaceFrontierNormalTowardsEmptySpace(fs_frep);

    fs_frep.nvx = cluster_indices->size();

    return fs_frep;
}

//internal method
void FreeSpaceFrontierExtractor::UpdateClusterSizeThreshold()
{
    double voxel_face_area = pow(resolution_, 2);
    double expected_cluster_area = M_PI * cluster_radius_ * cluster_radius_;
    min_no_of_voxels_in_cluster_ = static_cast<int>(floor((0.5 * expected_cluster_area) / voxel_face_area));
}

//internal method
void FreeSpaceFrontierExtractor::OrientFreeSpaceFrontierNormalTowardsEmptySpace(octomap_frontiers3d::FreeSpaceFrontierRepresentative &fs_frep)
{
    Eigen::Vector3f sp(fs_frep.p.x, fs_frep.p.y, fs_frep.p.z);    // source point
    Eigen::Vector3f n_pos(fs_frep.n.x, fs_frep.n.y, fs_frep.n.z); // positive normal
    Eigen::Vector3f n_neg = -n_pos;                               // negative normal
    Eigen::Vector3f n = n_pos;                                    // resultant normal, initialized to positive normal

    double step_size = octree_->getResolution();
    int limit = static_cast<int>(ceil(cluster_radius_ / step_size)); // number of times the stepping should be conducted
    int i = 0;
    while (i < limit)
    {
        i++;
        // step in positive and negative direction of the normal vector
        Eigen::Vector3f p1 = sp + i * step_size * n_pos;
        Eigen::Vector3f p2 = sp + i * step_size * n_neg;

        // find voxels containing p1 and p2
        octomap::OcTreeNode *n1 = octree_->search(p1.x(), p1.y(), p1.z());
        octomap::OcTreeNode *n2 = octree_->search(p2.x(), p2.y(), p2.z());

        if (n1 == NULL && n2 == NULL)
            continue;
        else if (n1 == NULL) // if p1 is in unknown
        {
            n = n_pos;
            break;
        }
        else if (n2 == NULL) // if p2 is in unknown
        {
            n = n_neg;
            break;
        }
    }

    // update the normal vector
    fs_frep.n.x = n.x();
    fs_frep.n.y = n.y();
    fs_frep.n.z = n.z();
}

//setter
void FreeSpaceFrontierExtractor::SetClusterRadius(double cluster_radius)
{
    cluster_radius_ = cluster_radius;
    UpdateClusterSizeThreshold();
}

//getter
double
FreeSpaceFrontierExtractor::GetClusterRadius()
{
    return cluster_radius_;
}

//setter
void FreeSpaceFrontierExtractor::SetVoxelNeighborhoodRadius(double neighborhood_radius)
{
    neighborhood_radius_ = neighborhood_radius;
}

//gettter
double
FreeSpaceFrontierExtractor::GetVoxelNeighborhoodRadius()
{
    return neighborhood_radius_;
}

//internal method
uint32_t FreeSpaceFrontierExtractor::ExtractFreeVoxelKeys(octomap::OcTree *octree,
                                                          octomap::KeySet &free_voxels)
{
    free_voxels.clear();
    uint32_t vox_count = 0;

    uint32_t tree_depth = octree->getTreeDepth();

    for (octomap::OcTree::iterator it = octree->begin(tree_depth),
                                   end = octree->end();
         it != end; it++)
    {
        if (octree->isNodeOccupied(*it))
            continue;

        octomap::KeySet key_set;
        uint32_t depth = it.getDepth();
        octomap::OcTreeKey key = it.getKey();

        if (depth < tree_depth)
            ExtractAllBasicVoxels(key, depth, tree_depth, key_set);
        else
            key_set.insert(key);

        free_voxels.insert(key_set.begin(), key_set.end());

        vox_count += key_set.size();
    }

    return vox_count;
}

//internal method
void FreeSpaceFrontierExtractor::ExtractAllBasicVoxels(octomap::OcTreeKey key,
                                                       uint32_t depth,
                                                       uint32_t max_tree_depth,
                                                       octomap::KeySet &key_set)
{
    // get the leaf voxel's (cube's) size in smallest voxels (i.e. resolution)
    int cube_size_in_voxels = (max_tree_depth - depth) << 1;

    int s = cube_size_in_voxels / 2;

    // for voxels with depth < max_tree_depth, their key corresponds is
    // generated from bottom south west coordinate of the top north east octant

    for (int i = -s; i < s; i++)
    {
        for (int j = -s; j < s; j++)
        {
            for (int l = -s; l < s; l++)
            {
                octomap::OcTreeKey nkey;
                nkey.k[0] = key.k[0] + i;
                nkey.k[1] = key.k[1] + j;
                nkey.k[2] = key.k[2] + l;
                key_set.insert(nkey);
            }
        }
    }
}