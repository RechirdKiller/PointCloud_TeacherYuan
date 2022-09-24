#ifndef _PCA_ICP_H_
#define _PCA_ICP_H_

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CPCA_ICP {
protected:
	double calculate_min_dist(Eigen::MatrixXd P, Eigen::MatrixXd Q);
	Eigen::MatrixXd rigidTransform(Eigen::MatrixXd A, Eigen::MatrixXd R, Eigen::VectorXd T);
	int QKPass(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high);
	void QKSort(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high);
	Eigen::VectorXi sort(Eigen::MatrixXd& A);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(Eigen::MatrixXd xyzPoints);
	void print4x4Matrix(const Eigen::Matrix4d & matrix);
	void pcshowpair(PointCloudT::Ptr ptCloudfixed, PointCloudT::Ptr ptCloudA, PointCloudT::Ptr ptCloudB, PointCloudT::Ptr ptCloudC, PointCloudT::Ptr ptCloudD);

public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointnew;
	Eigen::MatrixXd tformICP;
	pcl::PointCloud<pcl::PointXYZ>::Ptr movingRegICP, movingRegI, movingReg;
	int PCA_ICP(PointCloudT::Ptr pointCloudmoving, PointCloudT::Ptr pointCloudfixed);
};

#endif  /* _PCA_ICP_H_ */
