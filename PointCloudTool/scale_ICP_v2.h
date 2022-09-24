#ifndef _SCALE_ICP_V2_H_
#define _SCALE_ICP_V2_H_

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Cscale_ICP_v2 {
protected:
	Eigen::VectorXi sort(Eigen::MatrixXd& A);
	void BubbleSort(Eigen::MatrixXd& D, Eigen::MatrixXd& V, int n);
	int sign(double x);
	double calculate_min_dist(Eigen::MatrixXd P, Eigen::MatrixXd Q);
	Eigen::VectorXd Crossproduct(Eigen::VectorXd a, Eigen::VectorXd u);
	int QKPass(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high);
	void QKSort(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(Eigen::MatrixXd xyzPoints);

public:
	int nargin;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointout;
	unsigned iter;
	double Errn;
	double s;
	Eigen::MatrixXd R;
	Eigen::VectorXd T;
	int scale_ICP_v2(PointCloudT::Ptr pointmoving, PointCloudT::Ptr pointfixed, int flag);
	Cscale_ICP_v2() { nargin = 3; };
};

class Cknnsearch {
public:
	Eigen::VectorXi idx;
	Eigen::VectorXd dist;
	int knnsearch(Eigen::MatrixXd X, Eigen::MatrixXd Y);
};

class CmultiQueryKNNSearchImpl {
public:
	Eigen::MatrixXi indices; //index值比matlab中的数值少1
	Eigen::MatrixXd dists;
	Eigen::VectorXi valid;
	int multiQueryKNNSearchImpl(PointCloudT::Ptr cloud, Eigen::MatrixXd points, int K);
};

class CcomputeRigidTransform {
protected:
	int sign(double x);

public:
	Eigen::MatrixXd R;
	Eigen::VectorXd t;
	int computeRigidTransform(Eigen::MatrixXd p, Eigen::MatrixXd q);
};

#endif  /* _SCALE_ICP_V2_H_ */
