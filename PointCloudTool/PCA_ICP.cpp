#pragma warning(disable:4819)
#pragma warning(disable:4996)

#include <vector>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc
#include <Eigen/Dense>
#include <Eigen/Core>

#include "PCA_ICP.h"
#include "scale_ICP_v2.h"

class Cpcregistericp {
public:
	Eigen::MatrixXd tform;
	PointCloudT::Ptr movingReg;
	double rmse;
	int stopIteration;
	int pcregistericp(PointCloudT::Ptr moving, PointCloudT::Ptr fixed, int maxIterations);
};

int Cpcregistericp::pcregistericp(PointCloudT::Ptr moving, PointCloudT::Ptr fixed, int maxIterations)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	PointCloudT::Ptr movingReg(new PointCloudT);  // Transformed point cloud
	*movingReg = *moving;
	icp.setMaximumIterations(maxIterations);
	icp.setInputSource(movingReg);
	icp.setInputTarget(fixed);
	icp.align(*movingReg);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << icp.getMaximumIterations() << " : cloud_icp -> cloud_in" << std::endl;
		this->tform = icp.getFinalTransformation().cast<double>();
		this->tform(0, 1) = -this->tform(0, 1); this->tform(0, 2) = -this->tform(0, 2); this->tform(1, 2) = -this->tform(1, 2);
		this->tform(1, 0) = -this->tform(1, 0); this->tform(2, 0) = -this->tform(2, 0); this->tform(2, 1) = -this->tform(2, 1);
		this->movingReg = movingReg;
		return (1);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (0);
	}
}

class Cpcregistericp_v2 {
public:
	Eigen::MatrixXd tform;
	PointCloudT::Ptr movingReg;
	double rmse;
	int stopIteration;
	int pcregistericp_v2(PointCloudT::Ptr moving, PointCloudT::Ptr fixed);

protected:
	Eigen::VectorXi sort(Eigen::MatrixXd& A);
	Eigen::MatrixXd rigidTransform(Eigen::MatrixXd A, Eigen::MatrixXd R, Eigen::VectorXd T);
	int QKPass(Eigen::MatrixXd& r, Eigen::VectorXi& I, int low, int high);
	void QKSort(Eigen::MatrixXd& r, Eigen::VectorXi& I, int low, int high);
};

int CPCA_ICP::PCA_ICP(PointCloudT::Ptr pointCloudmoving, PointCloudT::Ptr pointCloudfixed)
{
	Eigen::MatrixXd direction(8, 3);
	Eigen::MatrixXd P, Q;
	int i, k;
	Eigen::VectorXd pc(3), qc(3);
	Eigen::MatrixXd Pi, Qi, Dummy01, Hp, Hq;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd  Up, Uq, P_pca, Q_pca;
	Eigen::VectorXd Dist(8);
	Eigen::MatrixXd Pk, Rr;
	int r;
	Eigen::VectorXd Tr;
	Eigen::MatrixXd Pnew;
	CmultiQueryKNNSearchImpl multiQueryKNNSearchImpl;
	Eigen::MatrixXi indices;
	Eigen::MatrixXd dists;
	Eigen::VectorXi keepInlierA, idx;
	Eigen::VectorXd inlierDist;
	double Err_PCA;
	Eigen::MatrixXd RI_ICP;
	Eigen::VectorXd TI_ICP, TI;
	Cpcregistericp pcregistericp;
	Eigen::MatrixXd R, RI;
	Cpcregistericp_v2 pcregistericp_v2;
	Eigen::MatrixXd R_ICP;
	Eigen::VectorXd T_ICP, T;

	direction << 1, 1, 1,
		1, 1, -1,
		1, -1, 1,
		1, -1, -1,
		-1, 1, 1,
		-1, 1, -1,
		-1, -1, 1,
		-1, -1, -1;
	for (i = 0; i < pointCloudmoving->points.size(); ++i)
	{
		if (!isnan(pointCloudmoving->points[i].x))
		{
			P.conservativeResize(P.rows() + 1, 3);
			P(P.rows() - 1, 0) = pointCloudmoving->points[i].x;
			P(P.rows() - 1, 1) = pointCloudmoving->points[i].y;
			P(P.rows() - 1, 2) = pointCloudmoving->points[i].z;
		}
	}

	for (i = 0; i < pointCloudfixed->points.size(); ++i)
	{
		if (!isnan(pointCloudfixed->points[i].x))
		{
			Q.conservativeResize(Q.rows() + 1, 3);
			Q(Q.rows() - 1, 0) = pointCloudfixed->points[i].x;
			Q(Q.rows() - 1, 1) = pointCloudfixed->points[i].y;
			Q(Q.rows() - 1, 2) = pointCloudfixed->points[i].z;
		}
	}

	for (i = 0; i < P.cols(); ++i)
	{
		pc(i) = P.col(i).sum() / P.rows();
		qc(i) = Q.col(i).sum() / Q.rows();
	}

	
	Dummy01.resize(P.rows(), P.cols());
	for (i = 0; i < P.cols(); ++i)
	{
		Dummy01.col(i).setConstant(pc(i));
	}
	Pi = P - Dummy01;
	Dummy01.resize(Q.rows(), Q.cols());
	for (i = 0; i < Q.cols(); ++i)
	{
		Dummy01.col(i).setConstant(qc(i));
	}
	Qi = Q - Dummy01;
	Hp = Pi.transpose() * Pi;
	Hq = Qi.transpose() * Qi;

	/*cout << Hp << endl << endl;
	cout << Hq << endl << endl;*/

	svd.compute(Hp, Eigen::ComputeThinU | Eigen::ComputeFullV);
	Up = svd.matrixU();
	svd.compute(Hq, Eigen::ComputeThinU | Eigen::ComputeFullV);
	Uq = svd.matrixU();
	P_pca = Pi*Up;
	Q_pca = Qi*Uq;

	Dist.setZero();
	Dummy01.resize(direction.cols(), direction.cols());
	Dummy01.setZero();
	for (k = 0; k < 8; ++k)
	{
		for (i = 0; i < direction.cols(); ++i)
		{
			Dummy01(i, i) = direction(k, i);
		}
		Pk = P_pca*Dummy01;
		Dist(k) = calculate_min_dist(Pk, Q_pca);
	}
	//cout << Dist << endl << endl;
	r = 0;
	for (i = 1; i < Dist.size(); ++i)
	{
		if (Dist(r) > Dist(i))
			r = i;
	}
	for (i = 0; i < direction.cols(); ++i)
	{
		Dummy01(i, i) = direction(r, i);
	}
	Rr = Uq*Dummy01*Up.transpose();
	Tr = (qc.matrix().transpose() - pc.matrix().transpose()*Rr.transpose()).transpose().col(0);

	//cout << Rr << endl << endl;
	//cout << Tr << endl << endl;
	Pnew = rigidTransform(P, Rr, Tr);
	multiQueryKNNSearchImpl.multiQueryKNNSearchImpl(pointCloudfixed, Pnew, 1);
	indices = multiQueryKNNSearchImpl.indices;
	dists = multiQueryKNNSearchImpl.dists;

	keepInlierA.resize(P.rows());
	keepInlierA.setZero();
	idx = sort(dists);
	for (i = 0; i < P.rows(); ++i)
	{
		keepInlierA(idx(i)) = 1;
	}
	for (i = 0; i < keepInlierA.size(); ++i)
	{
		if (keepInlierA(i))
		{
			inlierDist.conservativeResize(inlierDist.size() + 1);
			inlierDist(inlierDist.size() - 1) = dists(i);
		}
	}
	Err_PCA = sqrt(inlierDist.sum() / inlierDist.size());

	cout << Err_PCA << endl << endl;
	pointnew = pointCloud(Pnew);
	pcregistericp.pcregistericp(pointnew, pointCloudfixed, 1000);
	RI_ICP = pcregistericp.tform.block(0, 0, 3, 3);
	TI_ICP = pcregistericp.tform.col(3).head(3);
	movingRegI = pcregistericp.movingReg;
	R = RI_ICP * Rr;
	RI = R.transpose();
	TI = (RI_ICP*Tr.matrix()).col(0) + TI_ICP;

	/*cout << endl << RI_ICP << endl << endl;
	cout << TI_ICP << endl << endl;
	cout << Rr << endl << endl;
	cout << R << endl << endl;
	cout << RI << endl << endl;
	cout << TI << endl << endl;*/
	pcregistericp_v2.pcregistericp_v2(pointnew, pointCloudfixed);
	R_ICP = pcregistericp_v2.tform.block(0, 0, 3, 3);
	T_ICP = pcregistericp_v2.tform.col(3).head(3);
	movingReg = pcregistericp_v2.movingReg;
	cout <<endl<< R_ICP << endl << endl;
	cout << T_ICP << endl << endl;

	R = R_ICP * Rr;
	R = R.transpose();
	T = (R_ICP * Tr.transpose()).col(0) + T_ICP;

	//cout << endl << R << endl << endl;
	//cout << T << endl << endl;


	pcregistericp_v2.pcregistericp_v2(pointCloudmoving, pointCloudfixed);
	tformICP = pcregistericp_v2.tform;
	movingRegICP = pcregistericp_v2.movingReg;

	pcshowpair(pointCloudfixed, movingRegICP, pointnew, movingReg, movingRegI);
	return 0;
}

Eigen::MatrixXd CPCA_ICP::rigidTransform(Eigen::MatrixXd A, Eigen::MatrixXd R, Eigen::VectorXd T)
{
	Eigen::MatrixXd B=A * R.transpose();
	Eigen::VectorXd Dummy01(B.rows());

	Dummy01.setConstant(T(0));
	B.col(-1 + 1) = B.col(-1 + 1) + Dummy01;
	Dummy01.setConstant(T(1));
	B.col(-1 + 2) = B.col(-1 + 2) + Dummy01;
	Dummy01.setConstant(T(2));
	B.col(-1 + 3) = B.col(-1 + 3) + Dummy01;
	return B;
}

double CPCA_ICP::calculate_min_dist(Eigen::MatrixXd P, Eigen::MatrixXd Q)
{
	Cknnsearch knnsearch;
	int m;
	double dist;

	knnsearch.knnsearch(Q, P);
	m = P.rows();
	dist = knnsearch.dist.sum() / m;
	return dist;
}

Eigen::VectorXi CPCA_ICP::sort(Eigen::MatrixXd& A)
{
	Eigen::VectorXi I;
	int i;

	I.resize(A.cols());
	for (i = 0; i < I.size(); ++i)
	{
		I(i) = i;
	}

	QKSort(A, I, 0, A.cols() - 1);
	return I;
}

void CPCA_ICP::QKSort(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high)
{
	int pos;

	if (low < high)
	{
		pos = QKPass(r, I, low, high);
		QKSort(r, I, low, pos - 1);
		QKSort(r, I, pos + 1, high);
	}
}

int CPCA_ICP::QKPass(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high)
{
	double x;
	int t;

	x = r(0, low);
	t = I(low);
	while (low < high)
	{
		while (low < high && r(0, high) >= x)
			high--;
		if (low < high) { r(0, low) = r(0, high); I(low) = I(high); low++; }
		while (low < high && r(0, low) < x)
			low++;
		if (low < high) { r(0, high) = r(0, low); I(high) = I(low); high--; }
	}
	r(0, low) = x;
	I(low) = t;
	return low;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CPCA_ICP::pointCloud(Eigen::MatrixXd xyzPoints)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Generate pointcloud data
	cloud->width = xyzPoints.rows();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		(*cloud)[i].x = xyzPoints(i, 0);
		(*cloud)[i].y = xyzPoints(i, 1);
		(*cloud)[i].z = xyzPoints(i, 2);
	}

	return cloud;
}

void
CPCA_ICP::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

int Cpcregistericp_v2::pcregistericp_v2(PointCloudT::Ptr moving, PointCloudT::Ptr fixed)
{
	double Tolerance;
	int maxIterations;
	std::vector<Eigen::MatrixXd> Rs;
	std::vector<Eigen::VectorXd> Ts;
	int i, j, k;
	Eigen::VectorXd Err, qs;
	Eigen::MatrixXd moving_Location;
	Eigen::MatrixXd locA;
	CmultiQueryKNNSearchImpl multiQueryKNNSearchImpl;
	Eigen::MatrixXi indices;
	Eigen::MatrixXd dists;
	Eigen::VectorXi keepInlierA, idx, inlierIndicesA, inlierIndicesB;
	Eigen::MatrixXd X, Z, fixed_Location;
	Eigen::VectorXd inlierDist;
	CcomputeRigidTransform computeRigidTransform;
	Eigen::MatrixXd Rn;
	Eigen::VectorXd Tn;
	Eigen::MatrixXd Dummy01;
	Eigen::VectorXd squaredError;
	Eigen::RowVectorXd Dummy02;
	int ind = 0;
	double minErr;
	Eigen::MatrixXd R;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd U, V;
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud
	Eigen::VectorXd Dummy03;
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	maxIterations = 800;
	Tolerance = 1e-6;
	Rs.resize(maxIterations + 1);
	Ts.resize(maxIterations + 1);
	for (i = 0; i < maxIterations + 1; ++i)
	{
		Rs[i].resize(3, 3);
		Rs[i].setZero();
		Ts[i].resize(3);
		Ts[i].setZero();
	}
	Err.resize(maxIterations + 1);
	Err.setZero();

	qs.resize(7);
	qs.setZero();
	qs(0) = 1;
	Rs[0](0, 0) = Rs[0](1, 1) = Rs[0](2, 2) = 1;
	
	for (i = 0; i < moving->points.size(); ++i)
	{
		if (!isnan(moving->points[i].x))
		{
			moving_Location.conservativeResize(moving_Location.rows() + 1, 3);
			moving_Location(moving_Location.rows() - 1, 0) = moving->points[i].x;
			moving_Location(moving_Location.rows() - 1, 1) = moving->points[i].y;
			moving_Location(moving_Location.rows() - 1, 2) = moving->points[i].z;
		}
	}
	locA = rigidTransform(moving_Location, Rs[0], Ts[0]);

	//cout << endl << maxIterations << endl << endl;
	
	
	for (i = 0; i < maxIterations; ++i)
	{
		multiQueryKNNSearchImpl.multiQueryKNNSearchImpl(fixed, locA, 1);

		keepInlierA.resize(moving_Location.rows());
		keepInlierA.setZero();
		idx = sort(multiQueryKNNSearchImpl.dists);
		dists = multiQueryKNNSearchImpl.dists;
		for (j = 0; j < moving_Location.rows(); ++j)
		{
			keepInlierA(idx(j)) = 1;
		}
		inlierIndicesA.resize(0);
		for (j = 0; j < keepInlierA.size(); ++j)
		{
			if (keepInlierA(j))
			{
				inlierIndicesA.conservativeResize(inlierIndicesA.size() + 1);
				inlierIndicesA(inlierIndicesA.size() - 1) = j;
			}
		}
		inlierIndicesB.resize(0);
		for (j = 0; j < multiQueryKNNSearchImpl.indices.cols(); ++j)
		{
			if (keepInlierA(j))
			{
				inlierIndicesB.conservativeResize(inlierIndicesB.size() + 1);
				inlierIndicesB(inlierIndicesB.size() - 1) = multiQueryKNNSearchImpl.indices(0, j);
			}
		}

		X.resize(inlierIndicesA.size(), locA.cols());
		for (j = 0; j < inlierIndicesA.size(); ++j)
		{
			X.row(j) = locA.row(inlierIndicesA(j));
		}
		fixed_Location.resize(0, 0);
		for (j = 0; j < fixed->points.size(); ++j)
		{
			if (!isnan(fixed->points[j].x))
			{
				fixed_Location.conservativeResize(fixed_Location.rows() + 1, 3);
				fixed_Location(fixed_Location.rows() - 1, 0) = fixed->points[j].x;
				fixed_Location(fixed_Location.rows() - 1, 1) = fixed->points[j].y;
				fixed_Location(fixed_Location.rows() - 1, 2) = fixed->points[j].z;
			}
		}
		Z.resize(inlierIndicesB.size(), fixed_Location.cols());
		for (j = 0; j < inlierIndicesB.size(); ++j)
		{
			Z.row(j) = fixed_Location.row(inlierIndicesB(j));
		}

		if (i == 0)
		{
			for (j = 0; j < keepInlierA.size(); ++j)
			{
				if (keepInlierA(j))
				{
					inlierDist.conservativeResize(inlierDist.size() + 1);
					inlierDist(inlierDist.size() - 1) = dists(0, j);
				}
			}
			Err(i) = sqrt(inlierDist.sum() / inlierDist.size());
		}

		computeRigidTransform.computeRigidTransform(X, Z);
		Rn = computeRigidTransform.R;
		Tn = computeRigidTransform.t;

		//cout << i << endl << Rn << endl << endl;
		//cout << i << endl << Tn << endl << endl;

		Rs[i + 1] = Rn * Rs[i];
		Ts[i + 1] = Rn * Ts[i] + Tn;

		//cout << i << endl << Rs[i + 1] << endl << endl;
		//cout << endl << Ts[i + 1] << endl << endl;

		locA = rigidTransform(moving_Location, Rs[i + 1], Ts[i + 1]);
		Dummy01.resize(inlierIndicesA.size(), locA.cols());
		squaredError.resize(inlierIndicesB.size());
		for (j = 0; j < inlierIndicesA.size(); ++j)
		{
			Dummy02 = locA.row(inlierIndicesA(j)) - fixed_Location.row(inlierIndicesB(j));
			squaredError(j) = 0;
			for (k = 0; k < Dummy02.size(); ++k)
			{
				squaredError(j) += Dummy02(k) * Dummy02(k);
			}
		}
		//cout << i << "    " << squaredError.size() << endl << endl;
		Err(i + 1) = sqrt(squaredError.sum() / squaredError.size());

		//cout << i << "    " << Err(i + 1) << endl << endl;
		if (abs(Err(i + 1) - Err(i)) < Tolerance)
		{
			// Stop ICP if it already converges
			stopIteration = i;
			break;
		}
	}
	
	Dummy03 = Err;
	k = 0;
	for (i = 0; i < Dummy03.size(); ++i)
	{
		if (Dummy03(i))
		{
			Err(k) = Dummy03(i);
			k++;
		}
	}
	Err.conservativeResize(k);
	minErr = Err(0); //假设Err的大小大于0.
	for (i = 0; i < k; ++i)
	{
		if (Err(i) < minErr)
		{
			minErr = Err(i);
			ind = i;
		}
	}
	this->stopIteration = ind;
	R = Rs[ind].transpose();
	svd.compute(R, Eigen::ComputeThinU | Eigen::ComputeFullV);
	U = svd.matrixU();
	V = svd.matrixV();
	R = U * V.transpose();
	this->tform.resize(4, 4);
	this->tform.block(0, 0, 3, 3) = R;
	this->tform.col(3).head(3) = Ts[ind];
	for (j = 0; j < 4; ++j)
	{
		for (i = 0; i < 4; ++i)
		{
			transformation_matrix(i, j) = this->tform(i, j);
		}
	}
	pcl::transformPointCloud(*moving, *cloud_icp, transformation_matrix);
	movingReg = cloud_icp; 
	return 1;
}

Eigen::MatrixXd Cpcregistericp_v2::rigidTransform(Eigen::MatrixXd A, Eigen::MatrixXd R, Eigen::VectorXd T)
{
	Eigen::MatrixXd B(A.rows(), A.cols());
	Eigen::VectorXd Dummy01(A.rows());

	B = A * R.transpose();
	Dummy01.setConstant(T(-1 + 1));
	B.col(-1 + 1) = B.col(-1 + 1) + Dummy01;
	B.col(-1 + 2) = B.col(-1 + 2) + Dummy01;
	B.col(-1 + 3) = B.col(-1 + 3) + Dummy01;
	return B;
}

Eigen::VectorXi Cpcregistericp_v2::sort(Eigen::MatrixXd& A)
{
	int i;
	Eigen::VectorXi I;

	I.resize(A.cols());
	for (i = 0; i < I.size(); ++i)
	{
		I(i) = i;
	}

	QKSort(A, I, 0, A.rows() - 1);
	return I;
}

void Cpcregistericp_v2::QKSort(Eigen::MatrixXd& r, Eigen::VectorXi& I, int low, int high)
{
	int pos;

	if (low < high)
	{
		pos = QKPass(r, I, low, high);
		QKSort(r, I, low, pos - 1);
		QKSort(r, I, pos + 1, high);
	}
}

int Cpcregistericp_v2::QKPass(Eigen::MatrixXd& r, Eigen::VectorXi& I, int low, int high)
{
	double x;
	int t;

	x = r(0, low);
	t = I(low);
	while (low < high)
	{
		while (low < high && r(0, high) >= x)
			high--;
		if (low < high) { r(0, low) = r(0, high); I(low) = I(high); low++; }
		while (low < high && r(0, low) < x)
			low++;
		if (low < high) { r(0, high) = r(0, low); I(high) = I(low); high--; }
	}
	r(0, low) = x;
	I(low) = t;
	return low;
}

void CPCA_ICP::pcshowpair(PointCloudT::Ptr ptCloudfixed, PointCloudT::Ptr ptCloudA, PointCloudT::Ptr ptCloudB, PointCloudT::Ptr ptCloudC, PointCloudT::Ptr ptCloudD)
{
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	int v1(0);
	int v2(1);
	int v3(2);
	int v4(3);
	viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
	viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(ptCloudfixed, 180, 20, 20);
	viewer.addPointCloud(ptCloudfixed, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(ptCloudfixed, cloud_in_color_h, "cloud_in_v2", v2);
	viewer.addPointCloud(ptCloudfixed, cloud_in_color_h, "cloud_in_v3", v3);
	viewer.addPointCloud(ptCloudfixed, cloud_in_color_h, "cloud_in_v4", v4);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(ptCloudA, 20, 180, 20);
	viewer.addPointCloud(ptCloudA, cloud_tr_color_h, "cloud_tr_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h2(ptCloudB, 20, 180, 20);
	viewer.addPointCloud(ptCloudB, cloud_tr_color_h2, "cloud_tr_v2", v2);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h3(ptCloudC, 20, 180, 20);
	viewer.addPointCloud(ptCloudC, cloud_tr_color_h3, "cloud_tr_v3", v3);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h4(ptCloudD, 20, 180, 20);
	viewer.addPointCloud(ptCloudD, cloud_tr_color_h4, "cloud_tr_v4", v4);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	//viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	//viewer.setSize(1280, 1024);  // Visualiser window size

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}
