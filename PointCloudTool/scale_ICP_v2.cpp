#pragma warning(disable:4819)
#pragma warning(disable:4996)
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <cstdlib>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <flann/flann.hpp>
#include <flann/util/matrix.h>

#include "scale_ICP_v2.h"

class Cupdate_rigid_Transform_parameter {
protected:
	Eigen::VectorXi sort(Eigen::MatrixXd& A);
	Eigen::MatrixXd rigidsTransform(Eigen::MatrixXd A, double s, Eigen::MatrixXd R, Eigen::VectorXd T);
	double computeS(Eigen::VectorXd I, Eigen::MatrixXd Rn, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi);
	int QKPass(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high);
	void QKSort(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high);

public:
	Eigen::MatrixXd Rn;
	Eigen::MatrixXd curA;
	double Err;
	Eigen::VectorXd Tn;
	double sn;
	int update_rigid_Transform_parameter(Eigen::MatrixXd pointmoving_Location, Eigen::MatrixXd pointfixed_Location, double s0, Eigen::VectorXd I, Eigen::MatrixXd R0, Eigen::VectorXd T0);
};

class CmultiQueryKNNSearchImpl2 {
public:
	Eigen::MatrixXi indices; //index值比matlab中的数值少1
	Eigen::MatrixXd dists;
	Eigen::VectorXi valid;
	int multiQueryKNNSearchImpl(Eigen::MatrixXd this_Location, Eigen::MatrixXd points, int K);
};

int CcomputeRigidTransform::computeRigidTransform(Eigen::MatrixXd p, Eigen::MatrixXd q)
{
	Eigen::VectorXd centroid1(p.cols()), centroid2(q.cols()), Dummy01;
	Eigen::MatrixXd normPoints1(p.rows(), p.cols()), normPoints2(q.rows(), q.cols()), C, U, V, Dummy02;
	int i;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;

	for (i = 0; i < p.cols(); ++i)
	{
		centroid1(i) = p.col(i).sum() / p.rows();
	}
	for (i = 0; i < q.cols(); ++i)
	{
		centroid2(i) = q.col(i).sum() / q.rows();
	}
	Dummy01.resize(p.rows());
	for (i = 0; i < p.cols(); ++i)
	{
		Dummy01.setConstant(centroid1(i));
		normPoints1.col(i) = p.col(i) - Dummy01;
	}
	Dummy01.resize(q.rows());
	for (i = 0; i < q.cols(); ++i)
	{
		Dummy01.setConstant(centroid2(i));
		normPoints2.col(i) = q.col(i) - Dummy01;
	}
	C = normPoints1.transpose() * normPoints2;
	svd.compute(C, Eigen::ComputeThinU | Eigen::ComputeFullV);
	U = svd.matrixU();
	V = svd.matrixV();
	Dummy02.resize(V.cols(), U.cols());
	Dummy02.setZero();
	for (i = 0; i < p.cols()-1; ++i)
	{ 
		Dummy02(i, i) = 1;
	}
	Dummy02(Dummy02.rows() - 1, Dummy02.cols() - 1) = sign((U*V.transpose()).determinant());

	R = V*Dummy02*U.transpose();
	t = (centroid2.matrix() - R*centroid1.matrix()).col(0);
	return(1);
}

int CcomputeRigidTransform::sign(double x)
{
	if (x>0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

Eigen::VectorXi Cupdate_rigid_Transform_parameter::sort(Eigen::MatrixXd& A)
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

int Cupdate_rigid_Transform_parameter::update_rigid_Transform_parameter(Eigen::MatrixXd pointmoving_Location, Eigen::MatrixXd pointfixed_Location, double s0, Eigen::VectorXd I, Eigen::MatrixXd R0, Eigen::VectorXd T0)
{
	Eigen::MatrixXd locA, X, Z, Xi, Zi, Dummy01;
	CmultiQueryKNNSearchImpl2 multiQueryKNNSearchImpl;
	Eigen::VectorXi keepInlierA, idx, inlierIndicesA, inlierIndicesB;
	std::size_t upperBound, i, j;
	CcomputeRigidTransform computeRigidTransform;
	Eigen::VectorXd xc, zc, squaredError;

	locA = rigidsTransform(pointmoving_Location, s0, R0, T0);
	multiQueryKNNSearchImpl.multiQueryKNNSearchImpl(pointfixed_Location, locA, 1);
	keepInlierA.resize(pointmoving_Location.rows());
	keepInlierA.setZero();
	idx = sort(multiQueryKNNSearchImpl.dists);
	upperBound = 1 < pointmoving_Location.rows() ? pointmoving_Location.rows() : 1;
	for (i = 0; i < upperBound; ++i)
	{
		keepInlierA(idx(i)) = 1;
	}
	for (i = 0; i < keepInlierA.size(); ++i)
	{
		if (keepInlierA(i))
		{
			inlierIndicesA.conservativeResize(inlierIndicesA.size() + 1);
			inlierIndicesA(inlierIndicesA.size() - 1) = i;
		}
	}
	for (i = 0; i < multiQueryKNNSearchImpl.indices.cols(); ++i)
	{
		if (keepInlierA(i))
		{
			inlierIndicesB.conservativeResize(inlierIndicesB.size() + 1);
			inlierIndicesB(inlierIndicesB.size() - 1) = multiQueryKNNSearchImpl.indices(0, i);
		}
	}
	X.resize(inlierIndicesA.size(), pointmoving_Location.cols());
	for (i = 0; i < inlierIndicesA.size(); ++i)
	{
		X.row(i) = pointmoving_Location.row(inlierIndicesA(i));
	}
	Z.resize(inlierIndicesB.size(), pointfixed_Location.cols());
	for (i = 0; i < inlierIndicesB.size(); ++i)
	{
		Z.row(i) = pointfixed_Location.row(inlierIndicesB(i));
	}
	computeRigidTransform.computeRigidTransform(X, Z);
	xc.resize(X.cols());
	for (i = 0; i < X.cols(); ++i)
	{
		xc(i) = X.col(i).sum() / X.rows();
	}
	zc.resize(Z.cols());
	for (i = 0; i < Z.cols(); ++i)
	{
		zc(i) = Z.col(i).sum() / Z.rows();
	}
	Dummy01.resize(X.rows(), X.cols());
	for (i = 0; i < X.cols(); ++i)
	{
		Dummy01.col(i).setConstant(xc(i));
	}
	Xi = X - Dummy01;
	Dummy01.resize(Z.rows(), Z.cols());
	for (i = 0; i < Z.cols(); ++i)
	{
		Dummy01.col(i).setConstant(zc(i));
	}
	Zi = Z - Dummy01;
	sn = computeS(I, computeRigidTransform.R, Xi, Zi);
	Tn = (zc.matrix().transpose() - sn*xc.matrix().transpose()*computeRigidTransform.R.transpose()).transpose().col(0);
	curA = rigidsTransform(pointmoving_Location, sn, computeRigidTransform.R, Tn);
	X.resize(inlierIndicesA.size(), X.cols());
	for (i = 0; i < inlierIndicesA.size(); ++i)
	{
		X.row(i) = curA.row(inlierIndicesA(i));
	}
	Dummy01 = X - Z;
	squaredError.resize(Dummy01.rows());
	squaredError.setZero();
	for (i = 0; i < Dummy01.rows(); ++i)
	{
		for (j = 0; j < Dummy01.cols(); ++j)
		{
			squaredError(i) = squaredError(i) + Dummy01(i, j)*Dummy01(i, j);
		}
	}
	Err = sqrt(squaredError.sum() / squaredError.size());
	Rn = computeRigidTransform.R;
	return(1);
}

int CmultiQueryKNNSearchImpl::multiQueryKNNSearchImpl(PointCloudT::Ptr cloud_this, Eigen::MatrixXd points, int K)
{
	std::size_t i, j;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointXYZ searchPoint;
	Eigen::VectorXi Idx;
	unsigned int Counter = 0;
	
	Idx.resize(cloud_this->points.size());
	for (i = 0; i < cloud_this->points.size(); ++i)
	{
		if (!isnan(cloud_this->points[i].x))
		{
			Idx(i) = Counter++;
		}
	}

	indices.resize(K, points.rows());
	dists.resize(K, points.rows());
	valid.resize(points.rows());

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	kdtree.setInputCloud(cloud_this);

	for (i = 0; i < points.rows(); ++i)
	{
		searchPoint.x = points(i, 0);
		searchPoint.y = points(i, 1);
		searchPoint.z = points(i, 2);

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			valid(i) = pointIdxNKNSearch.size();
			for (j = 0; j < pointIdxNKNSearch.size(); ++j)
			{
				indices(j, i) = Idx(pointIdxNKNSearch[j]);
				dists(j, i) = pointNKNSquaredDistance[j];
				pointIdxNKNSearch[j];
				/*std::cout << "    " << (*cloud_this)[pointIdxNKNSearch[j]].x
					<< " " << (*cloud_this)[pointIdxNKNSearch[j]].y
					<< " " << (*cloud_this)[pointIdxNKNSearch[j]].z
					<< " (squared distance: " << pointNKNSquaredDistance[j] << ")" << std::endl;*/
			}
		}
		else
		{
			valid(i) = 0;
		}
	}
	return 1;
}

int CmultiQueryKNNSearchImpl2::multiQueryKNNSearchImpl(Eigen::MatrixXd this_Location, Eigen::MatrixXd points, int K)
{
	std::size_t i, j;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointXYZ searchPoint;
	unsigned int Counter = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = this_Location.rows();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for ( i = 0; i < cloud->size(); ++i)
	{
		(*cloud)[i].x = this_Location(i, 0);
		(*cloud)[i].y = this_Location(i, 1);
		(*cloud)[i].z = this_Location(i, 2);
	}

	K = K < this_Location.rows() ? K : this_Location.rows();

	indices.resize(K, points.rows());
	dists.resize(K, points.rows());
	valid.resize(points.rows());

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	kdtree.setInputCloud(cloud);

	for (i = 0; i < points.rows(); ++i)
	{
		searchPoint.x = points(i, 0);
		searchPoint.y = points(i, 1);
		searchPoint.z = points(i, 2);

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			valid(i) = pointIdxNKNSearch.size();
			for (j = 0; j < pointIdxNKNSearch.size(); ++j)
			{
				indices(j, i) = pointIdxNKNSearch[j];
				dists(j, i) = pointNKNSquaredDistance[j];
				pointIdxNKNSearch[j];
				/*std::cout << "    " << (*cloud_this)[pointIdxNKNSearch[j]].x
				<< " " << (*cloud_this)[pointIdxNKNSearch[j]].y
				<< " " << (*cloud_this)[pointIdxNKNSearch[j]].z
				<< " (squared distance: " << pointNKNSquaredDistance[j] << ")" << std::endl;*/
			}
		}
		else
		{
			valid(i) = 0;
		}
	}
	return 1;
}

int Cscale_ICP_v2::scale_ICP_v2(PointCloudT::Ptr pointmoving, PointCloudT::Ptr pointfixed, int flag)
{
	double precision = 0.0001;
	Eigen::MatrixXd X, Y;
	size_t i;
	size_t pointx;
	Eigen::VectorXd xc, yc;
	Eigen::MatrixXd x1, Mx, y1, My;
	Eigen::MatrixXd Vx, Dx, Vy, Dy;
	Eigen::EigenSolver<Eigen::MatrixXd> es;
	Eigen::VectorXd sq, I;
	double s0;
	CmultiQueryKNNSearchImpl multiQueryKNNSearchImpl;
	Eigen::MatrixXi indices;
	Eigen::MatrixXd dists;
	Eigen::MatrixXi keepInlierA;
	Eigen::VectorXi idx;
	Eigen::VectorXi inlierIndicesA, inlierIndicesB;
	Eigen::MatrixXd Dummy01, Dummy02, C;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd U, V, R0;
	Eigen::VectorXd T0;
	Eigen::MatrixXd direction(8, 3);
	Eigen::MatrixXd Up, Uq, P_pca, Q_pca;
	Eigen::VectorXd Dist;
	int k;
	Eigen::MatrixXd Pk;
	int r;
	Eigen::VectorXd p1, p2, q1, q2, q3, p3;
	double f;
	Eigen::VectorXd xc2;
	Cupdate_rigid_Transform_parameter update_rigid_Transform_parameter;
	Eigen::MatrixXd Rn;
	Eigen::MatrixXd curA;
	double Err_past, Errn;
	Eigen::VectorXd Tn;
	double sn, q;
	unsigned iter;
	

	for (i = 0; i < pointmoving->points.size(); ++i)
	{
		if (!isnan(pointmoving->points[i].x))
		{
			X.conservativeResize(X.rows() + 1, 3);
			X(X.rows() - 1, 0) = pointmoving->points[i].x;
			X(X.rows() - 1, 1) = pointmoving->points[i].y;
			X(X.rows() - 1, 2) = pointmoving->points[i].z;
		}
	}

	for (i = 0; i < pointfixed->points.size(); ++i)
	{
		if (!isnan(pointfixed->points[i].x))
		{
			Y.conservativeResize(Y.rows() + 1, 3);
			Y(Y.rows() - 1, 0) = pointfixed->points[i].x;
			Y(Y.rows() - 1, 1) = pointfixed->points[i].y;
			Y(Y.rows() - 1, 2) = pointfixed->points[i].z;
		}
	}

	pointx = pointmoving->points.size();
	xc.resize(3);
	xc(0) = X.col(0).sum() / X.col(0).size();
	xc(1) = X.col(1).sum() / X.col(1).size();
	xc(2) = X.col(2).sum() / X.col(2).size();
	yc.resize(3);
	yc(0) = Y.col(0).sum() / Y.col(0).size();
	yc(1) = Y.col(1).sum() / Y.col(1).size();
	yc(2) = Y.col(2).sum() / Y.col(2).size();

	x1.resize(X.rows(), X.cols());
	for (i = 0; i < X.rows(); ++i)
	{
		x1(i, 0) = X(i, 0) - xc(0);
		x1(i, 1) = X(i, 1) - xc(1);
		x1(i, 2) = X(i, 2) - xc(2);
	}

	Mx = x1.transpose() * x1;

	y1.resize(Y.rows(), Y.cols());
	for (i = 0; i < Y.rows(); ++i)
	{
		y1(i, 0) = Y(i, 0) - yc(0);
		y1(i, 1) = Y(i, 1) - yc(1);
		y1(i, 2) = Y(i, 2) - yc(2);
	}

	//[Vx,Dx] = eig(Mx,'nobalance');
	//[Vy, Dy] = eig(My, 'nobalance');
	//翻译得可能不对
	My = y1.transpose() * y1;

	es.compute(Mx, true);
	Vx = es.pseudoEigenvectors();
	Dx = es.pseudoEigenvalueMatrix();
	BubbleSort(Dx, Vx, Dx.rows());
	//cout << Mx << endl << endl << Vx << endl << endl << Dx << endl << endl;

	es.compute(My, true);
	Vy = es.pseudoEigenvectors();
	Dy = es.pseudoEigenvalueMatrix();
	BubbleSort(Dy, Vy, Dy.rows());
	//cout << My << endl << endl << Vy << endl << endl << Dy << endl << endl;

	//sq = sum(sqrt(Dy/Dx));
	//疑问：Dx, Dy 是否一定是对角阵，并且对角线元素按顺序递增？
	//如果是对角阵，则下述简化算法成立
	//   7.7864         0         0
    //   0         39.0135         0
    //   0         0         80.4009
	sq.resize(Dy.cols());
	for (i = 0; i < Dy.cols(); ++i)
	{
		sq(i) = sqrt(Dy(i, i) / Dx(i, i));
	}
	s0 = sq.sum() / 3;

	//cout << sq << endl << endl;
	I.resize(2);
	I(1) = sq.maxCoeff();
	I(0) = sq.minCoeff();

	//cout << I << endl << endl;

	if (nargin > 2)
	{
		if (flag == 1)
		{
			multiQueryKNNSearchImpl.multiQueryKNNSearchImpl(pointfixed, X, 1);
			dists = multiQueryKNNSearchImpl.dists;
			indices = multiQueryKNNSearchImpl.indices;
			keepInlierA.resize(pointx, 1);
			keepInlierA.setZero();
			idx = sort(dists);
			for (i = 0; i < idx.size(); ++i)
			{
				keepInlierA(idx(i), 0) = 1;
			}
			for (i = 0; i < keepInlierA.rows(); ++i)
			{
				if (keepInlierA(i,0) != 0)
				{
					inlierIndicesA.conservativeResize(inlierIndicesA.size() + 1);
					inlierIndicesA(inlierIndicesA.size() - 1) = i;
				}
			}

			
			for (i = 0; i < keepInlierA.rows(); ++i)
			{
				if (keepInlierA(i, 0))
				{
					inlierIndicesB.conservativeResize(inlierIndicesB.size() + 1);
					inlierIndicesB(inlierIndicesB.size() - 1) = indices(0, i);
				}
			}
			//cout << inlierIndicesB << endl << endl;


			Dummy01.resize(inlierIndicesA.size(), x1.cols());
			for (i = 0; i < Dummy01.rows(); ++i)
			{
				Dummy01.row(i) = x1.row(inlierIndicesA(i));
			}
			Dummy02.resize(inlierIndicesB.size(), y1.cols());
			for (i = 0; i < Dummy02.rows(); ++i)
			{
				Dummy02.row(i) = y1.row(inlierIndicesB(i));
			}
			C = Dummy01.transpose() * Dummy02;
			//cout << C << endl << endl;

			svd.compute(C, Eigen::ComputeThinU | Eigen::ComputeFullV);
			U = svd.matrixU();
			V = svd.matrixV();
			//cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
			//cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

			Dummy01.resize(V.cols(), U.cols());
			Dummy01.setZero();
			Dummy01(0, 0) = Dummy01(1, 1) = 1;
			Dummy01(2, 2) = sign((U*V.transpose()).determinant());
			R0 = V * Dummy01 * U.transpose();
			T0 = yc - (xc.matrix().transpose() * R0).transpose().col(0);

		}
		else
		{
			direction << 1, 1, 1,
				1, 1, -1,
				1, -1, 1,
				1, -1, -1,
				-1, 1, 1,
				-1, 1, -1,
				-1, -1, 1,
				-1, -1, -1;
			svd.compute(Mx, Eigen::ComputeThinU | Eigen::ComputeFullV);
			Up = svd.matrixU();
			//Up.col(1) = -Up.col(1);
			//cout << endl << Up << endl << endl;
			svd.compute(My, Eigen::ComputeThinU | Eigen::ComputeFullV);
			Uq = svd.matrixU();
			//Uq.col(0) = -Uq.col(0);
			//Uq.col(1) = -Uq.col(1);
			/**************************************************/
			//cout << Uq << endl << endl;
			P_pca = x1*Up;
			Q_pca = y1*Uq;
			//cout << y1.block(0, 0, 12, 3) << endl << endl;
			//cout << Q_pca.block(0, 0, 12,3) << endl << endl;
			Dist.resize(8);
			Dist.setZero();
			Dummy01.resize(3, 3);
			Dummy01.setZero();
			for (k = 0; k < 8; ++k)
			{
				Dummy01(0, 0) = direction(k, 0);
				Dummy01(1, 1) = direction(k, 1);
				Dummy01(2, 2) = direction(k, 2);
				Pk = P_pca*Dummy01;
				//cout << Pk.block(0, 0, 10,3) << endl << endl;
				//cout << Q_pca.block(0, 0, 10, 3) << endl << endl;
				Dist(k) = calculate_min_dist(Pk, Q_pca);
			}

			//cout << Dist << endl << endl;
			r = 0;
			for (i = 0; i < 8; ++i)
			{
				if (Dist[r] > Dist[i])
					r = i;
			}
			Dummy01.resize(3, 3);
			Dummy01.setZero();
			Dummy01(0, 0) = direction(r, 0);
			Dummy01(1, 1) = direction(r, 1);
			Dummy01(2, 2) = direction(r, 2);
			R0 = Uq*Dummy01*Up.transpose();
			T0 = yc - (xc.matrix().transpose() * R0).transpose().col(0);

			//cout << T0 << endl << endl;
		}
	}
	else
	{
		//cout << endl << Vx << endl << endl;
		//cout << Vy << endl << endl;
		p1 = Vx.col(-1+1);
		p2 = Vx.col(-1+2);
		q1 = Vy.col(-1+1);
		q2 = Vy.col(-1+2);
		q3 = Vy.col(-1+3);
		f = 0.8;   //f是阀值
		if (p1.dot(q1) < f)
		{
			p1 = -p1;
		}
		if (p2.dot(q2) < f)
		{
			p2 = -p2;
		}
		p3 = Crossproduct(p1,p2);  // 正交化，保证p1, p2, p3正交
		Dummy01.resize(p1.rows(), 3);
		Dummy01.col(0) = p1;
		Dummy01.col(1) = p2;
		Dummy01.col(2) = p3;
		Dummy02.resize(q1.rows(), 3);
		Dummy02.col(0) = q1;
		Dummy02.col(1) = q2;
		Dummy02.col(2) = q3;
		R0 = Dummy02 * Dummy01.inverse();
		Dummy01 = s0*X*R0;
		xc2.resize(Dummy01.cols());
		xc2(0) = Dummy01.col(0).sum() / Dummy01.rows();
		xc2(1) = Dummy01.col(1).sum() / Dummy01.rows();
		xc2(2) = Dummy01.col(2).sum() / Dummy01.rows();
		T0 = (yc - xc2);

		//cout << T0 << endl << endl;
	}
	
	update_rigid_Transform_parameter.update_rigid_Transform_parameter(X, Y, s0, I, R0, T0);
	sn = update_rigid_Transform_parameter.sn;
	Rn = update_rigid_Transform_parameter.Rn;
	Tn = update_rigid_Transform_parameter.Tn;
	Err_past = update_rigid_Transform_parameter.Err;
	curA = update_rigid_Transform_parameter.curA;

	q = 1;
	iter = 1;
	while (q > precision)
	{
		update_rigid_Transform_parameter.update_rigid_Transform_parameter(X, Y, sn, I, Rn, Tn);
		sn = update_rigid_Transform_parameter.sn;
		Rn = update_rigid_Transform_parameter.Rn;
		Tn = update_rigid_Transform_parameter.Tn;
		Errn = update_rigid_Transform_parameter.Err;
		curA = update_rigid_Transform_parameter.curA;
		q = abs(1 - Errn / Err_past);
		Err_past = Errn;
		iter = iter + 1;
		//cout << iter << endl;
	}

	//cout << curA.block(0, 0, 20, curA.cols()) << endl << endl;
	pointout = pointCloud(curA);
	this->s = sn;
	this->R = Rn;
	this->T = Tn;
	this->iter = iter;
	this->Errn = Errn;
	return 0;
}

int Cscale_ICP_v2::sign(double x)
{
	if (x>0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

void Cscale_ICP_v2::BubbleSort(Eigen::MatrixXd& D, Eigen::MatrixXd& V, int n)
{
	int i, j;
	int change = 1;
	Eigen::VectorXd veTmp;
	double x;

	for (i = 0; i < n - 1 && change; ++i)
	{
		change = 0;
		for (j = 0; j < n - 1 - i; ++j)
		{
			if (D(j, j) > D(j + 1, j + 1))
			{
				x = D(j, j);
				D(j, j) = D(j + 1, j + 1);
				D(j + 1, j + 1) = x;
				veTmp = V.col(j);
				V.col(j) = V.col(j + 1);
				V.col(j + 1) = veTmp;
				change = 1;
			}
		}
	}
}

Eigen::VectorXi Cscale_ICP_v2::sort(Eigen::MatrixXd& A)
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

void Cscale_ICP_v2::QKSort(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high)
{
	int pos;

	if (low < high)
	{
		pos = QKPass(r, I, low, high);
		QKSort(r, I, low, pos - 1);
		QKSort(r, I, pos + 1, high);
	}
}

int Cscale_ICP_v2::QKPass(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high)
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

void Cupdate_rigid_Transform_parameter::QKSort(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high)
{
	int pos;

	if (low < high)
	{
		pos = QKPass(r, I, low, high);
		QKSort(r, I, low, pos - 1);
		QKSort(r, I, pos + 1, high);
	}
}

int Cupdate_rigid_Transform_parameter:: QKPass(Eigen::MatrixXd& r, Eigen::VectorXi &I, int low, int high)
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

int Cknnsearch::knnsearch(Eigen::MatrixXd X, Eigen::MatrixXd Y)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::size_t i;

	cloud->width = X.rows();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for ( i = 0; i < cloud->size(); ++i)
	{
		(*cloud)[i].x = X(i, 0);
		(*cloud)[i].y = X(i, 1);
		(*cloud)[i].z = X(i, 2);
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	pcl::PointXYZ searchPoint;
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	idx.resize(Y.rows());
	dist.resize(Y.rows());
	for (i = 0; i < Y.rows(); ++i)
	{
		searchPoint.x = Y(i, 0);
		searchPoint.y = Y(i, 1);
		searchPoint.z = Y(i, 2);

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			idx(i) = pointIdxNKNSearch[0];
			dist(i) = sqrt(pointNKNSquaredDistance[0]);
			/*std::cout << "    " << pointIdxNKNSearch[0]
				<< "    " << (*cloud)[pointIdxNKNSearch[0]].x
				<< " " << (*cloud)[pointIdxNKNSearch[0]].y
				<< " " << (*cloud)[pointIdxNKNSearch[0]].z
				<< " (squared distance: " << pointNKNSquaredDistance[0] << ")" << std::endl;*/
		}
	}

	return 1;
}

double Cscale_ICP_v2::calculate_min_dist(Eigen::MatrixXd P, Eigen::MatrixXd Q)
{
	Cknnsearch knnsearch;
	int m;
	double dist;

	knnsearch.knnsearch(Q, P);
	m = P.rows();
	dist = knnsearch.dist.sum() / m;
	return dist;
}

Eigen::VectorXd Cscale_ICP_v2::Crossproduct(Eigen::VectorXd a, Eigen::VectorXd u)
{
	Eigen::Vector3d v(1, 2, 3);
	Eigen::Vector3d w(0, 1, 2);
	Eigen::VectorXd b(3);

	v(0) = a(0);
	v(1) = a(1);
	v(2) = a(2);
	w(0) = u(0);
	w(1) = u(1);
	w(2) = u(2);
	b = v.cross(w);

	return b;
}

double Cupdate_rigid_Transform_parameter::computeS(Eigen::VectorXd I, Eigen::MatrixXd Rn, Eigen::MatrixXd Xi, Eigen::MatrixXd Zi)
{
	Eigen::VectorXd Dummy01, Dummy02;
	int i;
	double sn;
	Eigen::MatrixXd Dummy03;

	Dummy03 = Xi * Rn.transpose();
	Dummy01.resize(Xi.rows());
	for (i = 0; i < Dummy01.size(); ++i)
	{
		Dummy01(i) = Dummy03.row(i).dot(Zi.row(i));
	}
	Dummy02.resize(Xi.rows());
	for (i = 0; i < Dummy02.size(); ++i)
	{
		Dummy02(i) = Xi.row(i).dot(Xi.row(i));
	}
	sn = Dummy01.sum() / Dummy02.sum();
	if (sn <= I(0))
		sn = I(0);
	else if (sn >= I(1))
		sn = I(1);

	return sn;
}

Eigen::MatrixXd Cupdate_rigid_Transform_parameter::rigidsTransform(Eigen::MatrixXd A, double s, Eigen::MatrixXd R, Eigen::VectorXd T)
{
	Eigen::MatrixXd B = s * (A * R.transpose());
	Eigen::VectorXd Dummy01(B.rows());

	Dummy01.setConstant(T(0));
	B.col(-1 + 1) = B.col(-1 + 1) + Dummy01;
	Dummy01.setConstant(T(1));
	B.col(-1 + 2) = B.col(-1 + 2) + Dummy01;
	Dummy01.setConstant(T(2));
	B.col(-1 + 3) = B.col(-1 + 3) + Dummy01;

	return B;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Cscale_ICP_v2::pointCloud(Eigen::MatrixXd xyzPoints)
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
