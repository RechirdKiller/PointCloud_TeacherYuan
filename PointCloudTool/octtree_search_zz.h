#pragma once

#include <pcl/point_types.h>
#include <iostream>
#include <math.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
namespace octree_my
{
	// ����˲����ڵ���
	class octree_node_my
	{
	private:
		//�Ƿ�ΪҶ�ӽڵ�;
		BOOLEAN IsLeaf;
		//�Ƿ�Ϊ��Ҷ�ӽڵ�;
		BOOLEAN hasCloud;

		//Ҷ�ӽڵ�����
		//�Ÿ趥��;
		float x_min = 0, x_max = 0;
		float y_min = 0, y_max = 0;
		float z_min = 0, z_max = 0;
		//�����±�;
		std::vector<int> pointIndex;
		//�������
		int depth = 0;

	public:
		//���ڵ�;
		octree_node_my * father_Node = NULL;
		//��Ҷ�ӽڵ�����;
		octree_node_my * top_left_front_Node = NULL;
		octree_node_my * top_left_back_Node = NULL;
		octree_node_my * top_right_front_Node = NULL;
		octree_node_my * top_right_back_Node = NULL;
		octree_node_my * bottom_left_front_Node = NULL;
		octree_node_my * bottom_left_back_Node = NULL;
		octree_node_my * bottom_right_front_Node = NULL;
		octree_node_my * bottom_right_back_Node = NULL;
		//���캯��
		octree_node_my()
		{
			IsLeaf = FALSE;//Ĭ�ϲ���Ҷ�ӽڵ�;
			hasCloud = FALSE;//Ĭ�ϲ����е���;
			pointIndex.resize(0);
			pointIndex.reserve(0);

		};
		//��������
		~octree_node_my()
		{};
		void setX_min(float x_min)
		{

			this->x_min = x_min;
		}
		void setX_max(float x_max)
		{

			this->x_max = x_max;
		}
		void setY_min(float y_min)
		{

			this->y_min = y_min;
		}
		void setY_max(float y_max)
		{

			this->y_max = y_max;
		}
		void setZ_min(float z_min)
		{

			this->z_min = z_min;
		}
		void setZ_max(float z_max)
		{

			this->z_max = z_max;
		}
		float getX_min()
		{
			return x_min;
		}
		float getX_Max()
		{
			return x_max;
		}
		float getY_min()
		{
			return y_min;
		}
		float getY_Max()
		{
			return y_max;
		}
		float getZ_min()
		{
			return z_min;
		}
		float getZ_Max()
		{
			return z_max;
		}
		void setBorder(float X_Min, float X_Max, float Y_Min, float Y_Max, float Z_Min, float Z_Max)
		{
			x_min = X_Min;
			x_max = X_Max;
			y_min = Y_Min;
			y_max = Y_Max;
			z_min = Z_Min;
			z_max = Z_Max;
		}

		void setLeaf()
		{
			this->IsLeaf = TRUE;
		}
		BOOLEAN isLeaf()
		{
			return this->IsLeaf;
		}
		void addPointIndex(float Index)
		{
			this->pointIndex.push_back(Index);
		}
		void setDepth(int depth)
		{
			this->depth = depth;
		}
		int getDepth()
		{
			return this->depth;
		}
		BOOLEAN getHasCloud()
		{
			return this->hasCloud;
		}
		std::vector<int> getPoints()
		{
			return this->pointIndex;
		}
		void HasCloud()
		{
			this->hasCloud = TRUE;
		}
	};



	//����˲���
	class octree
	{
	private:
		//���ڵ㣺
		octree_node_my * root = NULL;
		//�������;
		int depth = 0;
		//�������������;
		int max_points_size = 0;
		//���ƣ�
		PointCloudT::Ptr cloud = nullptr;


		void setInputCloud(PointCloudT::Ptr cloud)
		{
			this->cloud = cloud;
		}
		void setDepth(int depth)
		{
			this->depth = depth;
		}
		//��ȡ��ı߽�;
		BOOLEAN getSize(PointCloudT::Ptr cloud, float *x_min, float *x_max, float  *y_min, float *y_max, float *z_min, float *z_max)
		{
			if (cloud->empty()) {
				return FALSE;
			}
			float min = FLT_MIN;
			float max = FLT_MAX;
			*x_min = max;
			*x_max = min;
			*z_max = min;
			*z_min = max;
			*y_max = min;
			*y_min = max;
			for (int i = 0; i < cloud->points.size(); i++) {
				if (cloud->points[i].x >(*x_max)) {
					*x_max = cloud->points[i].x;
				}

				if (cloud->points[i].x < (*x_min)) {
					*x_min = cloud->points[i].x;
				}
				if (cloud->points[i].y >(*y_max)) {
					*y_max = cloud->points[i].y;
				}
				if (cloud->points[i].y < (*y_min)) {
					*y_min = cloud->points[i].y;
				}
				if (cloud->points[i].z >(*z_max)) {
					*z_max = cloud->points[i].z;
				}
				if (cloud->points[i].z < (*z_min)) {
					*z_min = cloud->points[i].z;
				}
			}
			return TRUE;
		}
		//rootΪ�ӵ���ڵ㴴����depth
		void CreatTree_root_depth(octree_node_my* root, int max_depth, PointCloudT::Ptr cloud, int* current_tree_max_depth)
		{
			if (*current_tree_max_depth < root->getDepth())
			{
				*current_tree_max_depth = root->getDepth();
			}
			if (root->getDepth() == max_depth) {
				//�ﵽ�����ȣ���ΪҶ�ڵ�;
				root->setLeaf();
				return;
			}
			else if (!root->getHasCloud()) {
				//û�е㣬��ΪҶ�ڵ�;
				root->setLeaf();
				return;
			}
			else {
				//�����֧���������;
				root->top_left_front_Node = new octree_node_my();
				root->top_left_front_Node->father_Node = root;
				root->top_left_front_Node->setDepth(root->getDepth() + 1);

				root->top_left_back_Node = new octree_node_my();
				root->top_left_back_Node->father_Node = root;
				root->top_left_back_Node->setDepth(root->getDepth() + 1);

				root->top_right_front_Node = new octree_node_my();
				root->top_right_front_Node->father_Node = root;
				root->top_right_front_Node->setDepth(root->getDepth() + 1);

				root->top_right_back_Node = new octree_node_my();
				root->top_right_back_Node->father_Node = root;
				root->top_right_back_Node->setDepth(root->getDepth() + 1);

				root->bottom_left_front_Node = new octree_node_my();
				root->bottom_left_front_Node->father_Node = root;
				root->bottom_left_front_Node->setDepth(root->getDepth() + 1);

				root->bottom_left_back_Node = new octree_node_my();
				root->bottom_left_back_Node->father_Node = root;
				root->bottom_left_back_Node->setDepth(root->getDepth() + 1);

				root->bottom_right_front_Node = new octree_node_my();
				root->bottom_right_front_Node->father_Node = root;
				root->bottom_right_front_Node->setDepth(root->getDepth() + 1);

				root->bottom_right_back_Node = new octree_node_my();
				root->bottom_right_back_Node->father_Node = root;
				root->bottom_right_back_Node->setDepth(root->getDepth() + 1);

				//�ҳ����ֳ߶�;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//���ñ߿�
				float left_border;
				float right_border;
				float front_border;
				float back_border;
				float bottom_border;
				float top_border;
				//�� �� ǰ
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);

				//�������
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					int flag = 1;
					if (cloud->points[root->getPoints()[i]].z > z_center)
					{
						//�� flag = 1;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//�� �� flag = 1;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 1;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 2;
							}
						}
						else
						{
							flag += 2;
							//�� �� flag = 3;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 3;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 4;
							}

						}
					}
					else
					{
						flag += 4;
						//�� flag = 5;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//�� �� flag = 5;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 5;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 6;
							}
						}
						else
						{
							flag += 2;
							//�� �� flag = 7;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 7;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 8;
							}
						}
					}
					switch (flag) {
					case 1:
						//�� �� ǰ
						root->top_left_front_Node->HasCloud();
						root->top_left_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 2:
						//�� �� ��
						root->top_left_back_Node->HasCloud();
						root->top_left_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 3:
						//�� �� ǰ
						root->top_right_front_Node->HasCloud();
						root->top_right_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 4:
						//�� �� ��
						root->top_right_back_Node->HasCloud();
						root->top_right_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 5:
						//�� �� ǰ
						root->bottom_left_front_Node->HasCloud();
						root->bottom_left_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 6:
						//�� �� ��
						root->bottom_left_back_Node->HasCloud();
						root->bottom_left_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 7:
						//�� �� ǰ
						root->bottom_right_front_Node->HasCloud();
						root->bottom_right_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 8:
						//�� �� ��
						root->bottom_right_back_Node->HasCloud();
						root->bottom_right_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					}
				}
			}

			//���������ݹ����;
			CreatTree_root_depth(root->top_left_front_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->top_left_back_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->top_right_front_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->top_right_back_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->bottom_left_front_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->bottom_left_back_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->bottom_right_front_Node, max_depth, cloud, current_tree_max_depth);
			CreatTree_root_depth(root->bottom_right_back_Node, max_depth, cloud, current_tree_max_depth);
		}
		//��������֮�����;
		float Distance(pcl::PointXYZ point1, pcl::PointXYZ point2)
		{
			float sum = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
			return sqrt(sum);
		}
		//rootΪ�ӵ���ڵ㴴��������������������;
		void CreatTree_root_maxPointsSize(octree_node_my* root, int max_points_size, PointCloudT::Ptr cloud, int* current_tree_max_depth)
		{
			if (*current_tree_max_depth < root->getDepth())
			{
				//�������ĸ߶�;
				*current_tree_max_depth = root->getDepth();
			}
			if (!root->getHasCloud()) {
				//û�е㣬��ΪҶ�ڵ�;
				root->setLeaf();
				return;
			}
			else if (root->getPoints().size() <= max_points_size) {
				//�ýڵ��ڵ�ĸ���С��Ҫ���������������;
				root->setLeaf();
				return;
			}
			else {
				//�����֧���������;
				root->top_left_front_Node = new octree_node_my();
				root->top_left_front_Node->father_Node = root;
				root->top_left_front_Node->setDepth(root->getDepth() + 1);

				root->top_left_back_Node = new octree_node_my();
				root->top_left_back_Node->father_Node = root;
				root->top_left_back_Node->setDepth(root->getDepth() + 1);

				root->top_right_front_Node = new octree_node_my();
				root->top_right_front_Node->father_Node = root;
				root->top_right_front_Node->setDepth(root->getDepth() + 1);

				root->top_right_back_Node = new octree_node_my();
				root->top_right_back_Node->father_Node = root;
				root->top_right_back_Node->setDepth(root->getDepth() + 1);

				root->bottom_left_front_Node = new octree_node_my();
				root->bottom_left_front_Node->father_Node = root;
				root->bottom_left_front_Node->setDepth(root->getDepth() + 1);

				root->bottom_left_back_Node = new octree_node_my();
				root->bottom_left_back_Node->father_Node = root;
				root->bottom_left_back_Node->setDepth(root->getDepth() + 1);

				root->bottom_right_front_Node = new octree_node_my();
				root->bottom_right_front_Node->father_Node = root;
				root->bottom_right_front_Node->setDepth(root->getDepth() + 1);

				root->bottom_right_back_Node = new octree_node_my();
				root->bottom_right_back_Node->father_Node = root;
				root->bottom_right_back_Node->setDepth(root->getDepth() + 1);

				//�ҳ����ֳ߶�;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//���ñ߿�
				float left_border;
				float right_border;
				float front_border;
				float back_border;
				float bottom_border;
				float top_border;
				//�� �� ǰ
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_min();
				top_border = bottom_border + z_length;
				root->bottom_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);

				//�������
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					int flag = 1;
					if (cloud->points[root->getPoints()[i]].z > z_center)
					{
						//�� flag = 1;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//�� �� flag = 1;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 1;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 2;
							}
						}
						else
						{
							flag += 2;
							//�� �� flag = 3;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 3;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 4;
							}

						}
					}
					else
					{
						flag += 4;
						//�� flag = 5;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//�� �� flag = 5;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 5;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 6;
							}
						}
						else
						{
							flag += 2;
							//�� �� flag = 7;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//�� �� ǰ flag = 7;
							}
							else
							{
								flag += 1;
								//�� �� �� flag = 8;
							}
						}
					}
					switch (flag) {
					case 1:
						//�� �� ǰ
						root->top_left_front_Node->HasCloud();
						root->top_left_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 2:
						//�� �� ��
						root->top_left_back_Node->HasCloud();
						root->top_left_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 3:
						//�� �� ǰ
						root->top_right_front_Node->HasCloud();
						root->top_right_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 4:
						//�� �� ��
						root->top_right_back_Node->HasCloud();
						root->top_right_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 5:
						//�� �� ǰ
						root->bottom_left_front_Node->HasCloud();
						root->bottom_left_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 6:
						//�� �� ��
						root->bottom_left_back_Node->HasCloud();
						root->bottom_left_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 7:
						//�� �� ǰ
						root->bottom_right_front_Node->HasCloud();
						root->bottom_right_front_Node->addPointIndex(root->getPoints()[i]);
						break;
					case 8:
						//�� �� ��
						root->bottom_right_back_Node->HasCloud();
						root->bottom_right_back_Node->addPointIndex(root->getPoints()[i]);
						break;
					}
				}
			}

			//���������ݹ����;
			CreatTree_root_maxPointsSize(root->top_left_front_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->top_left_back_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->top_right_front_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->top_right_back_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->bottom_left_front_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->bottom_left_back_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->bottom_right_front_Node, max_points_size, cloud, current_tree_max_depth);
			CreatTree_root_maxPointsSize(root->bottom_right_back_Node, max_points_size, cloud, current_tree_max_depth);
		}
		//����һ�������ڵĵ���;
		//�����㡢�߳����������±ꡢ���������;
		void Search(pcl::PointXYZ searchPoint, float length, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud)
		{
			if (!root->getHasCloud())
			{
				//û�е���;
				return;
			}
			//ͬʱ����������û�н���,��÷���������;
			//���Ʊ߽�
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();
			//����߽�
			float point_left_border = searchPoint.x - length;
			float point_right_border = searchPoint.x + length;
			float point_front_border = searchPoint.y - length;
			float point_back_border = searchPoint.y + length;
			float point_bottom_border = searchPoint.z - length;
			float point_top_border = searchPoint.z + length;

			BOOLEAN flag = FALSE;//flag = false ��ʾû�н���;
			if ((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//����������;
				flag = TRUE;
			}
			else if ((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border))
			{
				//����������
				flag = TRUE;
			}
			else if ((point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//�����Ǻ�ǰ
				flag = TRUE;
			}
			if (flag)
			{
				//�н���
				if (root->isLeaf())
				{
					//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
					for (int i = 0; i < root->getPoints().size(); i++)
					{
						if ((cloud->points[root->getPoints()[i]].x >= point_left_border && cloud->points[root->getPoints()[i]].x <= point_right_border)
							&& (cloud->points[root->getPoints()[i]].y >= point_front_border && cloud->points[root->getPoints()[i]].y <= point_back_border)
							&& (cloud->points[root->getPoints()[i]].z >= point_bottom_border && cloud->points[root->getPoints()[i]].z <= point_top_border))
						{
							pointIndex.push_back(root->getPoints()[i]);
							pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
						}
					}
					return;
				}
				else
				{
					//����Ҷ�ڵ�;
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->top_left_front_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->top_left_back_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->top_right_front_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->top_right_back_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_left_front_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_left_back_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_right_front_Node, cloud);
					Search(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_right_back_Node, cloud);

				}
			}
		};
		//��������
		void Vearch_root(pcl::PointXYZ searchPoint, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud)
		{

			if (!root->getHasCloud())
			{
				//û�е���;
				return;
			}
			//���Ʊ߽�
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();

			//�жϵ��Ƿ��ڵ���������;
			if (searchPoint.x <= cloud_right_border && searchPoint.x >= cloud_left_border
				&& searchPoint.y <= cloud_back_border && searchPoint.y >= cloud_front_border
				&& searchPoint.z <= cloud_top_border && searchPoint.z >= cloud_bottom_border)
			{
				//��������;
			}
			else
			{
				return;
			}

			//������ڵ����������ҵ���������Ҷ�ڵ㣬ֱ�ӷ��ص��������µĵ�;

			if (root->isLeaf())
			{
				//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					pointIndex.push_back(root->getPoints()[i]);
					pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
				}
				return;
			}
			//���ڵ��������µ������ز���Ҷ�ڵ㣬�������������ص�������;
			else
			{
				//�ҳ����ֳ߶�;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//����Ҷ�ڵ�;

				//Ѱ�ҵ�Ŀռ����λ�ã���flag��������ڵİ˸�����;
				int flag = 1;
				if (searchPoint.z > z_center)
				{
					//�� flag = 1;
					if (searchPoint.x < x_center)
					{
						//�� �� flag = 1;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 1;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 2;
						}
					}
					else
					{
						flag += 2;
						//�� �� flag = 3;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 3;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 4;
						}

					}
				}
				else
				{
					flag += 4;
					//�� flag = 5;
					if (searchPoint.x < x_center)
					{
						//�� �� flag = 5;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 5;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 6;
						}
					}
					else
					{
						flag += 2;
						//�� �� flag = 7;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 7;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 8;
						}
					}
				}


				switch (flag) {
				case 1:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_left_front_Node, cloud);

					break;
				case 2:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_left_back_Node, cloud);

					break;
				case 3:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_right_front_Node, cloud);

					break;
				case 4:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_right_back_Node, cloud);

					break;
				case 5:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_front_Node, cloud);

					break;
				case 6:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_back_Node, cloud);

					break;
				case 7:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_front_Node, cloud);

					break;
				case 8:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_back_Node, cloud);

					break;
				}

			}
		};
		//���������ڵ�����;
		BOOLEAN Voxel_point(pcl::PointXYZ searchPoint, octree_node_my* root, PointCloudT::Ptr cloud, octree_node_my** leaf)
		{
			if (!root->getHasCloud())
			{
				//û�е���;
				return FALSE;
			}
			//ͬʱ����������û�н���,��÷���������;
			//���Ʊ߽�
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();


			if (searchPoint.x <= cloud_right_border && searchPoint.x >= cloud_left_border
				&& searchPoint.y <= cloud_back_border && searchPoint.y >= cloud_front_border
				&& searchPoint.z <= cloud_top_border && searchPoint.z >= cloud_bottom_border)
			{
				//��������;
			}
			else
			{
				return FALSE;
			}

			if (root->isLeaf())
			{
				//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
				*leaf = root;
				return TRUE;
			}
			else
			{
				//�ҳ����ֳ߶�;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//����Ҷ�ڵ�;
				int flag = 1;
				if (searchPoint.z > z_center)
				{
					//�� flag = 1;
					if (searchPoint.x < x_center)
					{
						//�� �� flag = 1;
						if (searchPoint.y < y_center)
						{
							//�� �� �� flag = 1;
						}
						else
						{
							flag += 1;
							//�� �� ǰ flag = 2;
						}
					}
					else
					{
						flag += 2;
						//�� �� flag = 3;
						if (searchPoint.y < y_center)
						{
							//�� �� �� flag = 3;
						}
						else
						{
							flag += 1;
							//�� �� ǰ flag = 4;
						}

					}
				}
				else
				{
					flag += 4;
					//�� flag = 5;
					if (searchPoint.x < x_center)
					{
						//�� �� flag = 5;
						if (searchPoint.y < y_center)
						{
							//�� �� �� flag = 5;
						}
						else
						{
							flag += 1;
							//�� �� ǰ flag = 6;
						}
					}
					else
					{
						flag += 2;
						//�� �� flag = 7;
						if (searchPoint.y < y_center)
						{
							//�� �� �� flag = 7;
						}
						else
						{
							flag += 1;
							//�� �� ǰ flag = 8;
						}
					}
				}
				switch (flag) {
				case 1:
					//�� �� ��
					return Voxel_point(searchPoint, root->top_left_front_Node, cloud, leaf);
					break;
				case 2:
					//�� �� ǰ
					return Voxel_point(searchPoint, root->top_left_back_Node, cloud, leaf);
					break;
				case 3:
					//�� �� ��
					return Voxel_point(searchPoint, root->top_right_front_Node, cloud, leaf);
					break;
				case 4:
					//�� �� ǰ
					return Voxel_point(searchPoint, root->top_right_back_Node, cloud, leaf);
					break;
				case 5:
					//�� �� ��
					return Voxel_point(searchPoint, root->bottom_left_front_Node, cloud, leaf);
					break;
				case 6:
					//�� �� ǰ
					return Voxel_point(searchPoint, root->bottom_left_back_Node, cloud, leaf);
					break;
				case 7:
					//�� �� ��
					return Voxel_point(searchPoint, root->bottom_right_front_Node, cloud, leaf);
					break;
				case 8:
					//�� �� ǰ
					return Voxel_point(searchPoint, root->bottom_right_back_Node, cloud, leaf);
					break;
				}

			}
		}
		//��������һ��������k����;���밴��С����;
		void k_search_voxel(pcl::PointXYZ searchPoint, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud, int k, octree_node_my* leaf) {
			if (k <= 0)
			{
				return;
			}
			if (!root->getHasCloud())
			{
				//û�е���;
				return;
			}
			//ͬʱ����������û�н���,��÷���������;
			//���Ʊ߽�
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();


			if (searchPoint.x <= cloud_right_border && searchPoint.x >= cloud_left_border
				&& searchPoint.y <= cloud_back_border && searchPoint.y >= cloud_front_border
				&& searchPoint.z <= cloud_top_border && searchPoint.z >= cloud_bottom_border)
			{
				//��������;
			}
			else
			{
				return;
			}

			if (root->isLeaf())
			{
				leaf = root;
				//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					if (pointIndex.size() <= k)
					{
						if (pointIndex.size() == 0)
						{
							//����ѵ��㼯���ǿյģ�����Ѱ����ţ�ֱ�Ӳ���;
							pointIndex.push_back(root->getPoints()[i]);
							pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
						}
						else
						{
							float d = Distance(cloud->points[root->getPoints()[i]], searchPoint);
							int index = root->getPoints()[i];
							for (int j = 0; j < pointIndex.size(); j++)
							{
								//�������;
								if (d <= pointSquaredDistance[j])
								{
									pointIndex.insert(pointIndex.begin() + j, index);
									pointSquaredDistance.insert(pointSquaredDistance.begin() + j, d);
									if (pointIndex.size() == k)
									{
										//�ѹ���k��;
										return;
									}
								}
							}
						}

					}
				}
			}
			else
			{
				//�ҳ����ֳ߶�;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//����Ҷ�ڵ�;
				int flag = 1;
				if (searchPoint.z > z_center)
				{
					//�� flag = 1;
					if (searchPoint.x < x_center)
					{
						//�� �� flag = 1;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 1;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 2;
						}
					}
					else
					{
						flag += 2;
						//�� �� flag = 3;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 3;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 4;
						}

					}
				}
				else
				{
					flag += 4;
					//�� flag = 5;
					if (searchPoint.x < x_center)
					{
						//�� �� flag = 5;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 5;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 6;
						}
					}
					else
					{
						flag += 2;
						//�� �� flag = 7;
						if (searchPoint.y < y_center)
						{
							//�� �� ǰ flag = 7;
						}
						else
						{
							flag += 1;
							//�� �� �� flag = 8;
						}
					}
				}
				switch (flag) {
				case 1:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_left_front_Node, cloud);

					break;
				case 2:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_left_back_Node, cloud);

					break;
				case 3:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_right_front_Node, cloud);

					break;
				case 4:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_right_back_Node, cloud);

					break;
				case 5:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_front_Node, cloud);

					break;
				case 6:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_back_Node, cloud);

					break;
				case 7:
					//�� �� ǰ
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_front_Node, cloud);

					break;
				case 8:
					//�� �� ��
					Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_back_Node, cloud);

					break;
				}

			}
		}
		//����root�ڵ��������ӽڵ��ڵ�k���ڵ�;
		void k_search_full(pcl::PointXYZ searchPoint, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud, int k)
		{
			if (k <= 0)
			{
				return;
			}
			if (!root->getHasCloud())
			{
				//û�е���;
				return;
			}
			if (root->isLeaf())
			{
				//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					if (pointIndex.size() <= k)
					{
						if (pointIndex.size() == 0) {
							//û�е㣬����Ѱ�Ҳ���λ��;
							pointIndex.push_back(root->getPoints()[root->getPoints()[i]]);
							pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
						}
						else
						{
							//Ѱ�����;
							float d = Distance(cloud->points[root->getPoints()[i]], searchPoint);
							int index = root->getPoints()[i];
							for (int j = 0; j < pointIndex.size(); j++) {
								if (d < pointSquaredDistance[j]) {
									pointIndex.insert(pointIndex.begin() + j, index);
									pointSquaredDistance.insert(pointSquaredDistance.begin() + j, d);
									if (pointIndex.size() > k) {
										//����k���㣬ɾ����������;
										pointIndex.pop_back();
										pointSquaredDistance.pop_back();
									}
									break;
								}
							}
						}
					}
				}
			}
			else
			{
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->top_left_front_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->top_left_back_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->top_right_front_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->top_right_back_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_front_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_back_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_front_Node, cloud, k);
				k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_back_Node, cloud, k);

			}
		}
		//�����뾶�������k���㣬��С��������;
		void k_search_radius(pcl::PointXYZ searchPoint, float length, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud, int k)
		{
			if (k <= 0)
			{
				return;
			}
			if (!root->getHasCloud())
			{
				//û�е���;
				return;
			}
			//ͬʱ����������û�н���,��÷���������;
			//���Ʊ߽�
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();
			//����߽�
			float point_left_border = searchPoint.x - length;
			float point_right_border = searchPoint.x + length;
			float point_front_border = searchPoint.y - length;
			float point_back_border = searchPoint.y + length;
			float point_bottom_border = searchPoint.z - length;
			float point_top_border = searchPoint.z + length;

			BOOLEAN flag = FALSE;//flag = false ��ʾû�н���;
			if ((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//����������;
				flag = TRUE;
			}
			else if ((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border))
			{
				//����������
				flag = TRUE;
			}
			else if ((point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//�����Ǻ�ǰ
				flag = TRUE;
			}
			if (flag)
			{
				//�н���
				if (root->isLeaf())
				{
					//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
					for (int i = 0; i < root->getPoints().size(); i++)
					{
						if ((cloud->points[root->getPoints()[i]].x >= point_left_border && cloud->points[root->getPoints()[i]].x <= point_right_border)
							&& (cloud->points[root->getPoints()[i]].y >= point_front_border && cloud->points[root->getPoints()[i]].y <= point_back_border)
							&& (cloud->points[root->getPoints()[i]].z >= point_bottom_border && cloud->points[root->getPoints()[i]].z <= point_top_border))
						{
							if (Distance(cloud->points[root->getPoints()[i]], searchPoint) - length < 1e23
								&& pointIndex.size() <= k)
							{
								if (pointIndex.size() == 0) {
									pointIndex.push_back(root->getPoints()[i]);
									pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
								}
								else
								{
									float d = Distance(cloud->points[root->getPoints()[i]], searchPoint);
									int index = root->getPoints()[i];
									for (int j = 0; j < pointIndex.size(); j++)
									{
										if (d < pointSquaredDistance[j])
										{
											pointIndex.insert(pointIndex.begin() + j, index);
											pointSquaredDistance.insert(pointSquaredDistance.begin() + j, d);

											if (pointIndex.size() > k)
											{
												pointIndex.pop_back();
												pointSquaredDistance.pop_back();
											}
											break;
										}
									}
								}
							}
						}
					}
				}
				else
				{
					//����Ҷ�ڵ�;
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->top_left_front_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->top_left_back_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->top_right_front_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->top_right_back_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_left_front_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_left_back_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_right_front_Node, cloud, k);
					k_search_radius(searchPoint, length, pointIndex, pointSquaredDistance, root->bottom_right_back_Node, cloud, k);

				}
			}
		}
		//���ٽ������ػ����ҵ�k����;
		void quick_k_search(pcl::PointXYZ searchPoint, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud, int k)
		{
			if (k <= 0) {
				return;
			}
			//�������ؽڵ��������k������;
			k_search_full(searchPoint, pointIndex, pointSquaredDistance, root, cloud, k);
			if (pointIndex.size() == k) {
				//����������k����;
				return;
			}
			else
			{
				pointIndex.clear();
				pointIndex.resize(0);
				pointIndex.reserve(0);
				pointSquaredDistance.clear();
				pointSquaredDistance.resize(0);
				pointSquaredDistance.reserve(0);
				//������һ��
				if (root->father_Node != NULL)
				{
					k_search_full(searchPoint, pointIndex, pointSquaredDistance, root->father_Node, cloud, k);
				}
			}
		}

	public:
		octree()
		{
			this->root = new octree_node_my();
			this->depth = 1;
		};
		~octree()
		{};
		//�������ȴ�����;
		void CreatTree_depth(PointCloudT::Ptr cloud, int depth)
		{
			//���õ���;
			this->setInputCloud(cloud);
			//���ø����Ϊ1��
			this->root->setDepth(1);
			//���ø��ĵ��Ʊ߽�
			float left = 0;
			float right = 0;
			float front = 0;
			float back = 0;
			float top = 0;
			float bottom = 0;
			float* left_border = &left;
			float* right_border = &right;
			float* front_border = &front;
			float* back_border = &back;
			float* top_border = &top;
			float* bottom_border = &bottom;
			int tree_depth = 0;
			int* tree_depth_ptr = &tree_depth;
			if (!this->getSize(cloud, left_border, right_border, front_border, back_border, bottom_border, top_border))
			{
				//����Ϊ�գ��޷���ȡ���Ʊ߽�;
				return;
			};
			this->root->setBorder(*left_border, right, front, back, bottom, top);
			//���ø��ĵ��������±�
			for (int i = 0; i < cloud->points.size(); i++)
			{
				this->root->HasCloud();
				this->root->addPointIndex(i);
			}
			//�������������Ƹ���Ϊ���Ƹ���;
			this->max_points_size = cloud->points.size() + 1;
			//�ݹ鴴����;
			CreatTree_root_depth(this->root, depth, cloud, tree_depth_ptr);
			this->setDepth(tree_depth);

		}

		//��������������ȴ�������
		void CreatTree_max_pointSize(PointCloudT::Ptr cloud, int max_pointSize)
		{
			//���õ���;
			this->setInputCloud(cloud);
			//���ø����Ϊ1��
			this->root->setDepth(1);
			//���ø��ĵ��Ʊ߽�
			float left = 0;
			float right = 0;
			float front = 0;
			float back = 0;
			float top = 0;
			float bottom = 0;
			float* left_border = &left;
			float* right_border = &right;
			float* front_border = &front;
			float* back_border = &back;
			float* top_border = &top;
			float* bottom_border = &bottom;
			int tree_depth = 0;
			int* tree_depth_ptr = &tree_depth;
			if (!this->getSize(cloud, left_border, right_border, front_border, back_border, bottom_border, top_border))
			{
				//����Ϊ�գ��޷���ȡ���Ʊ߽�;
				return;
			}
			this->root->setBorder(left, right, front, back, bottom, top);
			//���ø��ĵ��������±�
			for (int i = 0; i < cloud->points.size(); i++)
			{
				this->root->addPointIndex(i);
				this->root->HasCloud();
			}
			//�������������Ƹ���Ϊ���Ƹ���;
			this->max_points_size = max_pointSize;
			//�ݹ鴴����;
			CreatTree_root_maxPointsSize(this->root, max_pointSize, cloud, tree_depth_ptr);
			this->setDepth(tree_depth);
		}

		PointCloudT::Ptr  getInputCloud(PointCloudT::Ptr cloud)
		{
			return cloud;
		}

		octree_node_my* getRoot()
		{
			return this->root;
		}
		//���������㷨;
		BOOLEAN voxelSearch(pcl::PointXYZ searchPoint, std::vector <int>& pointIndex, std::vector<float>& pointSquaredDistance)
		{
			pointIndex.clear();
			pointIndex.resize(0);
			pointIndex.reserve(0);
			pointSquaredDistance.clear();
			pointSquaredDistance.resize(0);
			pointSquaredDistance.reserve(0);
			Vearch_root(searchPoint, pointIndex, pointSquaredDistance, this->root, cloud);
			return pointIndex.size() > 0;
		}
		//�뾶���������㷨;
		BOOLEAN radiusSearch(pcl::PointXYZ searchPoint, float radius, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance)
		{
			pointIndex.clear();
			pointIndex.resize(0);
			pointIndex.reserve(0);
			pointIndex.reserve(0);
			pointSquaredDistance.clear();
			pointSquaredDistance.resize(0);
			pointSquaredDistance.reserve(0);
			pointSquaredDistance.reserve(0);
			std::vector<int> pointIndex_temp;
			std::vector<float> pointSquaredDistance_temp;
			Search(searchPoint, radius, pointIndex_temp, pointSquaredDistance_temp, this->root, cloud);
			for (int i = 0; i < pointIndex_temp.size(); i++)
			{
				float distance = Distance(searchPoint, cloud->points[pointIndex_temp[i]]);
				if (distance <= radius)
				{
					pointIndex.push_back(pointIndex_temp[i]);
					pointSquaredDistance.push_back(pointSquaredDistance_temp[i]);
				}
			}
			return pointIndex.size() > 0;
		}
		//k��������;
		BOOLEAN QuickKSearch(pcl::PointXYZ searchPoint, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, int k)
		{
			pointIndex.clear();
			pointIndex.resize(0);
			pointIndex.reserve(0);
			pointSquaredDistance.clear();
			pointSquaredDistance.resize(0);
			pointSquaredDistance.reserve(0);
			octree_node_my* leaf = NULL;
			octree_node_my** leaf_ptr = &leaf;
			if (Voxel_point(searchPoint, this->root, cloud, leaf_ptr))
			{
				quick_k_search(searchPoint, pointIndex, pointSquaredDistance, leaf, cloud, k);
				return pointIndex.size() > 0;
			}
			return FALSE;
		}
		BOOLEAN KSearch(pcl::PointXYZ searchPoint, std::vector<int> &pointIndex, std::vector<float> &pointSquaredDistance, int k)
		{
			octree_node_my* leaf = NULL;
			octree_node_my** leaf_ptr = &leaf;
			//���Ҹõ����������ص�Ҷ�ӽڵ�;
			if (!Voxel_point(searchPoint, this->root, cloud, leaf_ptr))
			{
				return FALSE;
			}
			//��Ҷ�ڵ���ݿ����ѵ�k����;
			quick_k_search(searchPoint, pointIndex, pointSquaredDistance, leaf, cloud, k);
			if (pointIndex.size() <= 0)
			{
				return FALSE;
			}
			float radius = pointSquaredDistance[pointSquaredDistance.size() - 1];
			//���а뾶k������;
			pointIndex.clear();
			pointIndex.resize(0);
			pointIndex.reserve(0);
			pointSquaredDistance.clear();
			pointSquaredDistance.resize(0);
			pointSquaredDistance.reserve(0);

			k_search_radius(searchPoint, radius, pointIndex, pointSquaredDistance, this->root, cloud, k);
			return pointIndex.size() > 0;
		}
		PointCloudT::Ptr getInputCloud()
		{
			return cloud;
		}
		int getDepth()
		{
			return this->depth;
		}
		void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, octree_node_my* root, PointCloudT::Ptr cloud)
		{

			if (!root->getHasCloud())
			{
				//û�е���;
				return;
			}
			if (root->isLeaf())
			{
				string name = to_string(root->getX_min()) + " " + to_string(root->getX_Max()) + " " + to_string(root->getY_min()) + " " + to_string(root->getY_Max()) + " " + to_string(root->getZ_min()) + " " + to_string(root->getZ_Max());
				//cout << name;
				
				viewer->addCube(root->getX_min(), root->getX_Max(), root->getY_min(), root->getY_Max(), root->getZ_min(), root->getZ_Max(), 1.0, 1.0, 0.0, name);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
			}
			else {
				visualize(viewer, root->top_left_front_Node, cloud);
				visualize(viewer, root->top_left_back_Node, cloud);
				visualize(viewer, root->top_right_front_Node, cloud);
				visualize(viewer, root->top_right_back_Node, cloud);
				visualize(viewer, root->bottom_left_front_Node, cloud);
				visualize(viewer, root->bottom_left_back_Node, cloud);
				visualize(viewer, root->bottom_right_front_Node, cloud);
				visualize(viewer, root->bottom_right_back_Node, cloud);

			}

		}
	};
}