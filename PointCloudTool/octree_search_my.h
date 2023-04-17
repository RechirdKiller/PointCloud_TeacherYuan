#pragma once

#include <pcl/point_types.h>
#include <iostream>
#include <math.h>

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
			pointIndex.clear();//��յ����±�;
			
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
	class octree_my 
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

		~octree_my()
		{};
		void setInputCloud(PointCloudT::Ptr cloud)
		{
			this->cloud = cloud;
		}
		
		//��ȡ��ı߽�;
		void getSize(PointCloudT::Ptr cloud, float *x_min, float *x_max,float  *y_min, float *y_max, float *z_min,float *z_max)
		{
			if (cloud->empty()) {
				return;
			}
			float X_Max = -1;
			float Y_Max = -1;
			float Z_Max = -1;
			float X_Min = INT_MAX;
			float Y_Min = INT_MAX;
			float Z_Min = INT_MAX;
			for (int i = 0; i < cloud->points.size(); i++) {
				if (cloud->points[i].x > X_Max) {
					X_Max = cloud->points[i].x;
				}

				if (cloud->points[i].x < X_Min) {
					X_Min = cloud->points[i].x;
				}
				if (cloud->points[i].y > Y_Max) {
					Y_Max = cloud->points[i].y;
				}
				if (cloud->points[i].y < Y_Min) {
					Y_Min = cloud->points[i].y;
				}
				if (cloud->points[i].z > Z_Max) {
					Z_Max = cloud->points[i].z;
				}
				if (cloud->points[i].z < Z_Min) {
					Z_Min = cloud->points[i].z;
				}
			}
			x_min = &X_Min;
			x_max = &X_Max;
			y_min = &Y_Min;
			y_max = &Y_Max;
			z_min = &Z_Min;
			z_max = &Z_Max;
		}
		//�ݹ��㷨;
		//rootΪ�ӵ�ǰ�ڵ㴴����depth
		void CreatTree_root_depth(octree_node_my* root, int max_depth, PointCloudT::Ptr cloud)
		{
			if (root->getDepth() == max_depth) {
				//�ﵽ�����ȣ���ΪҶ�ڵ�;
				root->setLeaf();
				return;
			}
			else if (!root->getHasCloud()){
				//û�е㣬��ΪҶ�ڵ�;
				root->setLeaf();
				return;
			}
			else {
				//�����֧���������;
				root->top_left_front_Node = &(octree_node_my());
				root->top_left_front_Node->setDepth(root->getDepth() + 1);
				root->top_left_back_Node = &(octree_node_my());
				root->top_left_back_Node->setDepth(root->getDepth() + 1);
				root->top_right_front_Node = &(octree_node_my());
				root->top_right_front_Node->setDepth(root->getDepth() + 1);
				root->top_right_back_Node = &(octree_node_my());
				root->top_right_back_Node->setDepth(root->getDepth() + 1);
				root->bottom_left_front_Node = &(octree_node_my());
				root->bottom_left_front_Node->setDepth(root->getDepth() + 1);
				root->bottom_left_back_Node = &(octree_node_my());
				root->bottom_left_back_Node->setDepth(root->getDepth() + 1);
				root->bottom_right_front_Node = &(octree_node_my());
				root->bottom_right_front_Node->setDepth(root->getDepth() + 1);
				root->bottom_right_back_Node = &(octree_node_my());
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
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
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
							} else
							{
								flag += 1;
								//�� �� �� flag = 2;
							}
						} else
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
					}else 
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
							root->top_left_front_Node->addPointIndex(i);
							break;
						case 2:
							//�� �� ��
							root->top_left_back_Node->HasCloud();
							root->top_left_back_Node->addPointIndex(i);
							break;
						case 3:
							//�� �� ǰ
							root->top_right_front_Node->HasCloud();
							root->top_right_front_Node->addPointIndex(i);
							break;
						case 4:
							//�� �� ��
							root->top_right_back_Node->HasCloud();
							root->top_right_back_Node->addPointIndex(i);
							break;
						case 5:
							//�� �� ǰ
							root->bottom_left_front_Node->HasCloud();
							root->bottom_left_front_Node->addPointIndex(i);
							break;
						case 6:
							//�� �� ��
							root->bottom_left_back_Node->HasCloud();
							root->bottom_left_back_Node->addPointIndex(i);
							break;
						case 7:
							//�� �� ǰ
							root->bottom_right_front_Node->HasCloud();
							root->bottom_right_front_Node->addPointIndex(i);
							break;
						case 8:
							//�� �� ��
							root->bottom_right_back_Node->HasCloud();
							root->bottom_right_back_Node->addPointIndex(i);
							break;
					}
				}
			}
			
			//���������ݹ����;
			CreatTree_root_depth(root->top_left_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->top_left_back_Node, max_depth, cloud);
			CreatTree_root_depth(root->top_right_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->top_right_back_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_left_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_left_back_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_right_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_right_back_Node, max_depth, cloud);
		}
		//��������֮�����;
		float Distance(pcl::PointXYZ point1, pcl::PointXYZ point2)
		{
			float sum = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
			return sqrt(sum);
		}
		//rootΪ�ӵ�ǰ�ڵ㴴��������������������;
		void CreatTree_root_maxPointsSize(octree_node_my* root, int max_points_size, PointCloudT::Ptr cloud)
		{
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
				root->top_left_front_Node = &(octree_node_my());
				root->top_left_front_Node->setDepth(root->getDepth() + 1);
				root->top_left_back_Node = &(octree_node_my());
				root->top_left_back_Node->setDepth(root->getDepth() + 1);
				root->top_right_front_Node = &(octree_node_my());
				root->top_right_front_Node->setDepth(root->getDepth() + 1);
				root->top_right_back_Node = &(octree_node_my());
				root->top_right_back_Node->setDepth(root->getDepth() + 1);
				root->bottom_left_front_Node = &(octree_node_my());
				root->bottom_left_front_Node->setDepth(root->getDepth() + 1);
				root->bottom_left_back_Node = &(octree_node_my());
				root->bottom_left_back_Node->setDepth(root->getDepth() + 1);
				root->bottom_right_front_Node = &(octree_node_my());
				root->bottom_right_front_Node->setDepth(root->getDepth() + 1);
				root->bottom_right_back_Node = &(octree_node_my());
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
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ǰ
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//�� �� ��
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
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
						root->top_left_front_Node->addPointIndex(i);
						break;
					case 2:
						//�� �� ��
						root->top_left_back_Node->HasCloud();
						root->top_left_back_Node->addPointIndex(i);
						break;
					case 3:
						//�� �� ǰ
						root->top_right_front_Node->HasCloud();
						root->top_right_front_Node->addPointIndex(i);
						break;
					case 4:
						//�� �� ��
						root->top_right_back_Node->HasCloud();
						root->top_right_back_Node->addPointIndex(i);
						break;
					case 5:
						//�� �� ǰ
						root->bottom_left_front_Node->HasCloud();
						root->bottom_left_front_Node->addPointIndex(i);
						break;
					case 6:
						//�� �� ��
						root->bottom_left_back_Node->HasCloud();
						root->bottom_left_back_Node->addPointIndex(i);
						break;
					case 7:
						//�� �� ǰ
						root->bottom_right_front_Node->HasCloud();
						root->bottom_right_front_Node->addPointIndex(i);
						break;
					case 8:
						//�� �� ��
						root->bottom_right_back_Node->HasCloud();
						root->bottom_right_back_Node->addPointIndex(i);
						break;
					}
				}
			}

			//���������ݹ����;
			CreatTree_root_maxPointsSize(root->top_left_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->top_left_back_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->top_right_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->top_right_back_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_left_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_left_back_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_right_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_right_back_Node, max_points_size, cloud);
		}
		//����һ�������ڵĵ���;
		//�����㡢�߳����������±ꡢ���������;
		void Search(pcl::PointXYZ searchPoint, float length, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud)
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
			} else if((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border))
			{
				//����������
				flag = TRUE;
			} else if((point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//������ǰ��
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
							pointIndex.push_back(i);
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
		void Vearch_root(pcl::PointXYZ searchPoint, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud)
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


			BOOLEAN flag = FALSE;//flag = false ��ʾû�н���;
			if (searchPoint.x <= cloud_right_border && searchPoint.x >= cloud_left_border
				&& searchPoint.y <= cloud_back_border && searchPoint.y >= cloud_front_border
				&& searchPoint.z <= cloud_top_border && searchPoint.z >= cloud_bottom_border)
			{
				//��������;
				flag = TRUE;
			}
			else
			{
				return;
			}
			
			if (flag)
			{
				if (root->isLeaf())
				{
					//��Ҷ�ڵ�,�ϲ������ڵ㣬����;
					for (int i = 0; i < root->getPoints().size(); i++)
					{
						pointIndex.push_back(i);
						pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
					}
					return;
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
			}
		};
	public:
		octree_my()
		{
			root = &(octree_node_my());
			this->depth = 1;
		};
		//�������ȴ�����;
		void CreatTree_depth(PointCloudT::Ptr cloud, int depth)
		{
			//���õ���;
			this->setInputCloud(cloud);
			//���ø����Ϊ1��
			this->root->setDepth(1);
			//���ø��ĵ��Ʊ߽�
			float left_border;
			float right_border;
			float front_border;
			float back_border;
			float top_border;
			float bottom_border;
			getSize(cloud, &left_border, &right_border, &front_border, &back_border, &bottom_border, &top_border);
			this->root->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
			//���ø��ĵ��������±�
			for (int i = 0; i < cloud->points.size(); i++)
			{
				this->root->addPointIndex(i);
			}
			//�������������Ƹ���Ϊ���Ƹ���;
			this->max_points_size = cloud->points.size() + 1;
			//�ݹ鴴����;
			CreatTree_root_depth(this->root, depth, cloud);
			
		}

		//��������������ȴ�������
		void CreatTree_max_pointSize(PointCloudT::Ptr cloud, int max_pointSize)
		{
			//���õ���;
			this->setInputCloud(cloud);
			//���ø����Ϊ1��
			this->root->setDepth(1);
			//���ø��ĵ��Ʊ߽�
			float left_border;
			float right_border;
			float front_border;
			float back_border;
			float top_border;
			float bottom_border;
			getSize(cloud, &left_border, &right_border, &front_border, &back_border, &bottom_border, &top_border);
			this->root->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
			//���ø��ĵ��������±�
			for (int i = 0; i < cloud->points.size(); i++)
			{
				this->root->addPointIndex(i);
			}
			//�������������Ƹ���Ϊ���Ƹ���;
			this->max_points_size = max_pointSize;
			//�ݹ鴴����;
			CreatTree_root_maxPointsSize(this->root, max_pointSize, cloud);
		}

		PointCloudT::Ptr  getInputCloud(PointCloudT::Ptr cloud)
		{
			return cloud;
		}

		//���������㷨;
		BOOLEAN voxelSearch(pcl::PointXYZ searchPoint, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance)
		{
			Vearch_root(searchPoint, pointIndex, pointSquaredDistance, this->root, cloud);
			return pointIndex.size() > 0;
		}
		//�뾶���������㷨;
		BOOLEAN radiusSearch(pcl::PointXYZ searchPoint, float radius, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance)
		{
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
	};
}