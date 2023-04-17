#pragma once

#include <pcl/point_types.h>
#include <iostream>
#include <math.h>

namespace octree_my 
{
	// 定义八叉树节点类
	class octree_node_my 
	{
	private:
		//是否为叶子节点;
		BOOLEAN IsLeaf;
		//是否为空叶子节点;
		BOOLEAN hasCloud;
		
		//叶子节点属性
		//放歌顶点;
		float x_min = 0, x_max = 0;
		float y_min = 0, y_max = 0;
		float z_min = 0, z_max = 0;
		//点云下标;
		std::vector<int> pointIndex;
		//所在深度
		int depth = 0;

	public:
		//非叶子节点属性;
		octree_node_my * top_left_front_Node = NULL;
		octree_node_my * top_left_back_Node = NULL;
		octree_node_my * top_right_front_Node = NULL;
		octree_node_my * top_right_back_Node = NULL;
		octree_node_my * bottom_left_front_Node = NULL;
		octree_node_my * bottom_left_back_Node = NULL;
		octree_node_my * bottom_right_front_Node = NULL;
		octree_node_my * bottom_right_back_Node = NULL;
		//构造函数
		octree_node_my()
		{
			IsLeaf = FALSE;//默认不是叶子节点;
			hasCloud = FALSE;//默认不含有点云;
			pointIndex.clear();//清空点云下标;
			
		};
		//析构函数
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



	//定义八叉树
	class octree_my 
	{
	private:
		//根节点：
		octree_node_my * root = NULL;
		//树的深度;
		int depth = 0;
		//体素内最大点个数;
		int max_points_size = 0;
		//点云；
		PointCloudT::Ptr cloud = nullptr;

		~octree_my()
		{};
		void setInputCloud(PointCloudT::Ptr cloud)
		{
			this->cloud = cloud;
		}
		
		//获取点的边界;
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
		//递归算法;
		//root为从当前节点创建，depth
		void CreatTree_root_depth(octree_node_my* root, int max_depth, PointCloudT::Ptr cloud)
		{
			if (root->getDepth() == max_depth) {
				//达到最大深度，设为叶节点;
				root->setLeaf();
				return;
			}
			else if (!root->getHasCloud()){
				//没有点，设为叶节点;
				root->setLeaf();
				return;
			}
			else {
				//定义分支，设置深度;
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

				//找出划分尺度;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//设置边框
				float left_border;
				float right_border;
				float front_border;
				float back_border;
				float bottom_border;
				float top_border;
				//上 左 前
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//上 左 后
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//上 右 前
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//上 右 后
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 左 前
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 左 后
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 右 前
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 右 后
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);

				//分配点云
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					int flag = 1;
					if (cloud->points[root->getPoints()[i]].z > z_center)
					{
						//上 flag = 1;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//上 左 flag = 1;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//上 左 前 flag = 1;
							} else
							{
								flag += 1;
								//上 左 后 flag = 2;
							}
						} else
						{
							flag += 2;
							//上 右 flag = 3;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//上 右 前 flag = 3;
							}
							else
							{
								flag += 1;
								//上 右 后 flag = 4;
							}

						}
					}else 
					{
						flag += 4;
						//下 flag = 5;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//下 左 flag = 5;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//下 左 前 flag = 5;
							}
							else
							{
								flag += 1;
								//下 左 后 flag = 6;
							}
						}
						else
						{
							flag += 2;
							//下 右 flag = 7;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//下 右 前 flag = 7;
							}
							else
							{
								flag += 1;
								//下 右 后 flag = 8;
							}
						}
					}
					switch (flag) {
						case 1:
							//左 上 前
							root->top_left_front_Node->HasCloud();
							root->top_left_front_Node->addPointIndex(i);
							break;
						case 2:
							//左 上 后
							root->top_left_back_Node->HasCloud();
							root->top_left_back_Node->addPointIndex(i);
							break;
						case 3:
							//右 上 前
							root->top_right_front_Node->HasCloud();
							root->top_right_front_Node->addPointIndex(i);
							break;
						case 4:
							//右 上 后
							root->top_right_back_Node->HasCloud();
							root->top_right_back_Node->addPointIndex(i);
							break;
						case 5:
							//左 下 前
							root->bottom_left_front_Node->HasCloud();
							root->bottom_left_front_Node->addPointIndex(i);
							break;
						case 6:
							//左 下 后
							root->bottom_left_back_Node->HasCloud();
							root->bottom_left_back_Node->addPointIndex(i);
							break;
						case 7:
							//右 下 前
							root->bottom_right_front_Node->HasCloud();
							root->bottom_right_front_Node->addPointIndex(i);
							break;
						case 8:
							//右 下 后
							root->bottom_right_back_Node->HasCloud();
							root->bottom_right_back_Node->addPointIndex(i);
							break;
					}
				}
			}
			
			//各个子树递归调用;
			CreatTree_root_depth(root->top_left_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->top_left_back_Node, max_depth, cloud);
			CreatTree_root_depth(root->top_right_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->top_right_back_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_left_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_left_back_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_right_front_Node, max_depth, cloud);
			CreatTree_root_depth(root->bottom_right_back_Node, max_depth, cloud);
		}
		//计算两点之间距离;
		float Distance(pcl::PointXYZ point1, pcl::PointXYZ point2)
		{
			float sum = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
			return sqrt(sum);
		}
		//root为从当前节点创建，体素内最多点云数量;
		void CreatTree_root_maxPointsSize(octree_node_my* root, int max_points_size, PointCloudT::Ptr cloud)
		{
			if (!root->getHasCloud()) {
				//没有点，设为叶节点;
				root->setLeaf();
				return;
			}
			else if (root->getPoints().size() <= max_points_size) {
				//该节点内点的个数小于要求的体素最大点云数;
				root->setLeaf();
				return;
			}
			else {
				//定义分支，设置深度;
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

				//找出划分尺度;
				float x_length = 0.5 * (root->getX_Max() - root->getX_min());
				float y_length = 0.5 * (root->getY_Max() - root->getY_min());
				float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
				float x_center = 0.5 * (root->getX_Max() + root->getX_min());
				float y_center = 0.5 * (root->getY_Max() + root->getY_min());
				float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
				//设置边框
				float left_border;
				float right_border;
				float front_border;
				float back_border;
				float bottom_border;
				float top_border;
				//上 左 前
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//上 左 后
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//上 右 前
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//上 右 后
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				top_border = root->getZ_Max();
				bottom_border = top_border - z_length;
				root->top_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 左 前
				left_border = root->getX_min();
				right_border = left_border + x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 左 后
				left_border = root->getX_min();
				right_border = left_border + x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_left_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 右 前
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				front_border = root->getY_min();
				back_border = front_border + y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_right_front_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
				//下 右 后
				right_border = root->getX_Max();
				left_border = right_border - x_length;
				back_border = root->getY_Max();
				front_border = back_border - y_length;
				bottom_border = root->getZ_Max();
				top_border = bottom_border + z_length;
				root->bottom_right_back_Node->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);

				//分配点云
				for (int i = 0; i < root->getPoints().size(); i++)
				{
					int flag = 1;
					if (cloud->points[root->getPoints()[i]].z > z_center)
					{
						//上 flag = 1;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//上 左 flag = 1;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//上 左 前 flag = 1;
							}
							else
							{
								flag += 1;
								//上 左 后 flag = 2;
							}
						}
						else
						{
							flag += 2;
							//上 右 flag = 3;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//上 右 前 flag = 3;
							}
							else
							{
								flag += 1;
								//上 右 后 flag = 4;
							}

						}
					}
					else
					{
						flag += 4;
						//下 flag = 5;
						if (cloud->points[root->getPoints()[i]].x < x_center)
						{
							//下 左 flag = 5;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//下 左 前 flag = 5;
							}
							else
							{
								flag += 1;
								//下 左 后 flag = 6;
							}
						}
						else
						{
							flag += 2;
							//下 右 flag = 7;
							if (cloud->points[root->getPoints()[i]].y < y_center)
							{
								//下 右 前 flag = 7;
							}
							else
							{
								flag += 1;
								//下 右 后 flag = 8;
							}
						}
					}
					switch (flag) {
					case 1:
						//左 上 前
						root->top_left_front_Node->HasCloud();
						root->top_left_front_Node->addPointIndex(i);
						break;
					case 2:
						//左 上 后
						root->top_left_back_Node->HasCloud();
						root->top_left_back_Node->addPointIndex(i);
						break;
					case 3:
						//右 上 前
						root->top_right_front_Node->HasCloud();
						root->top_right_front_Node->addPointIndex(i);
						break;
					case 4:
						//右 上 后
						root->top_right_back_Node->HasCloud();
						root->top_right_back_Node->addPointIndex(i);
						break;
					case 5:
						//左 下 前
						root->bottom_left_front_Node->HasCloud();
						root->bottom_left_front_Node->addPointIndex(i);
						break;
					case 6:
						//左 下 后
						root->bottom_left_back_Node->HasCloud();
						root->bottom_left_back_Node->addPointIndex(i);
						break;
					case 7:
						//右 下 前
						root->bottom_right_front_Node->HasCloud();
						root->bottom_right_front_Node->addPointIndex(i);
						break;
					case 8:
						//右 下 后
						root->bottom_right_back_Node->HasCloud();
						root->bottom_right_back_Node->addPointIndex(i);
						break;
					}
				}
			}

			//各个子树递归调用;
			CreatTree_root_maxPointsSize(root->top_left_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->top_left_back_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->top_right_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->top_right_back_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_left_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_left_back_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_right_front_Node, max_points_size, cloud);
			CreatTree_root_maxPointsSize(root->bottom_right_back_Node, max_points_size, cloud);
		}
		//搜索一个方框内的点云;
		//搜索点、边长、搜索点下标、搜索点距离;
		void Search(pcl::PointXYZ searchPoint, float length, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud)
		{
			if (!root->getHasCloud())
			{
				//没有点云;
				return;
			}
			//同时有两个坐标没有交集,则该方框在外面;
			//点云边界
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();
			//方框边界
			float point_left_border = searchPoint.x - length;
			float point_right_border = searchPoint.x + length;
			float point_front_border = searchPoint.y - length;
			float point_back_border = searchPoint.y + length;
			float point_bottom_border = searchPoint.z - length;
			float point_top_border = searchPoint.z + length;

			BOOLEAN flag = FALSE;//flag = false 表示没有交集;
			if ((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//不考虑上下;
				flag = TRUE;
			} else if((point_front_border <= cloud_back_border || point_back_border >= cloud_front_border) && (point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border))
			{
				//不考虑左右
				flag = TRUE;
			} else if((point_bottom_border <= cloud_top_border || point_top_border >= cloud_bottom_border) && (point_left_border <= cloud_right_border || point_right_border >= cloud_left_border))
			{
				//不考虑前后
				flag = TRUE;
			}
			if (flag)
			{
 				//有交集
				if (root->isLeaf())
				{
 					//是叶节点,合并方框内点，返回;
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
 					//不是叶节点;
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
		//体素搜索
		void Vearch_root(pcl::PointXYZ searchPoint, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance, octree_node_my* root, PointCloudT::Ptr cloud)
		{
			if (!root->getHasCloud())
			{
				//没有点云;
				return;
			}
			//同时有两个坐标没有交集,则该方框在外面;
			//点云边界
			float cloud_left_border = root->getX_min();
			float cloud_right_border = root->getX_Max();
			float cloud_front_border = root->getY_min();
			float cloud_back_border = root->getY_Max();
			float cloud_bottom_border = root->getZ_min();
			float cloud_top_border = root->getZ_Max();


			BOOLEAN flag = FALSE;//flag = false 表示没有交集;
			if (searchPoint.x <= cloud_right_border && searchPoint.x >= cloud_left_border
				&& searchPoint.y <= cloud_back_border && searchPoint.y >= cloud_front_border
				&& searchPoint.z <= cloud_top_border && searchPoint.z >= cloud_bottom_border)
			{
				//点在里面;
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
					//是叶节点,合并方框内点，返回;
					for (int i = 0; i < root->getPoints().size(); i++)
					{
						pointIndex.push_back(i);
						pointSquaredDistance.push_back(Distance(cloud->points[root->getPoints()[i]], searchPoint));
					}
					return;
				}
				else
				{
					//找出划分尺度;
					float x_length = 0.5 * (root->getX_Max() - root->getX_min());
					float y_length = 0.5 * (root->getY_Max() - root->getY_min());
					float z_length = 0.5 * (root->getZ_Max() - root->getZ_min());
					float x_center = 0.5 * (root->getX_Max() + root->getX_min());
					float y_center = 0.5 * (root->getY_Max() + root->getY_min());
					float z_center = 0.5 * (root->getZ_Max() + root->getZ_min());
					//不是叶节点;
					int flag = 1;
					if (searchPoint.z > z_center)
					{
						//上 flag = 1;
						if (searchPoint.x < x_center)
						{
							//上 左 flag = 1;
							if (searchPoint.y < y_center)
							{
								//上 左 前 flag = 1;
							}
							else
							{
								flag += 1;
								//上 左 后 flag = 2;
							}
						}
						else
						{
							flag += 2;
							//上 右 flag = 3;
							if (searchPoint.y < y_center)
							{
								//上 右 前 flag = 3;
							}
							else
							{
								flag += 1;
								//上 右 后 flag = 4;
							}

						}
					}
					else
					{
						flag += 4;
						//下 flag = 5;
						if (searchPoint.x < x_center)
						{
							//下 左 flag = 5;
							if (searchPoint.y < y_center)
							{
								//下 左 前 flag = 5;
							}
							else
							{
								flag += 1;
								//下 左 后 flag = 6;
							}
						}
						else
						{
							flag += 2;
							//下 右 flag = 7;
							if (searchPoint.y < y_center)
							{
								//下 右 前 flag = 7;
							}
							else
							{
								flag += 1;
								//下 右 后 flag = 8;
							}
						}
						switch (flag) {
						case 1:
							//左 上 前
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_left_front_Node, cloud);

							break;
						case 2:
							//左 上 后
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_left_back_Node, cloud);

							break;
						case 3:
							//右 上 前
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_right_front_Node, cloud);

							break;
						case 4:
							//右 上 后
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->top_right_back_Node, cloud);

							break;
						case 5:
							//左 下 前
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_front_Node, cloud);

							break;
						case 6:
							//左 下 后
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_left_back_Node, cloud);

							break;
						case 7:
							//右 下 前
							Vearch_root(searchPoint, pointIndex, pointSquaredDistance, root->bottom_right_front_Node, cloud);

							break;
						case 8:
							//右 下 后
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
		//按最大深度创建树;
		void CreatTree_depth(PointCloudT::Ptr cloud, int depth)
		{
			//设置点云;
			this->setInputCloud(cloud);
			//设置根深度为1；
			this->root->setDepth(1);
			//设置根的点云边界
			float left_border;
			float right_border;
			float front_border;
			float back_border;
			float top_border;
			float bottom_border;
			getSize(cloud, &left_border, &right_border, &front_border, &back_border, &bottom_border, &top_border);
			this->root->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
			//设置根的点云数据下标
			for (int i = 0; i < cloud->points.size(); i++)
			{
				this->root->addPointIndex(i);
			}
			//设置体素最大点云个数为点云个数;
			this->max_points_size = cloud->points.size() + 1;
			//递归创建树;
			CreatTree_root_depth(this->root, depth, cloud);
			
		}

		//按体素最大点云深度创建树；
		void CreatTree_max_pointSize(PointCloudT::Ptr cloud, int max_pointSize)
		{
			//设置点云;
			this->setInputCloud(cloud);
			//设置根深度为1；
			this->root->setDepth(1);
			//设置根的点云边界
			float left_border;
			float right_border;
			float front_border;
			float back_border;
			float top_border;
			float bottom_border;
			getSize(cloud, &left_border, &right_border, &front_border, &back_border, &bottom_border, &top_border);
			this->root->setBorder(left_border, right_border, front_border, back_border, bottom_border, top_border);
			//设置根的点云数据下标
			for (int i = 0; i < cloud->points.size(); i++)
			{
				this->root->addPointIndex(i);
			}
			//设置体素最大点云个数为点云个数;
			this->max_points_size = max_pointSize;
			//递归创建树;
			CreatTree_root_maxPointsSize(this->root, max_pointSize, cloud);
		}

		PointCloudT::Ptr  getInputCloud(PointCloudT::Ptr cloud)
		{
			return cloud;
		}

		//体素搜索算法;
		BOOLEAN voxelSearch(pcl::PointXYZ searchPoint, std::vector<int> pointIndex, std::vector<float> pointSquaredDistance)
		{
			Vearch_root(searchPoint, pointIndex, pointSquaredDistance, this->root, cloud);
			return pointIndex.size() > 0;
		}
		//半径邻域搜索算法;
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