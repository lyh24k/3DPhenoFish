#pragma once

#include <iostream>
#include <io.h>
#include <direct.h>
#include <string>
#include <cmath>
#include <exception>
#include <algorithm>
//#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/segmentation/supervoxel_clustering.h>  
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <Eigen/Dense>
#include <set>
#include <opencv2/opencv.hpp>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>

#include "SQL.h"

using namespace std;
using namespace cv;
using namespace Eigen;

#define Random(x) (rand() % x)

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZL PointTL;
typedef pcl::PointCloud<PointTL> PointCloudTL;
typedef pcl::PointXYZ Pointxyz;
typedef pcl::PointCloud<Pointxyz> PointCloudXYZ;


struct RGB {
	int r;
	int g;
	int b;
};
//存顶点信息
struct _xyz {
	double x;
	double y;
	double z;
	int r = 0;
	int g = 0;
	int b = 0;
};
//纹理坐标
struct _xy {
	double x;
	double y;
};
//法线
struct _normal {
	double nx;
	double ny;
	double nz;
};
//面
struct _trif {
	int v[3];
	int t[3];
	int n[3];
};
struct _BoundRect {
	Pointxyz centerpoint;
	float x;
	float y;
	float z;
};

struct ThreeKeys {
	double dorsalsx;
	double tailsx;
	double headex;
};

struct ArcPara {
	double arc_height;
	double arc_width;
	double arc_path;
};
struct TriAngle {
	double pectoral_angle;
	double dorsal_angle;
	double pelvic_angle;
};

class PreTreatment
{
public:
	PreTreatment(string fishname_);
public:
	
	int getfilesname();
	PointCloudT::Ptr DownSample(PointCloudT::Ptr cloud);
	PointCloudT::Ptr RemovePlane(PointCloudT::Ptr cloud);
	PointCloudTL::Ptr MorphologyOpen(PointCloudTL::Ptr cloud, double th);
	
	int Allin(string filename);
	PointCloudT::Ptr PointCloudScale(double k, PointCloudT::Ptr cloud);
	
	PointCloudT::Ptr Removebody(PointCloudT::Ptr cloud);
	PointCloudT::Ptr FilterZ(PointCloudT::Ptr cloud);
	PointCloudT::Ptr DoNSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr EuclideanSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_, std::vector <pcl::PointIndices> clusters_, float r, float g, float b);
	PointCloudT::Ptr OverSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr Label2Color(pcl::PointCloud<PointTL>::Ptr cloud);
	PointCloudT::Ptr getBoundary(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getPlane(PointCloudT::Ptr cloud);
	PointCloudT::Ptr BoundaryGrowth(PointCloudT::Ptr plane, PointCloudT::Ptr boundary);
	int getpcdfiles(vector<string> &filepaths, vector<string> &filenames, string folder);
	PointCloudT::Ptr getfishonly(PointCloudT::Ptr cloud);
	int pcdBatch(string folder);
	int FindPrincipalDir(PointCloudT::Ptr cloud);
	bool getfishheaddir(PointCloudT::Ptr cloud);
	PointCloudT::Ptr trasformfishaxis(PointCloudT::Ptr cloud);
	bool turnZaxis();
	bool IsMirr(PointCloudT::Ptr cloud);

	//int FacetSegmentation(PointCloudT::Ptr cloud);
	bool FacetSegmentation(PointCloudT::Ptr cloud, PointCloudT::Ptr &facet_grow_cloud,double normal_im, double color_im, double all_thresold);//小面分割
	bool FacetSegmentationRGB(PointCloudT::Ptr cloud, PointCloudT::Ptr &facet_grow_cloud, double normal_im, double color_im, double all_thresold);
	int segmentheadandbody(PointCloudT::Ptr cloud, PointCloudT &headcloud, PointCloudT &bodycloud);
	int cloudBin(PointCloudT::Ptr cloud);
	int getharriskeypoint(PointCloudT::Ptr cloud);
	PointCloudT::Ptr segmentbycurvature(PointCloudT::Ptr cloud,int maxpoints);
	PointCloudT::Ptr regiongrowingseg(PointCloudT::Ptr cloud, vector<pcl::PointIndices> &clusters);
	int getkeypoint(vector<pcl::PointIndices> clusters, PointCloudT::Ptr cloud);
	Mat projectionOnXY(PointCloudT::Ptr cloud);
	int SegmentOn2D(Mat image);
	PointCloudTL::Ptr getfins(pcl::PointCloud<PointTL>::Ptr cloud);
	PointCloudTL::Ptr SortByX(pcl::PointCloud<PointTL>::Ptr cloud);
	PointCloudT::Ptr SortByX(PointCloudT::Ptr cloud);//按X轴排序
	PointCloudT::Ptr gettailonly(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getdorsalfinonly(PointCloudT::Ptr cloud, vector<pcl::PointIndices> clusters, vector<PointT> avgxyz);
	PointCloudT::Ptr getanalfinonly(PointCloudT::Ptr cloud, vector<pcl::PointIndices> clusters, vector<PointT> avgxyz);
	float CalTwoPointDistance(PointT point1, PointT point2);
	_BoundRect getBoundRect(PointCloudT::Ptr cloud);
	PointT getNearestpoint(PointCloudT::Ptr cloud, PointT point);
	int Drawkeypoints();
	
	
	void RGBA2RGB(PointT &rgba, pcl::PointXYZRGB &rgb);
	void RGBA2XYZ(PointCloudT::Ptr cloudrgb, PointCloudXYZ::Ptr &cloudxyz);
	bool getbelly(PointCloudT::Ptr cloud, PointCloudT::Ptr &belly_cloud);
	int getheadpoints(PointCloudT::Ptr cloud);
	int getTailHandlepoints(PointCloudT::Ptr cloud);
	int getfishbody(PointCloudT::Ptr cloud);
	int getfisheyes(PointCloudT::Ptr cloud);
	int gettwofinscenter(PointCloudT::Ptr cloud, PointT &p1, PointT &p2);
	int gettwofinscenter(PointCloudT::Ptr cloud, PointT &p1, PointT &p2,double &p1Length,double &p2Length);
	int getfakekeypoints(PointCloudT::Ptr cloud);
	
	int getfakedorsalfin(PointCloudT::Ptr cloud);//beiqi
	int getfakeanalfin(PointCloudT::Ptr cloud);//tunqi
	int getfaketailfin(PointCloudT::Ptr cloud);//weiqi
	int getfakepectoralfin(PointCloudT::Ptr cloud);//xiongqi
	int getfakepelvicfin(PointCloudT::Ptr cloud);//fuqi
	int gettail(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getfullfish(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getfullfishbyfacet(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getfishbodyformesh(PointCloudT::Ptr cloud);
	void Triagulation(PointCloudT::Ptr cloud);
	pcl::PolygonMesh PoissonRemesh(PointCloudT::Ptr cloud);
	double getVolume(PointCloudT::Ptr cloud);
	void getSurfaceArea(PointCloudT::Ptr cloud);
	void getPhenotype();
	void InsertData();
	//sint getfakebody(PointCloudT::Ptr cloud);//fuqi

public:
	PointCloudT::Ptr Obj2Pcd(string filename);
	PointCloudT::Ptr PreProcess(PointCloudT::Ptr cloud);
	int Allin();
	map<string, pcl::PointXYZRGB> extractKeyPoints();//获取关键点
	pcl::PointXYZRGB RestorePoint(pcl::PointXYZRGB point);
	map<string, pcl::PointXYZRGB> RestoreAllPoints(map<string, pcl::PointXYZRGB> m);
	void test();
	void CompareCloud(PointCloudT::Ptr cloud);

	void get3DPheno(PointCloudT::Ptr cloud, PointT dorsals, PointT tails, PointT heade);
	PointCloudT::Ptr gethalffish(PointCloudT::Ptr cloud, PointT heade, PointT tails);
	PointCloudT::Ptr getFullFishBySection(PointCloudT::Ptr cloud, PointT heade, PointT dorsals, PointT ventrale, PointT tails, PointT taile);
	void get3DPhenoByExcel(PointCloudT::Ptr cloud, string name);
	map<string, ThreeKeys> getTXT(string filename);
	void getheadPheno(PointCloudT::Ptr cloud, Vector4f abc);
	map<string,vector<PointT>> getKeypointbyTXT(string filename);
	vector<double> get3DPhenoByTXT(PointCloudT::Ptr cloud, string name);
	Vector4f getPlaneCoeff();//得到归一化平面的参数---用于求弧度的
	ArcPara getArc(PointCloudT::Ptr cloud, PointT heade);
	PointCloudT::Ptr getcaudalCloud(PointCloudT::Ptr cloud, PointT caudale);
	bool Overseg(PointCloudT::Ptr cloud);
	bool FacetSeg(PointCloudT::Ptr cloud,double normal_im, double color_im, double all_thresold);//小面分割
	bool kmeansSeg(PointCloudT::Ptr cloud, double normal_im, double color_im, double all_thresold);//小面分割
	PointCloudT::Ptr RegionGrowingSeg(PointCloudT::Ptr cloud);
	int CPCSeg(PointCloudT::Ptr cloud);
	bool RegionGrowingRGBSeg(PointCloudT::Ptr cloud, PointCloudT::Ptr &colored_cloud, vector<pcl::PointIndices> &clusters);
	void validatecloud(PointCloudT::Ptr auto_cloud, PointCloudT::Ptr manual_cloud);
	double getAngleFromDorsalAndPelvic(PointT dorsals, PointT pelvics);
	TriAngle getTriAngle(PointT pectorals, PointT dorsals, PointT pelvics);
	//void get3DphenoBatch(string folder);



	//获取表型
	double getfull_length();
	double getbody_length();
	double gethead_length();
	double getbody_height();
	double getcaudal_peduncle_height();//尾柄高
	double getcaudal_peduncle_length();//尾柄长
	double getdorsal_proboscis_dis();//背吻距
	double getproboscis_length();//吻长
	double geteye_diameter();//眼径
	double gethead_behind_eye_length();//眼后头长
	double getanal_pelvic_dis();//臀腹鳍基距
	double getdorsal_length();//背鳍基长
	double getpectoral_length();//胸鳍长
	double getpelvic_length();//腹鳍长
	double getanal_length();//臀鳍基长
	double gettail_length();//尾鳍长
	double getdorsal_tail_dis();//背鳍后距
	double getsurface_area();
	double getvolume();


private:
	vector<_xyz> V;//存顶点坐标
	vector<_xy> VT;//存纹理坐标
	vector<_normal> VN;//法向量
	vector<_trif> F;//面
	int TEXWIDTH;
	int TEXHEIGHT;
	MatrixXf avg;//中心点
	MatrixXf rot;//旋转矩阵
	Matrix4f reduction;
	vector<double> plane_coeff;
	vector<pcl::PointIndices> facetcluster;
	vector<pcl::PointIndices> facetclusterrgb;
	double minx;
	double maxx;
	PointT beiqi_s;
	PointT beiqi_e;
	PointT tunqi_s;
	PointT tunqi_e;
	PointT weiqi_s;
	PointT weiqi_e;
	PointT xqi_s;
	PointT xqi_e;
	PointT fuqi_s;
	PointT fuqi_e;
	PointT head_s;
	PointT head_e;
	PointT head_top;
	PointT weib_top;
	PointT weib_bottom;
	PointT bei_top;
	PointT fu_bottom;
	PointT eye_s;
	PointT eye_e;
	
	PointCloudT::Ptr cloud_origin;
	PointCloudT::Ptr beiqi_cloud;
	PointCloudT::Ptr tunqi_cloud;
	
	_BoundRect rect;

	bool isturnz;
	bool isturnx;
	bool ismirr;

	//需要计算的性状
	double full_length;
	double body_length;
	double head_length;
	double body_height;
	double caudal_peduncle_height;//尾柄高
	double caudal_peduncle_length;//尾柄长
	double dorsal_proboscis_dis;//背吻距
	double proboscis_length;//吻长
	double eye_diameter;//眼径
	double head_behind_eye_length;//眼后头长
	double anal_pelvic_dis;//臀腹鳍基距
	double dorsal_length;//背鳍基长
	double pectoral_length;//胸鳍长
	double pelvic_length;//腹鳍长
	double anal_length;//臀鳍基长
	double tail_length;//尾鳍长
	double dorsal_tail_dis;//背鳍后距
	double surface_area;
	double volume;

	string fishname;

	double head_volume;
	double all_volume;
	double head_height;
	double head_length_3d;
	double head_width;
	double fish_width;
	double mid_height;
	double tail_height;
};