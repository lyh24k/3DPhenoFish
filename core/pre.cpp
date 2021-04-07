#include "pre.h"

PreTreatment::PreTreatment(string fishname_)
	:cloud_origin(new PointCloudT),
	beiqi_cloud(new PointCloudT),
	tunqi_cloud(new PointCloudT)
{
	fishname = fishname_;
}

int PreTreatment::getfilesname()
{
	string folder_path = "*.obj";
	vector<String> file_names;
	//保存到当前目录下的pcd文件夹
	string folderpath = folder_path;
	string::size_type position;
	position = folderpath.find_last_of("\\");
	if (position != folderpath.npos)
	{
		folderpath = folderpath.erase(position + 1, -1) + "pcd";
	}
	if (_access(folderpath.c_str(), 0)==-1)
	{
		if (_mkdir(folderpath.c_str())==-1)
		{
			return FALSE;
		}
	}

	glob(folder_path, file_names);
	
	for (int i = 0; i < file_names.size(); i++)
	{
		string filename;
		filename = file_names[i];
		string pcdpath=filename;
		position = pcdpath.find_last_of("\\");
		pcdpath = pcdpath.erase(0, position);
		position = pcdpath.find_last_of(".");
		pcdpath.replace(position+1, 3, "pcd");
		pcdpath = folderpath + pcdpath;
		//cout << filename << endl;
		//cout << pcdpath << endl;
		PointCloudT::Ptr objcloud(new PointCloudT);
		objcloud = Obj2Pcd(filename);
		PointCloudT::Ptr cloud_simple(new PointCloudT);
		cloud_simple = DownSample(objcloud);
		if (!cloud_simple->empty())
		{
			pcl::io::savePCDFileASCII(pcdpath, *cloud_simple);
		}
		else
		{
			cout << "pcd empty" << endl;
			continue;
		}
		cout << pcdpath << " success" << endl;
		
	}
	cout << "finish!!!" << endl;
	return 1;
}

PointCloudT::Ptr PreTreatment::Obj2Pcd(string filename)
{
	PointCloudT::Ptr objcloud(new PointCloudT);
	try
	{
		clock_t start, endt;
		start = clock();
		ifstream objfile(filename);
		if (!objfile.is_open())
		{
			cout << "open objfile error!" << endl;
			return FALSE;
		}
		string line;
		_xyz v;
		_xy vt;
		_normal vn;
		_trif f;
		Mat textureImg;
		map<int, int> vmap;//保存顶点和UV对应的索引
						   //读取obj，存V VT VN F
		while (getline(objfile, line))
		{
			if (line.length() < 2)
				continue;
			istringstream ss(line);
			string type;
			ss >> type;
			if (type == "#")
			{
				continue;
			}
			else if (type == "v")
			{
				ss >> v.x >> v.y >> v.z;
				V.push_back(v);
			}
			else if (type == "vt")
			{
				ss >> vt.x >> vt.y;
				VT.push_back(vt);
			}
			else if (type == "vn")
			{
				ss >> vn.nx >> vn.ny >> vn.nz;
				VN.push_back(vn);
			}
			else if (type == "mtllib")
			{
				string mtlname;
				string folderpath = filename;
				ss >> mtlname;
				string::size_type position;
				position = mtlname.find_first_of(".");
				position = mtlname.find(".");
				if (position == 0)
				{
					mtlname = mtlname.erase(position, 2);
				}
				position = folderpath.find_last_of("\\");
				if (position != folderpath.npos)
				{
					mtlname = folderpath.erase(position + 1, -1) + mtlname;
				}
				//mtlname= "input/obj/" + mtlname;
				ifstream mtlfile(mtlname);
				if (!mtlfile.is_open())
				{
					cout << "open mtlfile error!" << endl;
					return FALSE;
				}
string mtlline;
string texturename;
while (getline(mtlfile, mtlline))
{
	istringstream mtlss(mtlline);
	string map_kd;
	mtlss >> map_kd;
	if (map_kd == "map_Kd")
	{
		mtlss >> texturename;
		break;
	}
}
mtlfile.close();
texturename = folderpath + texturename;

//texturename = filename.replace(filename.find("."), 4, texturename);
textureImg = imread(texturename, 1);
if (textureImg.data == NULL)
{
	cout << "can't find texture file" << endl;
	return FALSE;
}
TEXWIDTH = textureImg.cols;
TEXHEIGHT = textureImg.rows;
			}
			else if (type == "f")
			{
				string s1, s2, s3;
				ss >> s1 >> s2 >> s3;
				for (int i = 0; i < s1.size(); i++)
				{
					if (s1[i] == '/')
					{
						s1[i] = ' ';
					}
				}
				istringstream temp1(s1);
				temp1 >> f.v[0] >> f.t[0] >> f.n[0];
				for (int i = 0; i < s2.size(); i++)
				{
					if (s2[i] == '/')
					{
						s2[i] = ' ';
					}
				}
				istringstream temp2(s2);
				temp2 >> f.v[1] >> f.t[1] >> f.n[1];
				for (int i = 0; i < s3.size(); i++)
				{
					if (s3[i] == '/')
					{
						s3[i] = ' ';
					}
				}
				istringstream temp3(s3);
				temp3 >> f.v[2] >> f.t[2] >> f.n[2];
				for (int i = 0; i < 3; i++)
				{
					vmap[f.v[i]] = f.t[i];
				}
				F.push_back(f);

			}
			else
			{
				continue;
			}
		}
		objfile.close();

		//转pcd
		
		objcloud->resize(V.size());
		for (int i = 0; i < V.size(); i++)
		{
			objcloud->points[i].x = V[i].x;
			objcloud->points[i].y = V[i].y;
			objcloud->points[i].z = V[i].z;
			//计算颜色
			int index = vmap[i + 1];//UV的索引
			if (index > 0 && index < VT.size()) {
				int x = VT[index - 1].x*TEXWIDTH;
				int y = TEXHEIGHT - VT[index - 1].y*TEXHEIGHT;
				if (x >= TEXWIDTH || y >= TEXHEIGHT || x < 0 || y < 0)
				{
					cout << "out of img" << endl;
					return FALSE;
				}
				objcloud->points[i].r = textureImg.at<Vec3b>(y, x)[2];
				objcloud->points[i].g = textureImg.at<Vec3b>(y, x)[1];
				objcloud->points[i].b = textureImg.at<Vec3b>(y, x)[0];
			}

		}
		endt = clock();
		cout << "success: " << (double)(endt - start) / CLOCKS_PER_SEC << endl;
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return objcloud;
}

PointCloudT::Ptr PreTreatment::DownSample(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_simple(new PointCloudT());
	try
	{
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(1, 1, 1);
		sor.filter(*cloud_simple);
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_simple;
}

PointCloudT::Ptr PreTreatment::RemovePlane(PointCloudT::Ptr cloud)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
	try {
		//ransac挑出平面
		pcl::SACSegmentation<PointT> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
		
		pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.8);
		seg.setMaxIterations(1000);
		seg.setInputCloud(cloud->makeShared());
		seg.segment(*inliers, *coefficients);

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);        //提取内点的索引并存储在其中
		extract.setNegative(true);
		extract.filter(*cloud_filtered);

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_filtered;
	//分割出多个平面
	/*pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.8);
	seg.setMaxIterations(1000);
	//seg.setProbability(0.3);
	pcl::copyPointCloud(*cloud, *cloud_filtered);
	//cloud_filtered = cloud;
	int nr_points = (int)cloud_filtered->points.size();//剩余点云的数量
	cout << "while pointcloud " << cloud->points.size() << endl;
	cout << "while pointcloud " << cloud_filtered->points.size() << endl;
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) 
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);       
		extract.setNegative(false);


		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}
	return cloud_filtered;*/
}

PointCloudT::Ptr PreTreatment::PreProcess(PointCloudT::Ptr cloud)
{
	try
	{
		//降采样
		pcl::PointCloud<PointT>::Ptr cloud_downSample(new pcl::PointCloud<PointT>());
		cloud_downSample = DownSample(cloud);
		PointCloudT::Ptr cloud_fishonly(new PointCloudT());
		cloud_fishonly = getfishonly(cloud_downSample);

		cloud_origin = trasformfishaxis(cloud_fishonly);
		
		rect = getBoundRect(cloud_origin);
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_origin;
}

int PreTreatment::Allin()
{
	try
	{
		//区域生长
		pcl::PointCloud<PointT>::Ptr cloud_regionSeg(new pcl::PointCloud<PointT>());
		cloud_regionSeg = Removebody(cloud_origin);
		pcl::PointCloud<PointT>::Ptr cloud_facet(new pcl::PointCloud<PointT>());
		if (!FacetSegmentation(cloud_regionSeg, cloud_facet, 0.7, 0.3, 20))
		{
			getfakekeypoints(cloud_origin);
		}
		else {

			getkeypoint(facetcluster, cloud_facet);//分割尾鳍、背鳍、臀鳍


			//分割鱼肚
			PointCloudT::Ptr cloud_belly(new PointCloudT());
			if (!getbelly(cloud_origin, cloud_belly))
			{
				getfakekeypoints(cloud_origin);
			}//分割胸鳍、腹鳍
			getfishbody(cloud_facet);//背最高点，腹最低点

			cout << "belly success" << endl;
			getheadpoints(cloud_origin);//分割头
			getfisheyes(cloud_origin);//eyes
			cout << "head" << endl;
			//找尾柄
			getTailHandlepoints(cloud_origin);
			cout << "tailhandle" << endl;
			//表面重建
			//PointCloudT::Ptr cloud_body(new PointCloudT());
			//cloud_body = getfishbodyformesh(cloud_origin);
			//PointCloudT::Ptr cloud_fullfishnotail(new PointCloudT());
			//cloud_fullfishnotail = getfullfish(cloud_body);
			//if (!cloud_fullfishnotail->empty())
			//{
			//	getVolume(cloud_fullfishnotail);
			//}
			//else
			PointCloudT::Ptr cloud_fullfish(new PointCloudT());
			cloud_fullfish = getfullfish(cloud_origin);
			//getSurfaceArea(cloud_fullfish);
			//获取表型
			getPhenotype();
			Drawkeypoints();//绘制关键点
			//test();
			InsertData();
			cout << "over" << endl;
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

map<string, pcl::PointXYZRGB> PreTreatment::extractKeyPoints()
{
	map<string, pcl::PointXYZRGB> m;
	try
	{
		pcl::PointXYZRGB beiqi1;
		pcl::PointXYZRGB beiqi2;

		pcl::PointXYZRGB tunqi1;
		pcl::PointXYZRGB tunqi2;

		pcl::PointXYZRGB weiqi1;
		pcl::PointXYZRGB weiqi2;

		pcl::PointXYZRGB xqi1;
		pcl::PointXYZRGB xqi2;

		pcl::PointXYZRGB fuqi1;
		pcl::PointXYZRGB fuqi2;

		pcl::PointXYZRGB head1;
		pcl::PointXYZRGB head2;
		pcl::PointXYZRGB head3;

		pcl::PointXYZRGB weib1;
		pcl::PointXYZRGB weib2;

		pcl::PointXYZRGB bei1;
		pcl::PointXYZRGB fu2;

		pcl::PointXYZRGB eye1;
		pcl::PointXYZRGB eye2;

		RGBA2RGB(beiqi_s, beiqi1);
		RGBA2RGB(beiqi_e, beiqi2);

		RGBA2RGB(tunqi_s, tunqi1);
		RGBA2RGB(tunqi_e, tunqi2);

		RGBA2RGB(weiqi_s, weiqi1);
		RGBA2RGB(weiqi_e, weiqi2);

		RGBA2RGB(xqi_s, xqi1);
		RGBA2RGB(xqi_e, xqi2);

		RGBA2RGB(fuqi_s, fuqi1);
		RGBA2RGB(fuqi_e, fuqi2);

		RGBA2RGB(head_s, head1);
		RGBA2RGB(head_e, head2);
		RGBA2RGB(head_top, head3);

		RGBA2RGB(weib_top, weib1);
		RGBA2RGB(weib_bottom, weib2);

		RGBA2RGB(bei_top, bei1);
		RGBA2RGB(fu_bottom, fu2);

		RGBA2RGB(eye_s, eye1);
		RGBA2RGB(eye_e, eye2);
		
		m.insert(pair<string, pcl::PointXYZRGB>("背鳍起点", beiqi1));
		m.insert(pair<string, pcl::PointXYZRGB>("背鳍基部后端", beiqi2));
		m.insert(pair<string, pcl::PointXYZRGB>("臀鳍起点", tunqi1));
		m.insert(pair<string, pcl::PointXYZRGB>("臀鳍基部后端", tunqi2));
		m.insert(pair<string, pcl::PointXYZRGB>("尾鳍起点", weiqi1));
		m.insert(pair<string, pcl::PointXYZRGB>("尾鳍末端", weiqi2));
		m.insert(pair<string, pcl::PointXYZRGB>("胸鳍起点", xqi1));
		m.insert(pair<string, pcl::PointXYZRGB>("胸鳍基部末端", xqi2));
		m.insert(pair<string, pcl::PointXYZRGB>("腹鳍起点", fuqi1));
		m.insert(pair<string, pcl::PointXYZRGB>("腹鳍基部末端", fuqi2));
		m.insert(pair<string, pcl::PointXYZRGB>("吻端", head1));
		m.insert(pair<string, pcl::PointXYZRGB>("鳃盖骨后缘", head2));
		m.insert(pair<string, pcl::PointXYZRGB>("头部最高点", head3));
		m.insert(pair<string, pcl::PointXYZRGB>("尾柄上", weib1));
		m.insert(pair<string, pcl::PointXYZRGB>("尾柄下", weib2));
		m.insert(pair<string, pcl::PointXYZRGB>("背缘最高点", bei1));
		m.insert(pair<string, pcl::PointXYZRGB>("腹缘最低点", fu2));
		m.insert(pair<string, pcl::PointXYZRGB>("眼前缘", eye1));
		m.insert(pair<string, pcl::PointXYZRGB>("眼后缘", eye2));
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return m;
}

void PreTreatment::RGBA2RGB(PointT &rgba, pcl::PointXYZRGB &rgb)
{
	rgb.x = rgba.x;
	rgb.y = rgba.y;
	rgb.z = rgba.z;
	rgb.r = rgba.r;
	rgb.g = rgba.g;
	rgb.b = rgba.b;
}

int PreTreatment::Allin(string filename)
{
	cout << "All in finish" << endl;
	return 1;
}

PointCloudT::Ptr PreTreatment::PointCloudScale(double k, PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr scalePointCloud(new PointCloudT);
	try
	{
		
		scalePointCloud->resize(cloud->points.size());
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			scalePointCloud->points[i].x = k*cloud->points[i].x;
			scalePointCloud->points[i].y = k*cloud->points[i].y;
			scalePointCloud->points[i].z = k*cloud->points[i].z;
			scalePointCloud->points[i].r = cloud->points[i].r;
			scalePointCloud->points[i].g = cloud->points[i].g;
			scalePointCloud->points[i].b = cloud->points[i].b;
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return scalePointCloud;
}

PointCloudT::Ptr PreTreatment::Removebody(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr indices_cloud(new PointCloudT);
	try
	{
		//区域生长分割确定背、臀、尾
		//PointCloudT::Ptr smallcloud(new PointCloudT);
		//smallcloud = PointCloudScale(0.01, cloud);
		pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setRadiusSearch(0.015);
		//normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
		normal_estimator.compute(*normals);
		pcl::IndicesPtr indices(new std::vector <int>);
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		//pass.setFilterLimits(-500, -300);//保留或过滤z轴方向
		pass.filter(*indices);
		pcl::RegionGrowing<PointT, pcl::Normal> reg;
		reg.setMinClusterSize(30);
		reg.setMaxClusterSize(1000000);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30);//1.30 2.20 3.30 4.20 5.20 6.40
		reg.setInputCloud(cloud);
		reg.setIndices(indices);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold(1.0);
		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);
		pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloudRGBA();
		//colored_cloud = PointCloudScale(100, colored_cloud);
		
		vector<bool> flag(cloud->points.size(), 0);
		for (auto i = 0; i < clusters.size(); i++)
		{
			for (auto j = 0; j < clusters[i].indices.size(); j++)
			{
				//PointT temp;
				//temp = smallcloud->points[clusters[i].indices[j]];
				flag[clusters[i].indices[j]] = true;
				//indices_cloud->points.push_back(temp);
			}
		}
		for (auto i = 0; i < cloud->points.size(); i++)
		{
			if (!flag[i]&&cloud->points[i].x>minx+full_length/6)
			{
				indices_cloud->points.push_back(cloud->points[i]);
			}
		}
		indices_cloud->resize(indices_cloud->points.size());
		//indices_cloud = PointCloudScale(100, indices_cloud);
		//cout << colored_cloud->points.size() << endl;
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return indices_cloud;
}

PointCloudT::Ptr PreTreatment::RegionGrowingSeg(PointCloudT::Ptr cloud)
{
	clock_t start, end;
	start = clock();
	pcl::PointCloud <PointT>::Ptr colored_cloud;
	try
	{
		//区域生长分割确定背、臀、尾
		PointCloudT::Ptr smallcloud(new PointCloudT);
		smallcloud = PointCloudScale(0.01, cloud);
		pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(smallcloud);
		//normal_estimator.setRadiusSearch(0.015);
		normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
		normal_estimator.compute(*normals);
		pcl::IndicesPtr indices(new std::vector <int>);
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(smallcloud);
		pass.setFilterFieldName("z");
		//pass.setFilterLimits(-500, -300);//保留或过滤z轴方向
		pass.filter(*indices);
		pcl::RegionGrowing<PointT, pcl::Normal> reg;
		reg.setMinClusterSize(30);
		reg.setMaxClusterSize(1000000);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30);//1.30 2.20 3.30 4.20 5.20 6.40
		reg.setInputCloud(smallcloud);
		reg.setIndices(indices);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold(1.0);
		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);
		
		colored_cloud = reg.getColoredCloudRGBA();
		colored_cloud = PointCloudScale(100, colored_cloud);
		end = clock();		//程序结束用时
		double endtime = (double)(end - start) / CLOCKS_PER_SEC;
		cout << "Total time:" << endtime * 1000 << "ms" << endl;	//ms为单位
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return colored_cloud;
}

PointCloudT::Ptr PreTreatment::FilterZ(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	try
	{
		
		pcl::ProgressiveMorphologicalFilter<PointT> pmf;
		pcl::PointIndicesPtr ground(new pcl::PointIndices);
		int max_w_s(20);
		float slope(3.0f);
		float initial_d(0.5f);
		float max_d(2.0f);
		pmf.setInputCloud(cloud);
		pmf.setMaxWindowSize(max_w_s);
		pmf.setSlope(slope);
		pmf.setInitialDistance(initial_d);
		pmf.setMaxDistance(max_d);
		pmf.extract(ground->indices);

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ground);
		extract.filter(*cloud_filtered);

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_filtered;
}

PointCloudT::Ptr PreTreatment::EuclideanSeg(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_EuSeg(new PointCloudT);
	try
	{
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		tree->setInputCloud(cloud);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointT> ec;   //欧式聚类对象
		ec.setClusterTolerance(1.5f);                     // 设置近邻搜索的搜索半径为1.5cm
		ec.setMinClusterSize(10);                 //设置一个聚类需要的最少的点数目为100
		ec.setMaxClusterSize(1000000);               //设置一个聚类需要的最大点数目为25000
		ec.setSearchMethod(tree);                    //设置点云的搜索机制
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
		
		std::vector<pcl::PointIndices>::const_iterator it;
		std::vector<pcl::PointIndices>::const_iterator temp;
		size_t maxpoints = 0;
		for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			if (it->indices.size() > maxpoints)
			{
				maxpoints = it->indices.size();
				temp = it;
			}
		}
		//cloud_EuSeg->resize(cloud_EuSeg->points.size());
		PointT pointEuc;
		for (int i = 0; i < temp->indices.size(); ++i)
		{
			pointEuc = cloud->points[temp->indices[i]];
			cloud_EuSeg->points.push_back(pointEuc);
		}
		cloud_EuSeg->resize(cloud_EuSeg->points.size());
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_EuSeg;
}

PointCloudT::Ptr PreTreatment::getColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_, std::vector <pcl::PointIndices> clusters_, float r, float g, float b)
{
	PointCloudT::Ptr colored_cloud;
	try
	{
		

		if (!clusters_.empty())
		{
			colored_cloud = (new PointCloudT)->makeShared();

			srand(static_cast<unsigned int> (time(0)));
			std::vector<unsigned char> colors;
			for (size_t i_segment = 0; i_segment < clusters_.size(); i_segment++)
			{
				colors.push_back(static_cast<unsigned char> (rand() % 256));
				colors.push_back(static_cast<unsigned char> (rand() % 256));
				colors.push_back(static_cast<unsigned char> (rand() % 256));
			}

			colored_cloud->width = input_->width;
			colored_cloud->height = input_->height;
			colored_cloud->is_dense = input_->is_dense;
			for (size_t i_point = 0; i_point < input_->points.size(); i_point++)
			{
				PointT point;
				point.x = *(input_->points[i_point].data);
				point.y = *(input_->points[i_point].data + 1);
				point.z = *(input_->points[i_point].data + 2);
				point.r = r;
				point.g = g;
				point.b = b;
				colored_cloud->points.push_back(point);
			}

			std::vector< pcl::PointIndices >::iterator i_segment;
			int next_color = 0;
			for (i_segment = clusters_.begin(); i_segment != clusters_.end(); i_segment++)
			{
				std::vector<int>::iterator i_point;
				for (i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
				{
					int index;
					index = *i_point;
					colored_cloud->points[index].r = colors[3 * next_color];
					colored_cloud->points[index].g = colors[3 * next_color + 1];
					colored_cloud->points[index].b = colors[3 * next_color + 2];
				}
				next_color++;
			}
		}

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return colored_cloud;
}

PointCloudT::Ptr PreTreatment::OverSeg(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr coloredcloud(new PointCloudT);
	try
	{
		PointCloudT::Ptr smallcloud(new PointCloudT);
		smallcloud = PointCloudScale(1, cloud);
		float smoothness_threshold = 0.1;
		float voxel_resolution = 0.5f;
		float seed_resolution = 5.0f;
		float color_importance = 0.0f;
		float spatial_importance = 1.0f;
		float normal_importance = 4.0f;
		cout << smallcloud->points.size() << endl;
		//生成结晶器
		pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
		super.setUseSingleCameraTransform(false);
		//输入点云和设置参数
		super.setInputCloud(smallcloud);
		super.setColorImportance(color_importance);
		super.setSpatialImportance(spatial_importance);
		super.setNormalImportance(normal_importance);
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
		super.extract(supervoxel_clusters);
		cout << "super " << supervoxel_clusters.size() << endl;
		std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
		super.getSupervoxelAdjacency(supervoxel_adjacency);
		pcl::PointCloud<PointTL>::Ptr supervoxel_centroid_cloud = super.getLabeledCloud();
		
		coloredcloud = Label2Color(supervoxel_centroid_cloud);
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return coloredcloud;
}

PointCloudT::Ptr PreTreatment::Label2Color(pcl::PointCloud<PointTL>::Ptr cloud)
{
	PointCloudT::Ptr colorvoxelcloud(new PointCloudT);
	try
	{
		int labelcount = 0;
		std::map<int, RGB> cloudrgb;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloudrgb.count(cloud->points[i].label) > 0)
			{
				continue;
			}
			else
			{
				RGB rgb;
				rgb.r = Random(255);
				rgb.g = Random(255);
				rgb.b = Random(255);
				cloudrgb[cloud->points[i].label] = rgb;
			}
			if (cloud->points[i].label > labelcount)
			{
				labelcount = cloud->points[i].label;
			}
		}
		
		colorvoxelcloud->resize(cloud->points.size());
		for (int i = 0; i < cloud->points.size(); i++)
		{
			map<int, RGB>::iterator it;
			it = cloudrgb.find(cloud->points[i].label);
			if (it != cloudrgb.end())
			{
				colorvoxelcloud->points[i].x = cloud->points[i].x;
				colorvoxelcloud->points[i].y = cloud->points[i].y;
				colorvoxelcloud->points[i].z = cloud->points[i].z;
				colorvoxelcloud->points[i].r = it->second.r;
				colorvoxelcloud->points[i].g = it->second.g;
				colorvoxelcloud->points[i].b = it->second.b;

			}
			else
			{
				cout << "can't find color" << endl;
				continue;
			}

		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return colorvoxelcloud;
}

PointCloudT::Ptr PreTreatment::getBoundary(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr boundarycloud(new PointCloudT);
	try
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Boundary> boundaries;
		pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> est;
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		pcl::NormalEstimation<PointT, pcl::Normal> normEst;
		normEst.setInputCloud(cloud);
		normEst.setSearchMethod(tree);
		normEst.setKSearch(9);  //法向估计的点数
		normEst.compute(*normals);

		est.setInputCloud(cloud);
		est.setInputNormals(normals);
		est.setSearchMethod(tree);
		est.setKSearch(20);  //一般这里的数值越高，最终边界识别的精度越好
		est.compute(boundaries);

		
		PointCloudT::Ptr noboundarycloud(new PointCloudT);
		for (int i = 0; i < cloud->size(); i++) {
			uint8_t x = (boundaries.points[i].boundary_point);
			int a = static_cast<int>(x); //该函数的功能是强制类型转换
			if (a == 1)
			{
				//  boundPoints.push_back(cloud->points[i]);
				boundarycloud->points.push_back(cloud->points[i]);
			}
			else
				noboundarycloud->points.push_back(cloud->points[i]);

		}
		boundarycloud->resize(boundarycloud->points.size());
		noboundarycloud->resize(noboundarycloud->points.size());
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return boundarycloud;
}

PointCloudT::Ptr PreTreatment::getPlane(PointCloudT::Ptr cloud)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
	try
	{
		//ransac挑出平面
		pcl::SACSegmentation<PointT> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
		
		pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.8);
		seg.setMaxIterations(1000);
		seg.setInputCloud(cloud->makeShared());
		seg.segment(*inliers, *coefficients);

		for (int i = 0; i < 4; i++)
		{
			plane_coeff.push_back(coefficients->values[i]);
			//cout << plane_coeff[i] << endl;
		}

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);        //提取内点的索引并存储在其中
		extract.setNegative(false);
		extract.filter(*cloud_filtered);

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_filtered;
}

PointCloudT::Ptr PreTreatment::BoundaryGrowth(PointCloudT::Ptr plane, PointCloudT::Ptr boundary)
{
	pcl::PointCloud<PointT>::Ptr cloud_newtail(new pcl::PointCloud<PointT>());
	try
	{
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
		for (int i = 0; i < boundary->points.size(); ++i)
		{
			cloud->points.push_back(boundary->points[i]);
			cloud_newtail->points.push_back(boundary->points[i]);
		}
		for (int i = 0; i < plane->points.size(); ++i)
		{
			cloud->points.push_back(plane->points[i]);
		}
		cloud->resize(cloud->points.size());
		//cout << plane->points.size() << " +" << boundary->points.size() << endl;
		//cout << cloud->points.size() << endl;
		pcl::KdTreeFLANN<PointT> tree;
		tree.setInputCloud(cloud);

		pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
		kdtree->setInputCloud(cloud);
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(kdtree);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setRadiusSearch(3.0);
		normal_estimator.compute(*normals);

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		double r = 3.0;
		
		vector<bool> flag;
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			flag.push_back(true);
		}
		//cout << flag.size() << endl;
		queue<int> growth_que;
		for (int i = 0; i < boundary->points.size(); ++i)
		{
			growth_que.push(i);
			flag[i] = false;
		}
		while (!growth_que.empty())
		{
			int index = growth_que.front();
			int graytail = (cloud->points[index].r * 30 + cloud->points[index].g * 59 + cloud->points[index].b * 11 + 50) / 100;
			if (tree.radiusSearch(cloud->points[index], r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				for (int n = 0; n < pointIdxRadiusSearch.size(); ++n)
				{
					int graynewtail = (cloud->points[pointIdxRadiusSearch[n]].r * 30 + cloud->points[pointIdxRadiusSearch[n]].g * 59 + cloud->points[pointIdxRadiusSearch[n]].b * 11 + 50) / 100;
					double normal_diff = acos(abs((normals->points[index].normal_x*normals->points[pointIdxRadiusSearch[n]].normal_x +
						normals->points[index].normal_y*normals->points[pointIdxRadiusSearch[n]].normal_y +
						normals->points[index].normal_z*normals->points[pointIdxRadiusSearch[n]].normal_z) /
						(sqrt(pow(normals->points[index].normal_x, 2) + pow(normals->points[index].normal_y, 2) + pow(normals->points[index].normal_z, 2))
							* sqrt(pow(normals->points[pointIdxRadiusSearch[n]].normal_x, 2) + pow(normals->points[pointIdxRadiusSearch[n]].normal_y, 2) + pow(normals->points[pointIdxRadiusSearch[n]].normal_z, 2))))) * 180 / 3.1415926;

					if (abs(graytail - graynewtail) < 10 && flag[pointIdxRadiusSearch[n]] && normal_diff < 1)
					{
						cloud_newtail->push_back(cloud->points[pointIdxRadiusSearch[n]]);
						growth_que.push(pointIdxRadiusSearch[n]);
						flag[pointIdxRadiusSearch[n]] = false;
					}
				}
			}
			growth_que.pop();
		}
		/*for (int i = 0; i < boundary->points.size(); ++i)
		{
			int graytail = (boundary->points[i].r * 30 + boundary->points[i].g * 59 + boundary->points[i].b * 11 + 50) / 100;
			if (tree.radiusSearch(boundary->points[i], r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				for (int n = 0; n < pointIdxRadiusSearch.size(); ++n)
				{
					int graynewtail= (cloud->points[pointIdxRadiusSearch[n]].r * 30 + cloud->points[pointIdxRadiusSearch[n]].g * 59 + cloud->points[pointIdxRadiusSearch[n]].b * 11 + 50) / 100;
					if (abs(graytail - graynewtail) < 10)
					{
						cloud_newtail->push_back(cloud->points[pointIdxRadiusSearch[n]]);
					}
				}
			}
		}*/
		cloud_newtail->resize(cloud_newtail->points.size());
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	if (cloud_newtail->points.size() != 0)
	{
		return cloud_newtail;
	}
	else
	{
		return FALSE;
	}
}

int PreTreatment::getpcdfiles(vector<string> &filepaths, vector<string> &filenames, string folder)
{
	try
	{
		boost::filesystem::path directory(folder);
		if (!boost::filesystem::exists(directory))
		{
			cerr << "目录不存在" << endl;
			return -1;
		}
		boost::filesystem::recursive_directory_iterator iter(directory), end_iter;
		for (; iter != end_iter; iter++)
		{
			if (boost::filesystem::is_regular_file(*iter) && (iter->path().extension().string() == ".pcd"))
			{
				filepaths.push_back(iter->path().string());
				filenames.push_back(iter->path().filename().string());
				//cout << iter->path().string() << " is file" << endl;
			}
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

PointCloudT::Ptr PreTreatment::getfishonly(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_fishonly(new PointCloudT());
	try
	{
		//去除平面
		pcl::PointCloud<PointT>::Ptr cloud_removePlane(new pcl::PointCloud<PointT>());
		cloud_removePlane = RemovePlane(cloud);

		//欧式分割
		pcl::PointCloud<PointT>::Ptr cloud_Euc(new pcl::PointCloud<PointT>());
		cloud_Euc = EuclideanSeg(cloud_removePlane);

		//get boundary
		pcl::PointCloud<PointT>::Ptr cloud_Plane(new pcl::PointCloud<PointT>());
		cloud_Plane = getPlane(cloud);

		pcl::PointCloud<PointT>::Ptr cloud_Boundary(new pcl::PointCloud<PointT>());
		cloud_Boundary = getBoundary(cloud_Euc);

		pcl::PointCloud<PointT>::Ptr cloud_newtail(new pcl::PointCloud<PointT>());
		cloud_newtail = BoundaryGrowth(cloud_Plane, cloud_Boundary);

		
		for (int i = 0; i < cloud_Euc->points.size(); i++)
		{
			cloud_fishonly->points.push_back(cloud_Euc->points[i]);
		}

		pcl::KdTreeFLANN<PointT> tree;
		tree.setInputCloud(cloud_Euc);
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		double r = 1.0;
		if (!cloud_newtail->empty())
		{
			for (int i = 0; i < cloud_newtail->points.size(); i++)
			{
				if (tree.radiusSearch(cloud_newtail->points[i], r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
				{
					int flag = 0;
					int n = 0;
					for (; n < pointIdxRadiusSearch.size(); ++n)
					{
						if (pointRadiusSquaredDistance[n] < 0.01)
						{
							flag = 1;
							break;
						}
					}
					if (flag == 0)
					{
						cloud_fishonly->points.push_back(cloud_newtail->points[i]);
					}
				}
				else
				{
					cloud_fishonly->points.push_back(cloud_newtail->points[i]);
				}
			}
			cloud_fishonly->resize(cloud_fishonly->points.size());
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_fishonly;
}

int PreTreatment::pcdBatch(string folder)
{
		vector<string> filepaths;
		vector<string> filenames;

		boost::filesystem::path inputpath(folder);
		string outputfolder = inputpath.parent_path().string() + "\\output";
		boost::filesystem::path outputpath(outputfolder);
		if (!boost::filesystem::exists(outputpath))
		{
			boost::filesystem::create_directory(outputpath);
		}
		getpcdfiles(filepaths, filenames, folder);
		try
		{
			if (filepaths.size() <= 0)
			{
				string s = "文件夹下没有pcd文件！请重新选择路径！";
				throw s;
			}
			else
			{
				for (int i = 0; i < filepaths.size(); i++)
				{
					PointCloudT::Ptr rgbcloud(new PointCloudT);
					PointCloudT::Ptr resultcloud(new PointCloudT);
					if (pcl::io::loadPCDFile<PointT>(filepaths[i], *rgbcloud) == -1) {
						cout << filepaths[i];
						PCL_ERROR(" Couldnot open file.\n");
						continue;
					}
					resultcloud = PreProcess(rgbcloud);
					if (resultcloud->points.size() == 0)
					{
						cerr << "分割失败" << endl;
						continue;
					}
					string outputname = outputfolder + "\\" + filenames[i];
					if (pcl::io::savePCDFileASCII(outputname, *resultcloud) == 0)
					{
						cout << outputname << " success" << endl;
						cout << resultcloud->points.size() << endl;
					}
					else
					{
						cout << outputname << "生成失败" << endl;
						continue;
					}
				}
				return 1;
			}
		}
		catch (string msg)
		{
			cerr << msg << endl;
			return -1;
		}
}

int PreTreatment::FindPrincipalDir(PointCloudT::Ptr cloud)
{
	try
	{
		MatrixXf xyz(cloud->points.size(), 3);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			xyz(i, 0) = cloud->points[i].x;
			xyz(i, 1) = cloud->points[i].y;
			xyz(i, 2) = cloud->points[i].z;
		}
		avg = xyz.colwise().mean();
		//cout <<"avg " <<avg << endl;
		MatrixXf adjust(cloud->points.size(), 3);
		for (int i = 0; i < adjust.rows(); i++) {
			adjust(i, 0) = xyz(i, 0) - avg(0, 0);
			adjust(i, 1) = xyz(i, 1) - avg(0, 1);
			adjust(i, 2) = xyz(i, 2) - avg(0, 2);
		}
		MatrixXf cov = adjust.adjoint()*adjust;
		//cout << cov << endl;
		SelfAdjointEigenSolver<MatrixXf> eig(cov);
		MatrixXf vec, val;
		vec = eig.eigenvectors();
		val = eig.eigenvalues();
		rot = vec;
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 0;
}

bool PreTreatment::getfishheaddir(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr mincloud(new PointCloudT);
	PointCloudT::Ptr maxcloud(new PointCloudT);
	double mincloud_var;
	double maxcloud_var;
	try
	{
		minx = 0, maxx = 0;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloud->points[i].x < minx)
			{
				minx = cloud->points[i].x;
			}
			if (cloud->points[i].x > maxx)
			{
				maxx = cloud->points[i].x;
			}
		}
		//cout << maxx << endl;
		full_length = maxx - minx;
		double radius = (maxx - minx) / 6;
		
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloud->points[i].x < minx + radius)
			{
				mincloud->points.push_back(cloud->points[i]);
			}
			if (cloud->points[i].x > maxx - radius)
			{
				maxcloud->points.push_back(cloud->points[i]);
			}
		}
		mincloud->resize(mincloud->points.size());
		maxcloud->resize(maxcloud->points.size());

		vector<double> mincloud_z(mincloud->points.size());
		vector<double> maxcloud_z(maxcloud->points.size());
		for (int i = 0; i < mincloud->points.size(); i++)
		{
			mincloud_z[i] = mincloud->points[i].z;
		}
		for (int i = 0; i < maxcloud->points.size(); i++)
		{
			maxcloud_z[i] = maxcloud->points[i].z;
		}
		//mincloudz轴标准差
		double sum = accumulate(mincloud_z.begin(), mincloud_z.end(), 0);
		double mean = sum / mincloud_z.size();
		double varsum = 0.0;
		for (int i = 0; i < mincloud_z.size(); i++)
		{
			varsum += (mincloud_z[i] - mean)*(mincloud_z[i] - mean);
		}
		mincloud_var = sqrt(varsum / (mincloud_z.size() - 1));
		//maxcloudz轴标准差
		sum = accumulate(maxcloud_z.begin(), maxcloud_z.end(), 0);
		mean = sum / maxcloud_z.size();
		varsum = 0;
		for (int i = 0; i < maxcloud_z.size(); i++)
		{
			varsum += (maxcloud_z[i] - mean)*(maxcloud_z[i] - mean);
		}
		maxcloud_var = sqrt(varsum / (maxcloud_z.size() - 1));
		cout << maxcloud_var << ' ' << mincloud_var << endl;
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	if (maxcloud_var > mincloud_var)
	{
		
		return TRUE;
	}
	else
	{
		
		return FALSE;
	}
}

PointCloudT::Ptr PreTreatment::trasformfishaxis(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr transformcloud(new PointCloudT);
	try
	{
		FindPrincipalDir(cloud);
		PointCloudT::Ptr translatecloud(new PointCloudT);
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		reduction= Eigen::Matrix4f::Identity();
		transform(0, 3) = 0 - avg(0, 0);
		transform(1, 3) = 0 - avg(0, 1);
		transform(2, 3) = 0 - avg(0, 2);
		reduction(0, 3) = avg(0, 0);
		reduction(1, 3) = avg(0, 1);
		reduction(2, 3) = avg(0, 2);
		pcl::transformPointCloud(*cloud, *translatecloud, transform);

		Eigen::Matrix3f axismat;
		axismat << 0, 0, 1,
			0, 1, 0,
			1, 0, 0;
		Eigen::Matrix3f rotmat;

		rotmat = rot.transpose().inverse()*axismat;
		//cout << rotmat << endl;
		Eigen::Matrix3f tempmat;
		tempmat = rotmat.inverse();
		transformcloud->resize(translatecloud->points.size());
		for (int i = 0; i < transformcloud->points.size(); i++)
		{
			transformcloud->points[i].x = translatecloud->points[i].x*rotmat(0, 0) + translatecloud->points[i].y*rotmat(1, 0) + translatecloud->points[i].z*rotmat(2, 0);
			transformcloud->points[i].y = translatecloud->points[i].x*rotmat(0, 1) + translatecloud->points[i].y*rotmat(1, 1) + translatecloud->points[i].z*rotmat(2, 1);
			transformcloud->points[i].z = translatecloud->points[i].x*rotmat(0, 2) + translatecloud->points[i].y*rotmat(1, 2) + translatecloud->points[i].z*rotmat(2, 2);
			transformcloud->points[i].rgb = translatecloud->points[i].rgb;
		}
		//PointCloudT::Ptr transform_zcloud(new PointCloudT);

		ismirr = IsMirr(transformcloud);
		cout << ismirr << endl;
		Eigen::Matrix3f mirrmat;
		mirrmat << 1, 0, 0,
			0, 1, 0,
			0, 0, -1;
		if (ismirr)
		{
			for (int i = 0; i < transformcloud->points.size(); i++)
			{
				transformcloud->points[i].z = 0 - transformcloud->points[i].z;
			}
			Eigen::Matrix3f rmat;
			rmat = mirrmat.inverse()*tempmat;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					reduction(i, j) = rmat(i, j);
				}
			}
			cout << "mirror success" << endl;
		}
		isturnx = turnZaxis();
		isturnz = getfishheaddir(transformcloud);
		if (ismirr)
		{
			isturnx = !isturnx;
		}
		Eigen::Matrix3f xrotmat;
		Eigen::Matrix3f zrotmat;
		
		xrotmat << 1, 0, 0,
			0, -1, 0,
			0, 0, -1;
		zrotmat << -1, 0, 0,
			0, -1, 0,
			0, 0, 1;
		
		if (isturnx)
		{
			Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();
			transform_z.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*transformcloud, *transformcloud, transform_z);
			if (!isturnz)
			{
				Eigen::Matrix3f rmat;
				rmat = xrotmat.inverse()*tempmat;
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						reduction(i, j) = rmat(i, j);
					}
				}
			}
			cout << "transform_z success" << endl;
		}
		if (isturnz)
		{
			Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
			transform_x.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*transformcloud, *transformcloud, transform_x);
			
			if (!isturnx)
			{
				Eigen::Matrix3f rmat;
				rmat = zrotmat.inverse()*tempmat;
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						reduction(i, j) = rmat(i, j);
					}
				}
			}
			double temp;
			temp = minx;
			minx = -maxx;
			maxx = -temp;
			cout << "transform_x success" << endl;
		}
		if (isturnx&&isturnz)
		{
			Eigen::Matrix3f rmat;
			rmat = zrotmat.inverse()*xrotmat.inverse()*tempmat;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					reduction(i, j) = rmat(i, j);
				}
			}
		}
		if(!isturnx&&!isturnz)
		{
			Eigen::Matrix3f xrotmat;
			Eigen::Matrix3f rmat;
			xrotmat << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			rmat = xrotmat.inverse()*tempmat;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					reduction(i, j) = rmat(i, j);
				}
			}
			cout << "transform success" << endl;
		}

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return transformcloud;
}

bool PreTreatment::IsMirr(PointCloudT::Ptr cloud)
{
	bool isHeadRight;
	bool isZbottom;
	bool isDorsaltop;
	isHeadRight = getfishheaddir(cloud);
	isZbottom = turnZaxis();
	//根据颜色确定背鳍方向
	PointCloudT::Ptr bin_cloud(new PointCloudT);
	int topsize = 0, bottomsize = 0;
	int tempmaxx = 0 + full_length / 7;
	int tempminx = 0 - full_length / 7;
	int avgtop = 0, avgbottom = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x < tempmaxx && cloud->points[i].x > tempminx)
		{
			int gray = (cloud->points[i].r * 30 + cloud->points[i].g * 59 + cloud->points[i].b * 11 + 50) / 100;
			if (cloud->points[i].y >= rect.centerpoint.y)
			{
				avgtop += gray;
				++topsize;
			}
			else
			{
				avgbottom += gray;
				++bottomsize;
			}
		}
	}
	
	avgtop /= topsize;
	avgbottom /= bottomsize;
	//cout << avgtop << ' ' << avgbottom << endl;
	isDorsaltop = avgtop < avgbottom ? true : false;
	if (!isHeadRight)
	{
		if (isZbottom == isDorsaltop)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (isZbottom != isDorsaltop)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	return true;
}

bool PreTreatment::turnZaxis()
{
	double t;
	try
	{
		Vector3d p1;//中心点
		p1 << avg(0, 0), avg(0, 1), avg(0, 2);
		Vector3d p2;//平面随机一点
		p2 << 0, 0, -plane_coeff[3] / plane_coeff[2];
		Vector3d n;
		n << plane_coeff[0], plane_coeff[1], plane_coeff[2];
		Vector3d u;
		u << rot(0, 0), rot(1, 0), rot(2, 0);

		
		t = (p2 - p1).dot(n) / u.dot(n);
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	if (t >= 0)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

int PreTreatment::segmentheadandbody(PointCloudT::Ptr cloud, PointCloudT &headcloud, PointCloudT &bodycloud)
{
	try
	{
		double radius = minx + full_length / 5;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloud->points[i].x < radius)
			{
				headcloud.points.push_back(cloud->points[i]);
			}
			else
			{
				bodycloud.points.push_back(cloud->points[i]);
			}
		}
		headcloud.resize(headcloud.points.size());
		bodycloud.resize(bodycloud.points.size());
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

bool PreTreatment::Overseg(PointCloudT::Ptr cloud)
{
	if (cloud->empty())
	{
		cout << "facet inputcloud empty" << endl;
		return false;
	}
	//PointCloudT::Ptr smallcloud(new PointCloudT);
	//smallcloud = PointCloudScale(0.01, cloud);
	float voxel_resolution = 0.8f;
	float seed_resolution = 6.0f;
	float color_importance = 0.1f;
	float spatial_importance = 0.2f;
	float normal_importance = 0.7f;
	//生成结晶器
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(false);
	//输入点云和设置参数
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	super.extract(supervoxel_clusters);
	cout << "super " << supervoxel_clusters.size() << endl;
	if (supervoxel_clusters.size() == 0)
	{
		cout << "supervoxel error size=0" << endl;
		return false;
	}

	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;
	size_t voxelsize = 0;
	for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
	{
		voxelsize += it_clusters->second->voxels_->size();
	}

	pcl::PointCloud<PointT>::Ptr cloud_overseg(new pcl::PointCloud<PointT>());
	int oversegsize = 0;
	for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
	{
	oversegsize += it_clusters->second->voxels_->points.size();
	}
	cloud_overseg->resize(oversegsize);
	int nc = 0;
	for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
	{
	int color_R = Random(255);
	int color_G = Random(255);
	int color_B = Random(255);
	for (int i = 0; i < it_clusters->second->voxels_->points.size(); i++)
	{
	cloud_overseg->points[nc].x = it_clusters->second->voxels_->points[i].x;
	cloud_overseg->points[nc].y = it_clusters->second->voxels_->points[i].y;
	cloud_overseg->points[nc].z = it_clusters->second->voxels_->points[i].z;
	cloud_overseg->points[nc].r = color_R;
	cloud_overseg->points[nc].g = color_G;
	cloud_overseg->points[nc].b = color_B;
	nc++;
	}
	}
	cout << voxelsize << endl;
	cout << nc << " overseg success " << oversegsize << endl;
}

bool PreTreatment::FacetSegmentation(PointCloudT::Ptr cloud, PointCloudT::Ptr &facet_grow_cloud, double normal_im, double color_im, double all_thresold)
{
	try {
		facetcluster.clear();
		if (cloud->empty())
		{
			cout << "facet inputcloud empty" << endl;
			return false;
		}
		PointCloudT::Ptr smallcloud(new PointCloudT);
		smallcloud = PointCloudScale(0.01, cloud);
		//float smoothness_threshold = 0.1;
		float voxel_resolution = 0.008f;
		float seed_resolution = 0.06f;
		float color_importance = 0.1f;
		float spatial_importance = 0.4f;
		float normal_importance = 1.0f;
		//生成结晶器
		pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
		super.setUseSingleCameraTransform(false);
		//输入点云和设置参数
		super.setInputCloud(smallcloud);
		super.setColorImportance(color_importance);
		super.setSpatialImportance(spatial_importance);
		super.setNormalImportance(normal_importance);
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
		super.extract(supervoxel_clusters);
		cout << "super " << supervoxel_clusters.size() << endl;
		if (supervoxel_clusters.size() == 0)
		{
			cout << "supervoxel error size=0" << endl;
			return false;
		}
		std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
		super.getSupervoxelAdjacency(supervoxel_adjacency);

		//区域生长
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;

		size_t voxelsize = 0;
		for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
		{
			voxelsize += it_clusters->second->voxels_->size();
		}

		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_minindex = supervoxel_clusters.begin();
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_m = supervoxel_clusters.begin();
		size_t tempmax = 0;
		for (; it_m != supervoxel_clusters.end(); it_m++)
		{
			if (it_m->first > tempmax)
			{
				tempmax = it_m->first;
			}
		}
		cout << tempmax << ' ' << supervoxel_clusters.size() << endl;
		//facet_grow_cloud->resize(voxelsize);
		float normal_thresold = 15.0;//法线差异
		double gray_thresold = 20;
		//double all_thresold = 25;
		vector<int> unused_clusters(supervoxel_clusters.size()*5, 0);
		int pointcount = 0;
		int facecount = 0;
		int loopcount = 0;
		//double color_im = 0.3;
		//double normal_im = 0.7;
		//cout << supervoxel_clusters.size() << endl;
		//vector<pcl::PointIndices> facetcluster;//分割后的点云索引
		int facetIndex = 0;
		vector<int> segmen_num;//存每个类的点数
		vector<int> point_laber;//存每个点的标签
		point_laber.resize(smallcloud->points.size());
		deque<int> face_que;
		while (facecount < supervoxel_clusters.size() && pointcount < voxelsize)
		{
			loopcount++;
			if (loopcount > (2 * supervoxel_clusters.size()))
			{
				break;
			}
			//找曲率最小的点
			double mincur = -999;

			for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
			{
				auto it_cur = it_clusters->second->normals_->begin();
				double tempcur = 0;
				for (; it_cur != it_clusters->second->normals_->end(); it_cur++)
				{
					tempcur += it_cur->curvature;
				}
				tempcur /= it_clusters->second->normals_->size();
				if (it_clusters->first >= unused_clusters.size())
				{
					cout << "out of range" << endl;
					cout << it_clusters->first << ' ' << unused_clusters.size() << endl;
					cout << "its unused" << endl;
				}
				if (unused_clusters[it_clusters->first]==0 && tempcur > mincur)
				{
					mincur = tempcur;
					it_minindex = it_clusters;
				}
			}
			//cout << facecount << endl;
			//cout << mincur << endl;
			if (it_minindex->first >= unused_clusters.size())
			{
				cout << "out of range" << endl;
				cout << it_minindex->first << ' ' << unused_clusters.size() << endl;
				cout << "its unused" << endl;
			}
			if (unused_clusters[it_minindex->first]==0)
			{
				int onefacetpointcount = 0;
				int color_R = Random(255);
				int color_G = Random(255);
				int color_B = Random(255);

				face_que.push_back(it_minindex->first);
				unused_clusters[it_minindex->first] = 1;
				facecount++;
				//染色
				for (int i = 0; i < it_minindex->second->voxels_->points.size(); ++i)
				{
					PointT facetemp;
					facetemp.x = it_minindex->second->voxels_->points[i].x;
					facetemp.y = it_minindex->second->voxels_->points[i].y;
					facetemp.z = it_minindex->second->voxels_->points[i].z;
					facetemp.r = color_R;
					facetemp.g = color_G;
					facetemp.b = color_B;
					facet_grow_cloud->push_back(facetemp);
					if (pointcount >= point_laber.size())
					{
						cout << "out of range" << endl;
						cout << pointcount << ' ' << point_laber.size() << endl;
						cout << "its unused" << endl;
					}
					point_laber[pointcount] = facetIndex;
					//facetcluster[facetIndex].indices.push_back(pointcount);
					++pointcount;
					onefacetpointcount++;
				}
				//float seed_cur = CalFacetCurv(it_clusters->second,smallcloud);
				int index = it_minindex->first;
				Vector3d face_normal;

				face_normal << supervoxel_clusters[index]->normal_.normal_x, supervoxel_clusters[index]->normal_.normal_y, supervoxel_clusters[index]->normal_.normal_z;
				//cout << "normal:"<<face_normal << endl;
				face_normal.normalize();
				//cout << face_normal << endl;
				int gray_seed = 0;
				for (int i = 0; i < supervoxel_clusters[index]->voxels_->points.size(); i++)
				{
					gray_seed = gray_seed + (supervoxel_clusters[index]->voxels_->points[i].r * 0.39 + supervoxel_clusters[index]->voxels_->points[i].g * 0.5 + supervoxel_clusters[index]->voxels_->points[i].b * 0.11);
				}
				gray_seed = gray_seed / supervoxel_clusters[index]->voxels_->points.size();
				while (!face_que.empty())
				{
					index = face_que.front();
					multimap<uint32_t, uint32_t>::iterator it_adjacency;
					it_adjacency = supervoxel_adjacency.find(index);
					for (int j = 0; j < supervoxel_adjacency.count(index); ++j)
					{
						int n = it_adjacency->second;
						Vector3d n_normal;
						n_normal << supervoxel_clusters[n]->normal_.normal_x, supervoxel_clusters[n]->normal_.normal_y, supervoxel_clusters[n]->normal_.normal_z;
						n_normal.normalize();
						//float target_cur = CalFacetCurv(supervoxel_clusters[n], smallcloud);
						//计算法线差异
						//float d = abs(seed_cur - target_cur);
						float d = acosf(face_normal.dot(n_normal))*180.0/M_PI;
						//cosf(2.0 / 180.0*M_PI)
						//float d = acos((face_normal(0)*n_normal(0) + face_normal(1)*n_normal(1) + face_normal(2)*n_normal(2)) /
							//(sqrt(pow(face_normal(0), 2) + pow(face_normal(1), 2) + pow(face_normal(2), 2)) *sqrt(pow(n_normal(0), 2) + pow(n_normal(1), 2) + pow(n_normal(2), 2)))) * 180 / 3.1415926;
						//计算灰度值
						int gray = 0;
						for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); i++)
						{
							gray = gray + (supervoxel_clusters[n]->voxels_->points[i].r * 0.39 + supervoxel_clusters[n]->voxels_->points[i].g * 0.5 + supervoxel_clusters[n]->voxels_->points[i].b * 0.11);
						}
						gray = gray / supervoxel_clusters[n]->voxels_->points.size();
						
						//线性加权
						//d = d * 255 / 180;
						int g = std::abs(gray - gray_seed) * 180 / 255;
						//cout << g << endl;
						if (d < 2)
						{
							swap(normal_im, color_im);
						}
						double th = normal_im*d + color_im*g;
						//小于阈值并且这个面没有被归类过，入队
						if ((th < all_thresold) && unused_clusters[n]==0)
						{
							//face_normal = face_normal + n_normal;
							//face_normal.normalize();
							face_que.push_back(n);
							//染相同颜色
							for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); ++i)
							{
								PointT facetemp;
								facetemp.x = supervoxel_clusters[n]->voxels_->points[i].x;
								facetemp.y = supervoxel_clusters[n]->voxels_->points[i].y;
								facetemp.z = supervoxel_clusters[n]->voxels_->points[i].z;
								facetemp.r = color_R;
								facetemp.g = color_G;
								facetemp.b = color_B;
								facet_grow_cloud->push_back(facetemp);
								point_laber[pointcount] = facetIndex;
								//facetcluster[facetIndex].indices.push_back(pointcount);
								++pointcount;
								onefacetpointcount++;
							}
							unused_clusters[n] = 1;
							facecount++;
						}
						it_adjacency++;
					}
					//出队
					face_que.pop_front();
				}
				face_que.clear();
				segmen_num.push_back(onefacetpointcount);
				//cout << segmen_num.capacity() << endl;
				facetIndex++;
			}
		}
		cout << unused_clusters.size() << " segmen_num " << segmen_num.size() << endl;
		//cout << segmen_num.size() << ' ' << facetIndex << endl;
		facetcluster.resize(segmen_num.size());
		for (auto i = 0; i < segmen_num.size(); i++)
		{
			facetcluster[i].indices.resize(segmen_num[i]);
		}
		vector<int> counter_2;
		counter_2.resize(segmen_num.size(), 0);
		for (auto i = 0; i < facet_grow_cloud->points.size(); i++)
		{
			int seg_idx = point_laber[i];
			int nebor_idx = counter_2[seg_idx];
			facetcluster[seg_idx].indices[nebor_idx] = i;
			counter_2[seg_idx] += 1;
		}
		if (facet_grow_cloud->empty())
		{
			cout << "facet success wrong" << endl;
			return false;
		}
		facet_grow_cloud = PointCloudScale(100, facet_grow_cloud);
		cout << "facet success" << endl;
		
	}
	catch (exception &e)
	{
		cout << e.what() << endl;
		return false;
	}
	return true;
}

bool PreTreatment::FacetSeg(PointCloudT::Ptr cloud, double normal_im, double color_im, double all_thresold)
{
	clock_t starttime, endtime;
	starttime = clock();
	try {
		PointCloudT::Ptr facet_grow_cloud(new PointCloudT);
		facetcluster.clear();
		if (cloud->empty())
		{
			cout << "facet inputcloud empty" << endl;
			return false;
		}
		//PointCloudT::Ptr smallcloud(new PointCloudT);
		//smallcloud = PointCloudScale(0.01, cloud);
		//float smoothness_threshold = 0.1;
		float voxel_resolution = 0.8f;
		float seed_resolution = 6.0f;
		float color_importance = 0.1f;
		float spatial_importance = 0.4f;
		float normal_importance = 1.0f;
		//生成结晶器
		pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
		super.setUseSingleCameraTransform(false);
		//输入点云和设置参数
		super.setInputCloud(cloud);
		super.setColorImportance(color_importance);
		super.setSpatialImportance(spatial_importance);
		super.setNormalImportance(normal_importance);
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
		super.extract(supervoxel_clusters);
		//cout << "super " << supervoxel_clusters.size() << endl;
		if (supervoxel_clusters.size() == 0)
		{
			cout << "supervoxel error size=0" << endl;
			return false;
		}
		std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
		super.getSupervoxelAdjacency(supervoxel_adjacency);

		//区域生长
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;

		size_t voxelsize = 0;
		for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
		{
			voxelsize += it_clusters->second->voxels_->size();
		}

		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_minindex = supervoxel_clusters.begin();
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_m = supervoxel_clusters.begin();
		size_t tempmax = 0;
		for (; it_m != supervoxel_clusters.end(); it_m++)
		{
			if (it_m->first > tempmax)
			{
				tempmax = it_m->first;
			}
		}
		//cout << tempmax << ' ' << supervoxel_clusters.size() << endl;
		//facet_grow_cloud->resize(voxelsize);
		float normal_thresold = 15.0;//法线差异
		double gray_thresold = 20;
		//double all_thresold = 25;
		vector<int> unused_clusters(supervoxel_clusters.size() * 5, 0);
		int pointcount = 0;
		int facecount = 0;
		int loopcount = 0;
		//double color_im = 0.3;
		//double normal_im = 0.7;
		//cout << supervoxel_clusters.size() << endl;
		//vector<pcl::PointIndices> facetcluster;//分割后的点云索引
		int facetIndex = 0;
		vector<int> segmen_num;//存每个类的点数
		vector<int> point_laber;//存每个点的标签
		point_laber.resize(cloud->points.size());
		deque<int> face_que;
		while (facecount < supervoxel_clusters.size() && pointcount < voxelsize)
		{
			loopcount++;
			if (loopcount >(2 * supervoxel_clusters.size()))
			{
				break;
			}
			//找曲率最小的点
			double mincur = -999;

			for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
			{
				auto it_cur = it_clusters->second->normals_->begin();
				double tempcur = 0;
				for (; it_cur != it_clusters->second->normals_->end(); it_cur++)
				{
					tempcur += it_cur->curvature;
				}
				tempcur /= it_clusters->second->normals_->size();
				if (it_clusters->first >= unused_clusters.size())
				{
					cout << "out of range" << endl;
					cout << it_clusters->first << ' ' << unused_clusters.size() << endl;
					cout << "its unused" << endl;
				}
				if (unused_clusters[it_clusters->first] == 0 && tempcur > mincur)
				{
					mincur = tempcur;
					it_minindex = it_clusters;
				}
			}
			//cout << facecount << endl;
			//cout << mincur << endl;
			if (it_minindex->first >= unused_clusters.size())
			{
				cout << "out of range" << endl;
				cout << it_minindex->first << ' ' << unused_clusters.size() << endl;
				cout << "its unused" << endl;
			}
			if (unused_clusters[it_minindex->first] == 0)
			{
				int onefacetpointcount = 0;
				int color_R = Random(255);
				int color_G = Random(255);
				int color_B = Random(255);

				face_que.push_back(it_minindex->first);
				unused_clusters[it_minindex->first] = 1;
				facecount++;
				//染色
				for (int i = 0; i < it_minindex->second->voxels_->points.size(); ++i)
				{
					PointT facetemp;
					facetemp.x = it_minindex->second->voxels_->points[i].x;
					facetemp.y = it_minindex->second->voxels_->points[i].y;
					facetemp.z = it_minindex->second->voxels_->points[i].z;
					facetemp.r = color_R;
					facetemp.g = color_G;
					facetemp.b = color_B;
					facet_grow_cloud->push_back(facetemp);
					if (pointcount >= point_laber.size())
					{
						cout << "out of range" << endl;
						cout << pointcount << ' ' << point_laber.size() << endl;
						cout << "its unused" << endl;
					}
					point_laber[pointcount] = facetIndex;
					//facetcluster[facetIndex].indices.push_back(pointcount);
					++pointcount;
					onefacetpointcount++;
				}
				//float seed_cur = CalFacetCurv(it_clusters->second,smallcloud);
				int index = it_minindex->first;
				Vector3d face_normal;

				face_normal << supervoxel_clusters[index]->normal_.normal_x, supervoxel_clusters[index]->normal_.normal_y, supervoxel_clusters[index]->normal_.normal_z;
				//cout << "normal:"<<face_normal << endl;
				face_normal.normalize();
				//cout << face_normal << endl;
				int gray_seed = 0;
				for (int i = 0; i < supervoxel_clusters[index]->voxels_->points.size(); i++)
				{
					gray_seed = gray_seed + (supervoxel_clusters[index]->voxels_->points[i].r * 0.39 + supervoxel_clusters[index]->voxels_->points[i].g * 0.5 + supervoxel_clusters[index]->voxels_->points[i].b * 0.11);
				}
				gray_seed = gray_seed / supervoxel_clusters[index]->voxels_->points.size();
				while (!face_que.empty())
				{
					index = face_que.front();
					multimap<uint32_t, uint32_t>::iterator it_adjacency;
					it_adjacency = supervoxel_adjacency.find(index);
					for (int j = 0; j < supervoxel_adjacency.count(index); ++j)
					{
						int n = it_adjacency->second;
						Vector3d n_normal;
						n_normal << supervoxel_clusters[n]->normal_.normal_x, supervoxel_clusters[n]->normal_.normal_y, supervoxel_clusters[n]->normal_.normal_z;
						n_normal.normalize();
						//float target_cur = CalFacetCurv(supervoxel_clusters[n], smallcloud);
						//计算法线差异
						//float d = abs(seed_cur - target_cur);
						float d = acosf(face_normal.dot(n_normal))*180.0 / M_PI;
						//cosf(2.0 / 180.0*M_PI)
						//float d = acos((face_normal(0)*n_normal(0) + face_normal(1)*n_normal(1) + face_normal(2)*n_normal(2)) /
						//(sqrt(pow(face_normal(0), 2) + pow(face_normal(1), 2) + pow(face_normal(2), 2)) *sqrt(pow(n_normal(0), 2) + pow(n_normal(1), 2) + pow(n_normal(2), 2)))) * 180 / 3.1415926;
						//计算灰度值
						int gray = 0;
						for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); i++)
						{
							gray = gray + (supervoxel_clusters[n]->voxels_->points[i].r * 0.39 + supervoxel_clusters[n]->voxels_->points[i].g * 0.5 + supervoxel_clusters[n]->voxels_->points[i].b * 0.11);
						}
						gray = gray / supervoxel_clusters[n]->voxels_->points.size();

						//线性加权
						//d = d * 255 / 180;
						int g = std::abs(gray - gray_seed) * 180 / 255;
						//cout << g << endl;
						if (d < 2)
						{
							swap(normal_im, color_im);
						}
						double th = normal_im*d + color_im*g;
						//小于阈值并且这个面没有被归类过，入队
						if ((th < all_thresold) && unused_clusters[n] == 0)
						{
							//face_normal = face_normal + n_normal;
							//face_normal.normalize();
							face_que.push_back(n);
							//染相同颜色
							for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); ++i)
							{
								PointT facetemp;
								facetemp.x = supervoxel_clusters[n]->voxels_->points[i].x;
								facetemp.y = supervoxel_clusters[n]->voxels_->points[i].y;
								facetemp.z = supervoxel_clusters[n]->voxels_->points[i].z;
								facetemp.r = color_R;
								facetemp.g = color_G;
								facetemp.b = color_B;
								facet_grow_cloud->push_back(facetemp);
								point_laber[pointcount] = facetIndex;
								//facetcluster[facetIndex].indices.push_back(pointcount);
								++pointcount;
								onefacetpointcount++;
							}
							unused_clusters[n] = 1;
							facecount++;
						}
						it_adjacency++;
					}
					//出队
					face_que.pop_front();
				}
				face_que.clear();
				segmen_num.push_back(onefacetpointcount);
				//cout << segmen_num.capacity() << endl;
				facetIndex++;
			}
		}
		//cout << unused_clusters.size() << " segmen_num " << segmen_num.size() << endl;
		//cout << segmen_num.size() << ' ' << facetIndex << endl;
		facetcluster.resize(segmen_num.size());
		for (auto i = 0; i < segmen_num.size(); i++)
		{
			facetcluster[i].indices.resize(segmen_num[i]);
		}
		vector<int> counter_2;
		counter_2.resize(segmen_num.size(), 0);
		for (auto i = 0; i < facet_grow_cloud->points.size(); i++)
		{
			int seg_idx = point_laber[i];
			int nebor_idx = counter_2[seg_idx];
			facetcluster[seg_idx].indices[nebor_idx] = i;
			counter_2[seg_idx] += 1;
		}
		if (facet_grow_cloud->empty())
		{
			cout << "facet success wrong" << endl;
			return false;
		}
		//facet_grow_cloud = PointCloudScale(100, facet_grow_cloud);
		//getkeypoint(facetcluster, facet_grow_cloud);
		endtime = clock();
		double totaltime = (double)(endtime - starttime) / CLOCKS_PER_SEC;
		cout << "Total time:" << totaltime * 1000 << "ms" << endl;
		cout << "facet success" << endl;

	}
	catch (exception &e)
	{
		cout << e.what() << endl;
		return false;
	}
	return true;
}

bool PreTreatment::kmeansSeg(PointCloudT::Ptr cloud, double normal_im, double color_im, double all_thresold)
{
	clock_t starttime, endtime;
	starttime = clock();
	try {
		PointCloudT::Ptr facet_grow_cloud(new PointCloudT);
		facetcluster.clear();
		if (cloud->empty())
		{
			cout << "facet inputcloud empty" << endl;
			return false;
		}
		PointCloudT::Ptr smallcloud(new PointCloudT);
		smallcloud = PointCloudScale(0.01, cloud);
		//float smoothness_threshold = 0.1;
		float voxel_resolution = 0.008f;
		float seed_resolution = 0.06f;
		float color_importance = 0.1f;
		float spatial_importance = 0.4f;
		float normal_importance = 1.0f;
		//生成结晶器
		pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
		super.setUseSingleCameraTransform(false);
		//输入点云和设置参数
		super.setInputCloud(smallcloud);
		super.setColorImportance(color_importance);
		super.setSpatialImportance(spatial_importance);
		super.setNormalImportance(normal_importance);
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
		super.extract(supervoxel_clusters);
		cout << "super " << supervoxel_clusters.size() << endl;
		if (supervoxel_clusters.size() == 0)
		{
			cout << "supervoxel error size=0" << endl;
			return false;
		}
		std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
		super.getSupervoxelAdjacency(supervoxel_adjacency);

		//区域生长
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;

		size_t voxelsize = 0;
		for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
		{
			voxelsize += it_clusters->second->voxels_->size();
		}

		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_minindex = supervoxel_clusters.begin();
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_m = supervoxel_clusters.begin();
		size_t tempmax = 0;
		for (; it_m != supervoxel_clusters.end(); it_m++)
		{
			if (it_m->first > tempmax)
			{
				tempmax = it_m->first;
			}
		}
		cout << tempmax << ' ' << supervoxel_clusters.size() << endl;
		//facet_grow_cloud->resize(voxelsize);
		float normal_thresold = 15.0;//法线差异
		double gray_thresold = 20;
		//double all_thresold = 25;
		vector<int> unused_clusters(supervoxel_clusters.size() * 5, 0);
		int pointcount = 0;
		int facecount = 0;
		int loopcount = 0;
		//double color_im = 0.3;
		//double normal_im = 0.7;
		//cout << supervoxel_clusters.size() << endl;
		//vector<pcl::PointIndices> facetcluster;//分割后的点云索引
		int facetIndex = 0;
		vector<int> segmen_num;//存每个类的点数
		vector<int> point_laber;//存每个点的标签
		point_laber.resize(smallcloud->points.size());
		deque<int> face_que;
		while (facecount < supervoxel_clusters.size() && pointcount < voxelsize)
		{
			loopcount++;
			if (loopcount >(2 * supervoxel_clusters.size()))
			{
				break;
			}
			//找曲率最小的点
			double mincur = -999;

			for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
			{
				auto it_cur = it_clusters->second->normals_->begin();
				double tempcur = 0;
				for (; it_cur != it_clusters->second->normals_->end(); it_cur++)
				{
					tempcur += it_cur->curvature;
				}
				tempcur /= it_clusters->second->normals_->size();
				if (it_clusters->first >= unused_clusters.size())
				{
					cout << "out of range" << endl;
					cout << it_clusters->first << ' ' << unused_clusters.size() << endl;
					cout << "its unused" << endl;
				}
				if (unused_clusters[it_clusters->first] == 0 && tempcur > mincur)
				{
					mincur = tempcur;
					it_minindex = it_clusters;
				}
			}
			//cout << facecount << endl;
			//cout << mincur << endl;
			if (it_minindex->first >= unused_clusters.size())
			{
				cout << "out of range" << endl;
				cout << it_minindex->first << ' ' << unused_clusters.size() << endl;
				cout << "its unused" << endl;
			}
			if (unused_clusters[it_minindex->first] == 0)
			{
				int onefacetpointcount = 0;
				int color_R = Random(255);
				int color_G = Random(255);
				int color_B = Random(255);

				face_que.push_back(it_minindex->first);
				unused_clusters[it_minindex->first] = 1;
				facecount++;
				//染色
				for (int i = 0; i < it_minindex->second->voxels_->points.size(); ++i)
				{
					PointT facetemp;
					facetemp.x = it_minindex->second->voxels_->points[i].x;
					facetemp.y = it_minindex->second->voxels_->points[i].y;
					facetemp.z = it_minindex->second->voxels_->points[i].z;
					facetemp.r = color_R;
					facetemp.g = color_G;
					facetemp.b = color_B;
					facet_grow_cloud->push_back(facetemp);
					if (pointcount >= point_laber.size())
					{
						cout << "out of range" << endl;
						cout << pointcount << ' ' << point_laber.size() << endl;
						cout << "its unused" << endl;
					}
					point_laber[pointcount] = facetIndex;
					//facetcluster[facetIndex].indices.push_back(pointcount);
					++pointcount;
					onefacetpointcount++;
				}
				//float seed_cur = CalFacetCurv(it_clusters->second,smallcloud);
				int index = it_minindex->first;
				Vector3d face_normal;

				face_normal << supervoxel_clusters[index]->normal_.normal_x, supervoxel_clusters[index]->normal_.normal_y, supervoxel_clusters[index]->normal_.normal_z;
				//cout << "normal:"<<face_normal << endl;
				face_normal.normalize();
				//cout << face_normal << endl;
				int gray_seed = 0;
				for (int i = 0; i < supervoxel_clusters[index]->voxels_->points.size(); i++)
				{
					gray_seed = gray_seed + (supervoxel_clusters[index]->voxels_->points[i].r * 0.39 + supervoxel_clusters[index]->voxels_->points[i].g * 0.5 + supervoxel_clusters[index]->voxels_->points[i].b * 0.11);
				}
				gray_seed = gray_seed / supervoxel_clusters[index]->voxels_->points.size();
				while (!face_que.empty())
				{
					index = face_que.front();
					multimap<uint32_t, uint32_t>::iterator it_adjacency;
					it_adjacency = supervoxel_adjacency.find(index);
					for (int j = 0; j < supervoxel_adjacency.count(index); ++j)
					{
						int n = it_adjacency->second;
						Vector3d n_normal;
						n_normal << supervoxel_clusters[n]->normal_.normal_x, supervoxel_clusters[n]->normal_.normal_y, supervoxel_clusters[n]->normal_.normal_z;
						n_normal.normalize();
						//float target_cur = CalFacetCurv(supervoxel_clusters[n], smallcloud);
						//计算法线差异
						//float d = abs(seed_cur - target_cur);
						float d = acosf(face_normal.dot(n_normal))*180.0 / M_PI;
						//cosf(2.0 / 180.0*M_PI)
						//float d = acos((face_normal(0)*n_normal(0) + face_normal(1)*n_normal(1) + face_normal(2)*n_normal(2)) /
						//(sqrt(pow(face_normal(0), 2) + pow(face_normal(1), 2) + pow(face_normal(2), 2)) *sqrt(pow(n_normal(0), 2) + pow(n_normal(1), 2) + pow(n_normal(2), 2)))) * 180 / 3.1415926;
						//计算灰度值
						int gray = 0;
						for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); i++)
						{
							gray = gray + (supervoxel_clusters[n]->voxels_->points[i].r * 0.39 + supervoxel_clusters[n]->voxels_->points[i].g * 0.5 + supervoxel_clusters[n]->voxels_->points[i].b * 0.11);
						}
						gray = gray / supervoxel_clusters[n]->voxels_->points.size();

						//线性加权
						//d = d * 255 / 180;
						int g = std::abs(gray - gray_seed) * 180 / 255;
						//cout << g << endl;
						double th = normal_im*d + color_im*g;
						//小于阈值并且这个面没有被归类过，入队
						if ((th < all_thresold) && unused_clusters[n] == 0)
						{
							//face_normal = face_normal + n_normal;
							//face_normal.normalize();
							face_que.push_back(n);
							//染相同颜色
							for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); ++i)
							{
								PointT facetemp;
								facetemp.x = supervoxel_clusters[n]->voxels_->points[i].x;
								facetemp.y = supervoxel_clusters[n]->voxels_->points[i].y;
								facetemp.z = supervoxel_clusters[n]->voxels_->points[i].z;
								facetemp.r = color_R;
								facetemp.g = color_G;
								facetemp.b = color_B;
								facet_grow_cloud->push_back(facetemp);
								point_laber[pointcount] = facetIndex;
								//facetcluster[facetIndex].indices.push_back(pointcount);
								++pointcount;
								onefacetpointcount++;
							}
							unused_clusters[n] = 1;
							facecount++;
						}
						it_adjacency++;
					}
					//出队
					face_que.pop_front();
				}
				face_que.clear();
				segmen_num.push_back(onefacetpointcount);
				//cout << segmen_num.capacity() << endl;
				facetIndex++;
			}
		}
		cout << unused_clusters.size() << " segmen_num " << segmen_num.size() << endl;
		//cout << segmen_num.size() << ' ' << facetIndex << endl;
		facetcluster.resize(segmen_num.size());
		for (auto i = 0; i < segmen_num.size(); i++)
		{
			facetcluster[i].indices.resize(segmen_num[i]);
		}
		vector<int> counter_2;
		counter_2.resize(segmen_num.size(), 0);
		for (auto i = 0; i < facet_grow_cloud->points.size(); i++)
		{
			int seg_idx = point_laber[i];
			int nebor_idx = counter_2[seg_idx];
			facetcluster[seg_idx].indices[nebor_idx] = i;
			counter_2[seg_idx] += 1;
		}
		if (facet_grow_cloud->empty())
		{
			cout << "facet success wrong" << endl;
			return false;
		}
		facet_grow_cloud = PointCloudScale(100, facet_grow_cloud);
		//getkeypoint(facetcluster, facet_grow_cloud);
		endtime = clock();
		double totaltime = (double)(endtime - starttime) / CLOCKS_PER_SEC;
		cout << "Total time:" << totaltime * 1000 << "ms" << endl;
		cout << "facet success" << endl;

	}
	catch (exception &e)
	{
		cout << e.what() << endl;
		return false;
	}
	return true;
}

bool PreTreatment::FacetSegmentationRGB(PointCloudT::Ptr cloud, PointCloudT::Ptr &facet_grow_cloud, double normal_im, double color_im, double all_thresold)
{
	try {
		facetclusterrgb.clear();
		if (cloud->empty())
		{
			cout << "facet inputcloud empty" << endl;
			return false;
		}
		PointCloudT::Ptr smallcloud(new PointCloudT);
		smallcloud = PointCloudScale(0.01, cloud);
		//float smoothness_threshold = 0.1;
		float voxel_resolution = 0.008f;
		float seed_resolution = 0.06f;
		float color_importance = 1.0f;
		float spatial_importance = 0.5f;
		float normal_importance = 0.1f;
		//生成结晶器
		pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
		super.setUseSingleCameraTransform(false);
		//输入点云和设置参数
		super.setInputCloud(smallcloud);
		super.setColorImportance(color_importance);
		super.setSpatialImportance(spatial_importance);
		super.setNormalImportance(normal_importance);
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
		super.extract(supervoxel_clusters);
		cout << "super " << supervoxel_clusters.size() << endl;
		if (supervoxel_clusters.size() == 0)
		{
			cout << "supervoxel error size=0" << endl;
			return false;
		}
		std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
		super.getSupervoxelAdjacency(supervoxel_adjacency);

		//区域生长
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;

		size_t voxelsize = 0;
		for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
		{
			voxelsize += it_clusters->second->voxels_->size();
		}
		//pcl::PointCloud<PointT>::Ptr facet_grow_cloud(new pcl::PointCloud<PointT>());


		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_minindex = supervoxel_clusters.begin();

		//facet_grow_cloud->resize(voxelsize);
		float normal_thresold = 15.0;//法线差异
		double gray_thresold = 20;
		//double all_thresold = 25;
		vector<int> unused_clusters(supervoxel_clusters.size()*5, 0);
		int pointcount = 0;
		int facecount = 0;
		int loopcount = 0;
		//double color_im = 0.3;
		//double normal_im = 0.7;
		//cout << supervoxel_clusters.size() << endl;
		//vector<pcl::PointIndices> facetcluster;//分割后的点云索引
		int facetIndex = 0;
		vector<int> segmen_num;//存每个类的点数
		vector<int> point_laber;//存每个点的标签
		point_laber.resize(smallcloud->points.size());
		deque<int> face_que;
		while (facecount < supervoxel_clusters.size() && pointcount < voxelsize)
		{
			loopcount++;
			if (loopcount > (2 * supervoxel_clusters.size()))
			{
				break;
			}
			//灰度值最da的点开始
			double maxgray = 999;

			for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
			{
				auto it_g = it_clusters->second->voxels_->points.begin();
				size_t tempg = 0;
				for (; it_g != it_clusters->second->voxels_->points.end(); it_g++)
				{
					tempg += ((it_g->r * 30 + it_g->g * 59 + it_g->b * 11 + 50) / 100);
				}
				tempg /= it_clusters->second->voxels_->points.size();
				if (unused_clusters[it_clusters->first]==0 && tempg < maxgray)
				{
					maxgray = tempg;
					it_minindex = it_clusters;
				}
			}
			//cout << facecount << endl;
			//cout << mincur << endl;
			if (unused_clusters[it_minindex->first]==0)
			{
				int onefacetpointcount = 0;
				int color_R = Random(255);
				int color_G = Random(255);
				int color_B = Random(255);

				face_que.push_back(it_minindex->first);
				unused_clusters[it_minindex->first] = 1;
				facecount++;
				//染色
				for (int i = 0; i < it_minindex->second->voxels_->points.size(); ++i)
				{
					PointT facetemp;
					/*facet_grow_cloud->points[pointcount].x = it_minindex->second->voxels_->points[i].x;
					facet_grow_cloud->points[pointcount].y = it_minindex->second->voxels_->points[i].y;
					facet_grow_cloud->points[pointcount].z = it_minindex->second->voxels_->points[i].z;
					facet_grow_cloud->points[pointcount].r = color_R;
					facet_grow_cloud->points[pointcount].g = color_G;
					facet_grow_cloud->points[pointcount].b = color_B;*/
					facetemp.x = it_minindex->second->voxels_->points[i].x;
					facetemp.y = it_minindex->second->voxels_->points[i].y;
					facetemp.z = it_minindex->second->voxels_->points[i].z;
					facetemp.r = color_R;
					facetemp.g = color_G;
					facetemp.b = color_B;
					facet_grow_cloud->push_back(facetemp);
					point_laber[pointcount] = facetIndex;
					//facetcluster[facetIndex].indices.push_back(pointcount);
					++pointcount;
					onefacetpointcount++;
				}
				//float seed_cur = CalFacetCurv(it_clusters->second,smallcloud);
				int index = it_minindex->first;
				Vector3d face_normal;

				face_normal << supervoxel_clusters[index]->normal_.normal_x, supervoxel_clusters[index]->normal_.normal_y, supervoxel_clusters[index]->normal_.normal_z;
				//cout << "normal:"<<face_normal << endl;
				face_normal.normalize();
				//cout << face_normal << endl;
				int gray_seed = 0;
				for (int i = 0; i < supervoxel_clusters[index]->voxels_->points.size(); i++)
				{
					gray_seed = gray_seed + ((supervoxel_clusters[index]->voxels_->points[i].r * 30 + supervoxel_clusters[index]->voxels_->points[i].g * 59 + supervoxel_clusters[index]->voxels_->points[i].b * 11 + 50) / 100);
				}
				gray_seed = gray_seed / supervoxel_clusters[index]->voxels_->points.size();
				while (!face_que.empty())
				{
					index = face_que.front();
					multimap<uint32_t, uint32_t>::iterator it_adjacency;
					it_adjacency = supervoxel_adjacency.find(index);
					for (int j = 0; j < supervoxel_adjacency.count(index); ++j)
					{
						int n = it_adjacency->second;
						Vector3d n_normal;
						n_normal << supervoxel_clusters[n]->normal_.normal_x, supervoxel_clusters[n]->normal_.normal_y, supervoxel_clusters[n]->normal_.normal_z;
						n_normal.normalize();
						//float target_cur = CalFacetCurv(supervoxel_clusters[n], smallcloud);
						//计算法线差异
						//float d = abs(seed_cur - target_cur);
						float d = acosf(face_normal.dot(n_normal)) * 180 / M_PI;
						//float d = acos((face_normal(0)*n_normal(0) + face_normal(1)*n_normal(1) + face_normal(2)*n_normal(2)) /
							//(sqrt(pow(face_normal(0), 2) + pow(face_normal(1), 2) + pow(face_normal(2), 2)) *sqrt(pow(n_normal(0), 2) + pow(n_normal(1), 2) + pow(n_normal(2), 2)))) * 180 / 3.1415926;
						//计算灰度值
						int gray = 0;
						for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); i++)
						{
							gray = gray + ((supervoxel_clusters[n]->voxels_->points[i].r * 30 + supervoxel_clusters[n]->voxels_->points[i].g * 59 + supervoxel_clusters[n]->voxels_->points[i].b * 11 + 50) / 100);
						}
						gray = gray / supervoxel_clusters[n]->voxels_->points.size();
						//线性加权
						//d = d * 255 / 180;
						//cout << d << endl;
						double th = normal_im*d + color_im*(abs(gray - gray_seed) * 180 / 255);
						//cout << th << endl;
						//小于阈值并且这个面没有被归类过，入队
						if ((th < all_thresold) && unused_clusters[n]==0)
						{
							//face_normal = face_normal + n_normal;
							//face_normal.normalize();
							face_que.push_back(n);
							//染相同颜色
							for (int i = 0; i < supervoxel_clusters[n]->voxels_->points.size(); ++i)
							{
								PointT facetemp;
								facetemp.x = supervoxel_clusters[n]->voxels_->points[i].x;
								facetemp.y = supervoxel_clusters[n]->voxels_->points[i].y;
								facetemp.z = supervoxel_clusters[n]->voxels_->points[i].z;
								facetemp.r = color_R;
								facetemp.g = color_G;
								facetemp.b = color_B;
								facet_grow_cloud->push_back(facetemp);
								point_laber[pointcount] = facetIndex;
								//facetcluster[facetIndex].indices.push_back(pointcount);
								++pointcount;
								onefacetpointcount++;
							}
							unused_clusters[n] = 1;
							facecount++;
						}
						it_adjacency++;
					}
					//出队
					face_que.pop_front();
				}
				face_que.clear();
				segmen_num.push_back(onefacetpointcount);
				facetIndex++;
			}
		}
		//cout << segmen_num.size() << ' ' << facetIndex << endl;
		facetclusterrgb.resize(segmen_num.size());
		for (auto i = 0; i < segmen_num.size(); i++)
		{
			facetclusterrgb[i].indices.resize(segmen_num[i]);
		}
		vector<int> counter_2;
		counter_2.resize(segmen_num.size(), 0);
		for (auto i = 0; i < facet_grow_cloud->points.size(); i++)
		{
			int seg_idx = point_laber[i];
			int nebor_idx = counter_2[seg_idx];
			facetclusterrgb[seg_idx].indices[nebor_idx] = i;
			counter_2[seg_idx] += 1;
		}
		if (facet_grow_cloud->empty())
		{
			cout << "facet rgb success wrong" << endl;
			return false;
		}
		facet_grow_cloud = PointCloudScale(100, facet_grow_cloud);
		cout << "facet rgb success" << endl;
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
		return false;
	}
	return true;
}

int PreTreatment::cloudBin(PointCloudT::Ptr cloud)
{
	try
	{
		PointCloudT::Ptr bin_cloud(new PointCloudT);
		bin_cloud->resize(cloud->points.size());
		int thresold = 128;
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			int gray = (cloud->points[i].r * 30 + cloud->points[i].g * 59 + cloud->points[i].b * 11 + 50) / 100;
			if (gray >= 0 && gray <= 255)
			{
				if (gray > thresold)
				{
					bin_cloud->points[i].x = cloud->points[i].x;
					bin_cloud->points[i].y = cloud->points[i].y;
					bin_cloud->points[i].z = cloud->points[i].z;
					bin_cloud->points[i].r = 255;
					bin_cloud->points[i].g = 255;
					bin_cloud->points[i].b = 255;
				}
				else
				{
					bin_cloud->points[i].x = cloud->points[i].x;
					bin_cloud->points[i].y = cloud->points[i].y;
					bin_cloud->points[i].z = cloud->points[i].z;
					bin_cloud->points[i].r = 255;
					bin_cloud->points[i].g = 0;
					bin_cloud->points[i].b = 0;
				}
			}
			else {
				cout << "rgb error" << endl;
				return -1;
			}
		}
		cout << "bin success" << endl;
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

int PreTreatment::getharriskeypoint(PointCloudT::Ptr cloud)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI, pcl::Normal> harris;
		harris.setInputCloud(cloud);
		//harris.setNonMaxSupression(true);
		harris.setRadius(2.5f);
		//harris.setThreshold(0.2f);
		harris.setRadiusSearch(3.0f);
		harris.compute(*cloud_out);
		cout << "Harris_keypoints的大小是" << cloud_out->size() << endl;

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

PointCloudT::Ptr PreTreatment::segmentbycurvature(PointCloudT::Ptr cloud, int maxpoints)
{
	PointCloudT::Ptr cur_cloud(new PointCloudT);
	try
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
		pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
		normalEstimation.setInputCloud(cloud);
		normalEstimation.setRadiusSearch(1.5);
		//normalEstimation.setKSearch(20);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);
		//kdtree->setInputCloud(cloud);

		pcl::PrincipalCurvaturesEstimation<PointT, pcl::Normal, pcl::PrincipalCurvatures>pc;
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		pc.setInputCloud(cloud);
		pc.setInputNormals(normals);
		pc.setSearchMethod(kdtree);
		pc.setRadiusSearch(1.5);
		//pc.setKSearch(5);
		pc.compute(*cloud_curvatures);



		typedef struct PCURVATURE {
			int index;
			float curvature;
		}PCURVATURE;
		vector<PCURVATURE> tempcur;
		float curvature = 0.0;
		PCURVATURE pv;
		for (int i = 0; i < cloud_curvatures->size(); i++)
		{
			//平均曲率
			//curvature = ((*cloud_curvatures)[i].pc1 + (*cloud_curvatures)[i].pc2) / 2;
			//高斯曲率
			curvature = (*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2;
			pv.index = i;
			pv.curvature = curvature;
			tempcur.push_back(pv);
		}
		PCURVATURE temp;
		int maxIndex = 0;
		int count = 0;
		PointCloudT::Ptr result_cloud(new PointCloudT);
		for (int i = 0; i < tempcur.size(); i++) {
			float maxCurvature = -99999;
			for (int j = i + 1; j < tempcur.size(); j++) {
				if (maxCurvature < tempcur[j].curvature) {
					maxCurvature = tempcur[j].curvature;
					maxIndex = j;
				}
			}
			if (maxCurvature > tempcur[i].curvature) {
				temp = tempcur[maxIndex];
				tempcur[maxIndex] = tempcur[i];
				tempcur[i] = temp;
				count++;
				PointT p;
				p.x = cloud->points[temp.index].x;
				p.y = cloud->points[temp.index].y;
				p.z = cloud->points[temp.index].z;
				p.r = 255;
				p.g = 0;
				p.b = 0;
				result_cloud->points.push_back(p);

			}
			if (count > maxpoints) {
				break;
			}
		}
		result_cloud->resize(result_cloud->points.size());
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		
		for (int i = 0; i < result_cloud->points.size(); i++)
		{
			if (kdtree->radiusSearch(result_cloud->points[i], 5, pointIdxRadiusSearch, pointRadiusSquaredDistance))
			{
				if (pointIdxRadiusSearch.size() > 90)
				{
					cur_cloud->push_back(result_cloud->points[i]);
				}
			}
		}
		
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cur_cloud;
}

PointCloudT::Ptr PreTreatment::regiongrowingseg(PointCloudT::Ptr cloud, vector<pcl::PointIndices> &clusters)
{
	PointCloudT::Ptr cloud_color(new PointCloudT());
	try
	{
		cloud = PointCloudScale(0.01, cloud);
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		tree->setInputCloud(cloud);
		ne.setInputCloud(cloud);
		ne.setSearchMethod(tree);
		ne.setKSearch(50);
		ne.compute(*normals);
		int k = 30;
		size_t points_num = cloud->points.size();
		vector<int> k_nebor_index;
		vector<float> k_nebor_index_dis;
		vector<vector<int>> point_k_index;
		point_k_index.resize(points_num, k_nebor_index);
		for (size_t i = 0; i < points_num; i++)
		{
			if (tree->nearestKSearch(cloud->points[i], k, k_nebor_index, k_nebor_index_dis))
			{
				point_k_index[i].swap(k_nebor_index);
			}
			else
			{
				PCL_ERROR("WARNING:FALSE NEARERTKSEARCH");
			}
		}
		vector<pair<float, int>> vec_curvature;
		for (size_t i = 0; i < points_num; i++)
		{
			vec_curvature.push_back(make_pair(normals->points[i].curvature, i));
		}
		sort(vec_curvature.begin(), vec_curvature.end());
		float curvature_threshold = 1.0;
		float normal_threshold = cosf(2.5 / 180.0*M_PI);
		int seed_orginal = vec_curvature[0].second;
		int counter_0 = 0;
		int segment_laber(0);
		vector<int> segmen_num;
		vector<int> point_laber;
		point_laber.resize(points_num, -1);
		while (counter_0 < points_num)
		{
			queue<int> seed;
			seed.push(seed_orginal);
			point_laber[seed_orginal] = segment_laber;
			int counter_1(1);
			while (!seed.empty())
			{
				int curr_seed = seed.front();
				seed.pop();
				int curr_nebor_index(0);
				while (curr_nebor_index < k)
				{
					bool is_a_seed = false;
					int cur_point_idx = point_k_index[curr_seed][curr_nebor_index];
					if (point_laber[cur_point_idx] != -1)
					{
						curr_nebor_index++;
						continue;
					}
					Eigen::Map<Eigen::Vector3f> vec_curr_point(static_cast<float*>(normals->points[curr_seed].normal));
					Eigen::Map<Eigen::Vector3f> vec_nebor_point(static_cast<float*>(normals->points[cur_point_idx].normal));
					float dot_normals = fabsf(vec_curr_point.dot(vec_nebor_point));
					if (dot_normals < normal_threshold)
					{
						is_a_seed = false;
					}
					else if (normals->points[cur_point_idx].curvature > curvature_threshold)
					{
						is_a_seed = false;
					}
					else
					{
						is_a_seed = true;
					}
					if (!is_a_seed)
					{
						curr_nebor_index++;
						continue;
					}
					point_laber[cur_point_idx] = segment_laber;
					counter_1++;
					if (is_a_seed)
					{
						seed.push(cur_point_idx);
					}
					curr_nebor_index++;
				}
			}
			segment_laber++;
			counter_0 += counter_1;
			segmen_num.push_back(counter_1);
			for (size_t i = 0; i < points_num; i++)
			{
				int index_curvature = vec_curvature[i].second;
				if (point_laber[index_curvature] == -1)
				{
					seed_orginal = index_curvature;
					break;
				}
			}
		}
		cout << "seg_num:" << segmen_num.size() << endl;
		//summary of segmentation results
		vector<pcl::PointIndices> cluster;
		pcl::PointIndices segment_points;
		size_t seg_num = segmen_num.size();
		cluster.resize(seg_num, segment_points);
		for (size_t i_seg = 0; i_seg < seg_num; i_seg++)
		{
			cluster[i_seg].indices.resize(segmen_num[i_seg], 0);
		}
		vector<int> counter_2;
		counter_2.resize(seg_num, 0);
		for (int i_point = 0; i_point < points_num; i_point++)
		{
			int seg_idx = point_laber[i_point];
			int nebor_idx = counter_2[seg_idx];
			cluster[seg_idx].indices[nebor_idx] = i_point;
			counter_2[seg_idx] += 1;
		}

		//Remove outline points
		
		int minNum = 10;
		int maxNum = 100000;
		//挑出尾鳍
		for (size_t i_cluster = 0; i_cluster < seg_num; i_cluster++)
		{
			if (cluster[i_cluster].indices.size() < maxNum && cluster[i_cluster].indices.size() > minNum)
			{
				clusters.push_back(cluster[i_cluster]);
			}
		}

		
		srand(time(nullptr));
		vector<unsigned char> color;
		for (size_t i = 0; i < clusters.size(); i++)
		{
			color.push_back(static_cast<unsigned char>(rand() % 256));
			color.push_back(static_cast<unsigned char>(rand() % 256));
			color.push_back(static_cast<unsigned char>(rand() % 256));
		}
		int color_index(0);
		for (size_t i = 0; i < clusters.size(); i++)
		{
			for (size_t j = 0; j < clusters[i].indices.size(); j++)
			{
				PointT n_points;
				n_points.x = cloud->points[clusters[i].indices[j]].x;
				n_points.y = cloud->points[clusters[i].indices[j]].y;
				n_points.z = cloud->points[clusters[i].indices[j]].z;
				n_points.r = color[3 * color_index];
				n_points.g = color[3 * color_index + 1];
				n_points.b = color[3 * color_index + 2];
				cloud_color->push_back(n_points);
			}
			color_index++;
		}
		cloud_color = PointCloudScale(100, cloud_color);
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return cloud_color;
}
//计算尾鳍背鳍臀鳍关键点
int PreTreatment::getkeypoint(vector<pcl::PointIndices> clusters, PointCloudT::Ptr cloud) 
{
	try
	{
		PointCloudT::Ptr weiqi_cloud(new PointCloudT);
		//挑出尾鳍
		double maxavgx = 0;
		int maxavgxIndex = 0;
		int weiqi_size = 200;
		vector<PointT> avgxyz(clusters.size());
		for (auto i = 0; i < clusters.size(); i++)
		{
			//PointT avgxyz;
			//cout << avgxyz.x << endl;
			for (auto j = 0; j < clusters[i].indices.size(); j++)
			{
				avgxyz[i].x += cloud->points[clusters[i].indices[j]].x;
				avgxyz[i].y += cloud->points[clusters[i].indices[j]].y;
				avgxyz[i].z += cloud->points[clusters[i].indices[j]].z;
				if (cloud->points[clusters[i].indices[j]].x > weiqi_e.x)
				{
					weiqi_e = cloud->points[clusters[i].indices[j]];
				}
			}
			avgxyz[i].x = avgxyz[i].x / clusters[i].indices.size();
			avgxyz[i].y = avgxyz[i].y / clusters[i].indices.size();
			avgxyz[i].z = avgxyz[i].z / clusters[i].indices.size();
			//尾鳍
			if (avgxyz[i].x > maxavgx&&clusters[i].indices.size() > weiqi_size)
			{
				maxavgx = avgxyz[i].x;
				maxavgxIndex = i;
			}

		}
		//找尾鳍开始和结束的点
		weiqi_s.x = 9999;
		for (auto i = 0; i < clusters[maxavgxIndex].indices.size(); i++)
		{
			if (cloud->points[clusters[maxavgxIndex].indices[i]].x > weiqi_e.x)
			{
				weiqi_e = cloud->points[clusters[maxavgxIndex].indices[i]];
			}
			if (cloud->points[clusters[maxavgxIndex].indices[i]].x < weiqi_s.x)
			{
				weiqi_s = cloud->points[clusters[maxavgxIndex].indices[i]];
			}
		}
		vector<int> weiqi_idxs;
		for (int i = 0; i < clusters.size(); i++)
		{
			int tempcount = 0;
			PointT temppoint = weiqi_s;
			for (int j = 0; j < clusters[i].indices.size(); j++)
			{
				if (cloud->points[clusters[i].indices[j]].x >= weiqi_s.x)
				{
					tempcount++;
				}
				if (cloud->points[clusters[i].indices[j]].x < temppoint.x)
				{
					temppoint = cloud->points[clusters[i].indices[j]];
				}
			}
			if (tempcount > clusters[i].indices.size()*0.7)
			{
				weiqi_idxs.push_back(i);
				if (temppoint.x < weiqi_s.x)
				{
					weiqi_s = temppoint;
				}
			}
		}
		//cout  << full_length << endl;
		//cout << "weiqi_s: " << weiqi_s << endl;
		//cout << weiqi_e << endl;
		//cout << maxx << endl;
		//尾鳍起始点纠正
		if (abs(weiqi_e.x - weiqi_s.x) < full_length / 15 || abs(weiqi_e.x - weiqi_s.x) > full_length / 4 || abs(maxx - weiqi_e.x) > 10)
		{
			weiqi_s.x = maxx - full_length / 7;
			weiqi_s.y = 0;
			weiqi_s.z = 0;
			weiqi_e.x = maxx;
			weiqi_e.y = 0;
			weiqi_e.z = 0;
			weiqi_s = getNearestpoint(cloud, weiqi_s);
			weiqi_e = getNearestpoint(cloud, weiqi_e);
			for (auto i = 0; i < cloud->points.size(); i++)
			{
				if (cloud->points[i].x > weiqi_s.x&&cloud->points[i].x < weiqi_e.x)
				{
					weiqi_cloud->push_back(cloud->points[i]);
				}
			}
			//weiqi_cloud->resize(weiqi_cloud->points.size());
			cout << "weiqi fake" << maxx << endl;
		}
		else
		{
			weiqi_s.y = 0;
			weiqi_s.z = 0;
			weiqi_s = getNearestpoint(cloud, weiqi_s);
			for (int j = 0; j < weiqi_idxs.size(); j++)
			{
				for (auto i = 0; i < clusters[weiqi_idxs[j]].indices.size(); i++)
				{
					weiqi_cloud->push_back(cloud->points[clusters[weiqi_idxs[j]].indices[i]]);
				}
			}
			//weiqi_cloud->resize(weiqi_cloud->points.size());
			cout << "weiqi" << endl;
		}
		//找背鳍
		//PointCloudT::Ptr beiqi_cloud(new PointCloudT);
		beiqi_cloud = getdorsalfinonly(cloud, clusters, avgxyz);
		//找臀鳍
		//PointCloudT::Ptr tunqi_cloud(new PointCloudT);
		tunqi_cloud = getanalfinonly(cloud, clusters, avgxyz);
		//重新确定尾鳍
		
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

Mat PreTreatment::projectionOnXY(PointCloudT::Ptr cloud)
{
	
	Mat pro(600, 800, CV_8UC3, Scalar(0, 0, 0));
	int dx = pro.cols / 2;
	int dy = pro.rows / 2;
	//cout << dx << dy << endl;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		int px = cloud->points[i].x + dx;
		int py = dy - cloud->points[i].y;
		if (px < pro.cols&&px>0 && py < pro.rows&&py>0)
		{
			Vec3b rgb;
			rgb[0] = cloud->points[i].b;
			rgb[1] = cloud->points[i].g;
			rgb[2] = cloud->points[i].r;
			pro.at<Vec3b>(py, px) = rgb;
		}
	}
	imwrite("outputtest/pro.jpg", pro);
	return pro;
}

int PreTreatment::SegmentOn2D(Mat image)
{
	//闭操作
	Mat closed;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(2, 2));
	morphologyEx(image, closed, CV_MOP_CLOSE, kernel);
	imwrite("outputtest/close.jpg", closed);
	//提取边界
	Mat grayImg;
	cvtColor(image, grayImg, COLOR_RGB2GRAY);
	Mat edge;
	Laplacian(grayImg, edge, CV_16S, 3, 1, 0, BORDER_DEFAULT);
	//convertScaleAbs(dst, abs_dst);
	imwrite("outputtest/edge.jpg", edge);
	Mat bin_edge;
	threshold(edge, bin_edge, 0, 255, THRESH_BINARY | THRESH_TRIANGLE);
	imwrite("outputtest/bin_edge.jpg", bin_edge);
	return 1;
}

PointCloudTL::Ptr PreTreatment::getfins(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZL>::Ptr transformcloud(new pcl::PointCloud<pcl::PointXYZL>);
	try
	{
		MatrixXf xyz(cloud->points.size(), 3);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			xyz(i, 0) = cloud->points[i].x;
			xyz(i, 1) = cloud->points[i].y;
			xyz(i, 2) = cloud->points[i].z;
		}
		MatrixXf avg_t;//中心点
		MatrixXf rot_t;//旋转矩阵
		avg_t = xyz.colwise().mean();
		//cout <<"avg " <<avg << endl;
		MatrixXf adjust(cloud->points.size(), 3);
		for (int i = 0; i < adjust.rows(); i++) {
			adjust(i, 0) = xyz(i, 0) - avg_t(0, 0);
			adjust(i, 1) = xyz(i, 1) - avg_t(0, 1);
			adjust(i, 2) = xyz(i, 2) - avg_t(0, 2);
		}
		MatrixXf cov = adjust.adjoint()*adjust;
		//cout << cov << endl;
		SelfAdjointEigenSolver<MatrixXf> eig(cov);
		MatrixXf vec, val;
		vec = eig.eigenvectors();
		val = eig.eigenvalues();
		rot_t = vec;

		pcl::PointCloud<pcl::PointXYZL>::Ptr translatecloud(new pcl::PointCloud<pcl::PointXYZL>);
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		transform(0, 3) = 0 - avg_t(0, 0);
		transform(1, 3) = 0 - avg_t(0, 1);
		transform(2, 3) = 0 - avg_t(0, 2);
		pcl::transformPointCloud(*cloud, *translatecloud, transform);
		Eigen::Matrix3f axismat;
		axismat << 0, 0, 1,
			0, 1, 0,
			1, 0, 0;
		Eigen::Matrix3f rotmat;

		rotmat = rot_t.transpose().inverse()*axismat;
		//cout << rotmat << endl;
		
		transformcloud->resize(translatecloud->points.size());
		for (int i = 0; i < transformcloud->points.size(); i++)
		{
			transformcloud->points[i].x = translatecloud->points[i].x*rotmat(0, 0) + translatecloud->points[i].y*rotmat(1, 0) + translatecloud->points[i].z*rotmat(2, 0);
			transformcloud->points[i].y = translatecloud->points[i].x*rotmat(0, 1) + translatecloud->points[i].y*rotmat(1, 1) + translatecloud->points[i].z*rotmat(2, 1);
			transformcloud->points[i].z = translatecloud->points[i].x*rotmat(0, 2) + translatecloud->points[i].y*rotmat(1, 2) + translatecloud->points[i].z*rotmat(2, 2);
			transformcloud->points[i].label = translatecloud->points[i].label;
		}

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return transformcloud;
}

PointCloudTL::Ptr PreTreatment::SortByX(pcl::PointCloud<PointTL>::Ptr cloud)
{
	for (int i = 0; i < cloud->points.size(); i++)
	{
		for (int j = 0; j < cloud->points.size() - 1 - i; ++j)
		{
			if (cloud->points[j].x > cloud->points[j + 1].x)
			{
				swap(cloud->points[j], cloud->points[j + 1]);
			}
		}
	}
	return cloud;
}

PointCloudT::Ptr PreTreatment::SortByX(PointCloudT::Ptr cloud)
{
	for (int i = 0; i < cloud->points.size(); i++)
	{
		for (int j = 0; j < cloud->points.size() - 1 - i; ++j)
		{
			if (cloud->points[j].x > cloud->points[j + 1].x)
			{
				swap(cloud->points[j], cloud->points[j + 1]);
			}
		}
	}
	return cloud;
}

//开操作 先腐蚀 后膨胀
//其实是根据上下y距离腐蚀
PointCloudTL::Ptr PreTreatment::MorphologyOpen(PointCloudTL::Ptr cloud,double th)
{
	pcl::search::KdTree<PointTL>::Ptr tree(new pcl::search::KdTree<PointTL>);
	tree->setInputCloud(cloud);
	float r = 2.0;
	PointCloudTL::Ptr fushi_cloud(new PointCloudTL);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if (tree->radiusSearch(cloud->points[i], r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			//auto temp = min_element(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end());
			double maxy = -9999, miny = 9999;
			for (auto j = 0; j < pointIdxRadiusSearch.size(); ++j)
			{
				if (cloud->points[pointIdxRadiusSearch[j]].y > maxy)
					maxy = cloud->points[pointIdxRadiusSearch[j]].y;
				if (cloud->points[pointIdxRadiusSearch[j]].y < miny)
					miny = cloud->points[pointIdxRadiusSearch[j]].y;
			}
			if (maxy - miny > th)
			{
				fushi_cloud->push_back(cloud->points[i]);
			}
		}
	}
	return fushi_cloud;
}

PointCloudT::Ptr PreTreatment::gettailonly(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr tail_cloud(new PointCloudT);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x<rect.x&&cloud->points[i].x>tunqi_e.x + 10)
		{
			tail_cloud->push_back(cloud->points[i]);
		}
	}
	PointCloudT::Ptr tail_facet_cloud(new PointCloudT);
	FacetSegmentationRGB(tail_cloud, tail_facet_cloud, 0.2, 0.8, 30);
	
	return tail_facet_cloud;
}

PointCloudT::Ptr PreTreatment::getdorsalfinonly(PointCloudT::Ptr cloud, vector<pcl::PointIndices> clusters, vector<PointT> avgxyz)
{
	
	try
	{
		vector<int> beiqis;
		int beiqisize = 20;
		for (auto i = 0; i < avgxyz.size(); i++)
		{
			if (avgxyz[i].y > 0 && avgxyz[i].x < weiqi_s.x - full_length / 7 && avgxyz[i].x>rect.centerpoint.x - full_length / 10)
			{
				beiqis.push_back(i);
			}
		}
		//cout << beiqis.size() << endl;
		size_t labelcount = beiqis.size();
		PointCloudTL::Ptr beiqis_cloud(new PointCloudTL);
		for (auto i = 0; i < beiqis.size(); i++)
		{
			for (auto j = 0; j < clusters[beiqis[i]].indices.size(); j++)
			{
				PointTL ptl;
				ptl.x = cloud->points[clusters[beiqis[i]].indices[j]].x;
				ptl.y = cloud->points[clusters[beiqis[i]].indices[j]].y;
				ptl.z = cloud->points[clusters[beiqis[i]].indices[j]].z;
				ptl.label = beiqis[i];
				beiqis_cloud->push_back(ptl);
			}
		}
		//PointCloudT::Ptr cloud_EuSeg(new PointCloudT);
		if (!beiqis_cloud->empty())
		{
			PointCloudTL::Ptr open_cloud(new PointCloudTL);
			open_cloud = MorphologyOpen(beiqis_cloud,2);//tunqi
			map<int, int> beiqislabel;
			for (int i = 0; i < open_cloud->points.size(); ++i)
			{
				beiqislabel[open_cloud->points[i].label] += 1;
			}
			cout << beiqislabel.size() << endl;
			map<int, int>::iterator label_iter;

			for (label_iter = beiqislabel.begin(); label_iter != beiqislabel.end(); label_iter++)
			{
				cout << label_iter->second << " : " << clusters[label_iter->first].indices.size()*0.2 << endl;
				if (label_iter->second > clusters[label_iter->first].indices.size()*0.2)
				{
					for (int i = 0; i < clusters[label_iter->first].indices.size(); i++)
					{
						beiqi_cloud->push_back(cloud->points[clusters[label_iter->first].indices[i]]);
					}
				}
			}

			//确定背鳍的点
			double tempminx = 999;
			//double tempminy = 999;
			_BoundRect beiqi_rect = getBoundRect(beiqi_cloud);
			PointT beiqi_RB;
			beiqi_RB.x = beiqi_rect.centerpoint.x + beiqi_rect.x / 2;
			beiqi_RB.y = beiqi_rect.centerpoint.y - beiqi_rect.y / 2;
			beiqi_RB.z = beiqi_rect.centerpoint.z + beiqi_rect.z / 2;
			beiqi_e = getNearestpoint(beiqi_cloud, beiqi_RB);
			PointT beiqiavg;
			for (int i = 0; i < beiqi_cloud->points.size(); i++)
			{
				beiqiavg.x += beiqi_cloud->points[i].x;
				beiqiavg.y += beiqi_cloud->points[i].y;
				beiqiavg.z += beiqi_cloud->points[i].z;
				if (beiqi_cloud->points[i].x < tempminx)
				{
					tempminx = beiqi_cloud->points[i].x;
					beiqi_s = beiqi_cloud->points[i];
				}
				//if (beiqi_cloud->points[i].y < tempminy)
				//{
					//tempminy = beiqi_cloud->points[i].y;
					//beiqi_e = beiqi_cloud->points[i];
				//}
			}
			beiqiavg.x = beiqiavg.x / beiqi_cloud->points.size();
			beiqiavg.y = beiqiavg.y / beiqi_cloud->points.size();
			beiqiavg.z = beiqiavg.z / beiqi_cloud->points.size();
			PointT fishcenter;
			fishcenter.x = avg(0, 0);
			fishcenter.y = avg(0, 1);
			fishcenter.z = avg(0, 2);
			float beiqi_dis = CalTwoPointDistance(beiqiavg, fishcenter);
			//纠正可能错误的背鳍点
			if (abs(beiqi_e.x - weiqi_s.x) < full_length / 7)
			{
				PointT fake_s;
				fake_s.x = rect.centerpoint.x;
				fake_s.y = rect.centerpoint.y + rect.y / 3;
				fake_s.z = rect.centerpoint.z;
				fake_s = getNearestpoint(cloud, fake_s);
				beiqi_s = fake_s;
				PointT fake_e;
				fake_e = fake_s;
				fake_e.x += full_length / 15;
				fake_e = getNearestpoint(cloud, fake_e);
				beiqi_e = fake_e;
			}
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return beiqi_cloud;
}

PointCloudT::Ptr PreTreatment::getanalfinonly(PointCloudT::Ptr cloud, vector<pcl::PointIndices> clusters, vector<PointT> avgxyz)
{
	
	try
	{
		vector<int> tunqis;
		int tunqisize = 20;
		for (auto i = 0; i < avgxyz.size(); i++)
		{
			if (avgxyz[i].y < 0 && avgxyz[i].x < weiqi_s.x - 15 && avgxyz[i].x>beiqi_e.x+10)
			{
				tunqis.push_back(i);
			}
		}
		//cout << tunqis.size() << endl;
		size_t labelcount = tunqis.size();
		PointCloudTL::Ptr tunqis_cloud(new PointCloudTL);
		PointCloudT::Ptr cloud_EuSeg(new PointCloudT);
		for (auto i = 0; i < tunqis.size(); i++)
		{
			for (auto j = 0; j < clusters[tunqis[i]].indices.size(); j++)
			{
				PointTL ptl;
				ptl.x = cloud->points[clusters[tunqis[i]].indices[j]].x;
				ptl.y = cloud->points[clusters[tunqis[i]].indices[j]].y;
				ptl.z = cloud->points[clusters[tunqis[i]].indices[j]].z;
				ptl.label = tunqis[i];
				tunqis_cloud->push_back(ptl);
			}
		}
		if (!tunqis_cloud->empty())
		{
			
			PointCloudTL::Ptr open_cloud(new PointCloudTL);
			open_cloud = MorphologyOpen(tunqis_cloud,2);//tunqi
			map<int, int> tunqislabel;
			for (int i = 0; i < open_cloud->points.size(); ++i)
			{
				tunqislabel[open_cloud->points[i].label] += 1;
			}
			cout << tunqislabel.size() << endl;
			
			map<int, int>::iterator label_iter;

			for (label_iter = tunqislabel.begin(); label_iter != tunqislabel.end(); label_iter++)
			{
				cout << label_iter->second<<" : " <<clusters[label_iter->first].indices.size()*0.2 << endl;
				if (label_iter->second > clusters[label_iter->first].indices.size()*0.2)
				{
					for (int i = 0; i < clusters[label_iter->first].indices.size(); i++)
					{
						cloud_EuSeg->push_back(cloud->points[clusters[label_iter->first].indices[i]]);
					}
				}
			}

			//欧式聚类找臀鳍
			
			pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
			tree->setInputCloud(cloud_EuSeg);
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<PointT> ec;   //欧式聚类对象
			ec.setClusterTolerance(1.5f);                     // 设置近邻搜索的搜索半径为1.5cm
			ec.setMinClusterSize(10);                 //设置一个聚类需要的最少的点数目为100
			ec.setMaxClusterSize(1000000);               //设置一个聚类需要的最大点数目为25000
			ec.setSearchMethod(tree);                    //设置点云的搜索机制
			ec.setInputCloud(cloud_EuSeg);
			ec.extract(cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

			std::vector<pcl::PointIndices>::const_iterator it;
			std::vector<pcl::PointIndices>::const_iterator temp;
			size_t maxpoints = 0;
			for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			{
				if (it->indices.size() > maxpoints)
				{
					maxpoints = it->indices.size();
					temp = it;
				}
			}
			PointT pointEuc;
			for (int i = 0; i < temp->indices.size(); ++i)
			{
				pointEuc = cloud_EuSeg->points[temp->indices[i]];
				tunqi_cloud->points.push_back(pointEuc);
			}
			tunqi_cloud->resize(tunqi_cloud->points.size());
			//确定臀鳍的点
			double tempminx = 999;
			//double tempmaxy = -999;
			_BoundRect tq_rect = getBoundRect(tunqi_cloud);
			PointT tq_RT;
			tq_RT.x = tq_rect.centerpoint.x + tq_rect.x / 2;
			tq_RT.y = tq_rect.centerpoint.y + tq_rect.y / 2;
			tq_RT.z = tq_rect.centerpoint.z - tq_rect.z / 2;
			tunqi_e = getNearestpoint(tunqi_cloud, tq_RT);
			double maxtunqiy = -9999;
			for (int i = 0; i < tunqi_cloud->points.size(); ++i)
			{
				if (tunqi_cloud->points[i].y > maxtunqiy)
				{
					maxtunqiy = tunqi_cloud->points[i].y;
					tunqi_e = tunqi_cloud->points[i];
				}
			}
			//cout << tunqi_e.x << ' ' << tunqi_e.y << ' ' << tunqi_e.z << endl;
			PointT tunqiavg;
			for (int i = 0; i < tunqi_cloud->points.size(); i++)
			{
				tunqiavg.x += tunqi_cloud->points[i].x;
				tunqiavg.y += tunqi_cloud->points[i].y;
				tunqiavg.z += tunqi_cloud->points[i].z;
				if (tunqi_cloud->points[i].x < tempminx)
				{
					tempminx = tunqi_cloud->points[i].x;
					tunqi_s = tunqi_cloud->points[i];
				}
				//if (tunqi_cloud->points[i].y > tempmaxy)
				//{
					//tempmaxy = tunqi_cloud->points[i].y;
					//tunqi_e = tunqi_cloud->points[i];
				//}
			}
			tunqiavg.x = tunqiavg.x / tunqi_cloud->points.size();
			tunqiavg.y = tunqiavg.y / tunqi_cloud->points.size();
			tunqiavg.z = tunqiavg.z / tunqi_cloud->points.size();
			PointT fishcenter;
			fishcenter.x = avg(0, 0);
			fishcenter.y = avg(0, 1);
			fishcenter.z = avg(0, 2);
			/*if (tqf.size() > 5)
			{
				int offset = tqf.back() - tqf[0];
				tunqi_s.x -= offset;
				tunqi_e.x -= offset;
				tunqi_s = getNearestpoint(cloud, tunqi_s);
				tunqi_e = getNearestpoint(cloud, tunqi_e);
			}*/
			float tunqi_dis = CalTwoPointDistance(tunqi_s, tunqi_e);


			//纠正可能错误的臀鳍点
			if (tunqi_dis > 60 || ((tunqi_s.x + tunqi_e.x) / 2 > weiqi_s.x || (tunqi_s.x + tunqi_e.x) / 2 < beiqi_e.x))
			{
				PointT fake_s;
				fake_s.x = rect.centerpoint.x + rect.x / 6.5;
				fake_s.y = rect.centerpoint.y - rect.y / 2;
				fake_s.z = rect.centerpoint.z;
				fake_s = getNearestpoint(cloud, fake_s);
				tunqi_s = fake_s;
				PointT fake_e;
				fake_e = fake_s;
				fake_e.x += full_length / 13;
				fake_e = getNearestpoint(cloud, fake_e);
				tunqi_e = fake_e;
				cout << "fake tunqi!!!" << endl;
			}
		}
		else
		{
			getfakeanalfin(cloud);
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return tunqi_cloud;
}

float PreTreatment::CalTwoPointDistance(PointT point1, PointT point2)
{
	float distance = 0;
	distance = sqrtf((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y) + (point1.z - point2.z)*(point1.z - point2.z));
	return distance;
}

_BoundRect PreTreatment::getBoundRect(PointCloudT::Ptr cloud)
{
	_BoundRect boundrect;
	try
	{
		PointCloudXYZ::Ptr xyzcloud(new PointCloudXYZ);
		xyzcloud->resize(cloud->points.size());
		for (int i = 0; i < cloud->points.size(); i++)
		{
			xyzcloud->points[i].x = cloud->points[i].x;
			xyzcloud->points[i].y = cloud->points[i].y;
			xyzcloud->points[i].z = cloud->points[i].z;
		}
		Pointxyz minPt, maxPt;
		pcl::getMinMax3D(*xyzcloud, minPt, maxPt);
		
		boundrect.centerpoint.x = (minPt.x + maxPt.x) / 2;
		boundrect.centerpoint.y = (minPt.y + maxPt.y) / 2;
		boundrect.centerpoint.z = (minPt.z + maxPt.z) / 2;
		boundrect.x = abs(maxPt.x - minPt.x);
		boundrect.y = abs(maxPt.y - minPt.y);
		boundrect.z = abs(maxPt.z - minPt.z);
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return boundrect;
}

PointT PreTreatment::getNearestpoint(PointCloudT::Ptr cloud, PointT point)
{
	PointT targetpoint;
	try
	{
		if (!cloud->empty())
		{
			float mindistance = CalTwoPointDistance(point, cloud->points[0]);
			targetpoint = cloud->points[0];
			for (int i = 0; i < cloud->points.size(); i++)
			{
				if (CalTwoPointDistance(point, cloud->points[i]) < mindistance)
				{
					mindistance = CalTwoPointDistance(point, cloud->points[i]);
					targetpoint = cloud->points[i];
				}
			}
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	//cout << targetpoint.x << ' ' << targetpoint.y << ' ' << targetpoint.z << endl;
	return targetpoint;
}

int PreTreatment::Drawkeypoints()
{
	try
	{
		PointCloudT::Ptr keypoints_cloud(new PointCloudT);
		weiqi_s.r = 255;
		weiqi_s.g = 0;
		weiqi_s.b = 0;
		weiqi_e.r = 255;
		weiqi_e.g = 0;
		weiqi_e.b = 0;
		keypoints_cloud->push_back(weiqi_s);
		keypoints_cloud->push_back(weiqi_e);
		beiqi_s.r = 0;
		beiqi_s.g = 255;
		beiqi_s.b = 0;
		beiqi_e.r = 0;
		beiqi_e.g = 255;
		beiqi_e.b = 0;
		keypoints_cloud->push_back(beiqi_s);
		keypoints_cloud->push_back(beiqi_e);
		tunqi_s.r = 255;
		tunqi_s.g = 255;
		tunqi_s.b = 0;
		tunqi_e.r = 255;
		tunqi_e.g = 255;
		tunqi_e.b = 0;
		keypoints_cloud->push_back(tunqi_s);
		keypoints_cloud->push_back(tunqi_e);
		xqi_s.r = 0;
		xqi_s.g = 255;
		xqi_s.b = 255;
		xqi_e.r = 0;
		xqi_e.g = 255;
		xqi_e.b = 255;
		keypoints_cloud->push_back(xqi_s);
		keypoints_cloud->push_back(xqi_e);
		fuqi_s.r = 0;
		fuqi_s.g = 0;
		fuqi_s.b = 255;
		fuqi_e.r = 0;
		fuqi_e.g = 0;
		fuqi_e.b = 255;
		keypoints_cloud->push_back(fuqi_s);
		keypoints_cloud->push_back(fuqi_e);
		head_s.r = 255;
		head_s.g = 0;
		head_s.b = 255;
		head_e.r = 255;
		head_e.g = 0;
		head_e.b = 255;
		head_top.r = 255;
		head_top.g = 0;
		head_top.b = 255;
		keypoints_cloud->push_back(head_s);
		keypoints_cloud->push_back(head_e);
		keypoints_cloud->push_back(head_top);
		weib_top.r = 255;
		weib_top.g = 255;
		weib_top.b = 255;
		weib_bottom.r = 255;
		weib_bottom.g = 255;
		weib_bottom.b = 255;
		keypoints_cloud->push_back(weib_top);
		keypoints_cloud->push_back(weib_bottom);
		bei_top.r = 255;
		bei_top.g = 255;
		bei_top.b = 255;
		fu_bottom.r = 255;
		fu_bottom.g = 255;
		fu_bottom.b = 255;
		keypoints_cloud->push_back(bei_top);
		keypoints_cloud->push_back(fu_bottom);
		eye_s.r = 255;
		eye_s.g = 255;
		eye_s.b = 255;
		eye_e.r = 255;
		eye_e.g = 255;
		eye_e.b = 255;
		keypoints_cloud->push_back(eye_s);
		keypoints_cloud->push_back(eye_e);
		
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

bool PreTreatment::getbelly(PointCloudT::Ptr cloud, PointCloudT::Ptr &belly_cloud)
{
	try
	{
		//PointCloudT::Ptr belly_cloud(new PointCloudT);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloud->points[i].y<beiqi_e.y - rect.y / 6 && cloud->points[i].x < tunqi_s.x - full_length / 25 && cloud->points[i].x > minx + full_length / 8 && cloud->points[i].z> (rect.centerpoint.z - rect.z / 5))
			{
				belly_cloud->push_back(cloud->points[i]);
			}
		}
		//cout << rect.centerpoint.z <<' '<<rect.x << ' ' <<rect.y << ' '<<rect.z << endl;
		//PointCloudT::Ptr belly_cloud_c(new PointCloudT);
		//vector<pcl::PointIndices> clusters;
		//RegionGrowingRGBSeg(belly_cloud, belly_cloud_c, clusters);
		PointCloudT::Ptr facet_cloud(new PointCloudT);
		//vector<pcl::PointIndices> clusters;
		if (belly_cloud->points.size() == 0)
		{
			cout << "cant find belly" << endl;
			return false;
		}
		if (!FacetSegmentationRGB(belly_cloud, facet_cloud, 0.3, 0.7, 20))
		{
			cout << "belly error" << endl;
			return false;
		}
		cout << facet_cloud->points.size() << endl;
		//找曲率大的点
		PointCloudT::Ptr cur_cloud(new PointCloudT);
		cur_cloud = segmentbycurvature(belly_cloud,150);
		PointT xq, fq;
		double xqdis = 0, fqdis = 0;
		int Isfake;
		Isfake = gettwofinscenter(cur_cloud, xq, fq, xqdis, fqdis);

		cout << xq <<"  "<< fq << endl;
		//确定胸鳍和腹鳍
		double minxd = 999, minfd = 999;
		int xqIndex = -1, fqIndex = -1;
		for (int i = 0; i < facetclusterrgb.size(); i++)
		{
			//最左边的点
			PointT tempcenter;
			double minxtemp = 9999, maxxtemp = -9999;
			for (int j = 0; j < facetclusterrgb[i].indices.size(); j++)
			{
				if (facet_cloud->points[facetclusterrgb[i].indices[j]].x < minxtemp)
				{
					tempcenter = facet_cloud->points[facetclusterrgb[i].indices[j]];
					minxtemp = facet_cloud->points[facetclusterrgb[i].indices[j]].x;
				}
				if (facet_cloud->points[facetclusterrgb[i].indices[j]].x > maxxtemp)
				{
					maxxtemp = facet_cloud->points[facetclusterrgb[i].indices[j]].x;
				}
				//tempcenter.x += facet_cloud->points[facetclusterrgb[i].indices[j]].x;
				//tempcenter.y += facet_cloud->points[facetclusterrgb[i].indices[j]].y;
				//tempcenter.z += facet_cloud->points[facetclusterrgb[i].indices[j]].z;
			}
			//tempcenter.x /= facetclusterrgb[i].indices.size();
			//tempcenter.y /= facetclusterrgb[i].indices.size();
			//tempcenter.z /= facetclusterrgb[i].indices.size();
			double disxtemp = maxxtemp - minxtemp;
			double xd, fd;
			xd = CalTwoPointDistance(xq, tempcenter);
			fd = CalTwoPointDistance(fq, tempcenter);
			if (xd < minxd&&tempcenter.y < xq.y*1.2&&facetclusterrgb[i].indices.size()>100&& disxtemp>=xqdis*0.8)
			{
				minxd = xd;
				xqIndex = i;
				cout <<"xiongqidis: " <<disxtemp << endl;
			}
			if (fd < minfd&&tempcenter.z < fq.z*1.2&&facetclusterrgb[i].indices.size()>100&&disxtemp>=fqdis*0.8)
			{
				minfd = fd;
				fqIndex = i;
			}
		}
		//PointCloudT::Ptr xqi_cloud(new PointCloudT);
		//PointCloudT::Ptr fuqi_cloud(new PointCloudT);
		//for (int i = 0; i < facetclusterrgb[xqIndex].indices.size(); i++)
		//{
			//xqi_cloud->push_back()
		//}
		if (CalTwoPointDistance(xq, fq) < 35)
		{
			if (xq.x < rect.centerpoint.x - full_length / 12)
			{
				PointT fake_s;
				fake_s = xq;
				fake_s.x = xq.x - 5;
				xqi_s = getNearestpoint(cloud, fake_s);
				xqi_e = xqi_s;
				xqi_e.x = xqi_s.x + rect.x / 10;
				xqi_e = getNearestpoint(cloud, xqi_e);
				fuqi_s = xqi_e;
				fuqi_s.x += full_length / 10;
				fuqi_s = getNearestpoint(cloud, fuqi_s);
				fuqi_e = fuqi_s;
				fuqi_e.x += full_length / 10;;
				fuqi_e = getNearestpoint(cloud, fuqi_e);
				cout << "xq fake1" << endl;
			}
		}
		else {
			if (xqIndex != -1 && Isfake == 1)
			{
				//xq
				PointCloudT::Ptr xq_cloud(new PointCloudT);
				double maxxqx = -999;
				for (int i = 0; i < facetclusterrgb[xqIndex].indices.size(); i++)
				{
					xq_cloud->push_back(facet_cloud->points[facetclusterrgb[xqIndex].indices[i]]);
					if (facet_cloud->points[facetclusterrgb[xqIndex].indices[i]].x > maxxqx)
					{
						maxxqx = facet_cloud->points[facetclusterrgb[xqIndex].indices[i]].x;
						xqi_e = facet_cloud->points[facetclusterrgb[xqIndex].indices[i]];
					}
				}
				//_BoundRect xqrect = getBoundRect(xq_cloud);
				//PointT xq_LT;
				//xq_LT.x = xqrect.centerpoint.x - xqrect.x / 2;
				//xq_LT.y = xqrect.centerpoint.y + xqrect.y / 2;
				//xq_LT.z = xqrect.centerpoint.z + xqrect.z / 2;
				double minxqx = 999;
				for (int i = 0; i < xq_cloud->points.size(); i++)
				{
					if (xq_cloud->points[i].x < minxqx)
					{
						minxqx = xq_cloud->points[i].x;
						xqi_s = xq_cloud->points[i];
					}
				}
				//xqi_s = getNearestpoint(xq_cloud, xq_LT);
				PointT temp;
				temp.x = (xqi_s.x + xqi_e.x) / 2;
				temp.y = (xqi_s.y + xqi_e.y) / 2;
				temp.z = (xqi_s.z + xqi_e.z) / 2;
				if (CalTwoPointDistance(temp, xq) > 30)
				{
					PointT fake_s;
					fake_s = xq;
					fake_s.x = xq.x - 5;
					xqi_s = getNearestpoint(cloud, fake_s);
					xqi_e = xqi_s;
					xqi_e.x = xqi_s.x + rect.x / 10;
					xqi_e = getNearestpoint(cloud, xqi_e);
					cout << "xq fake2" << endl;
					//cout << xqi_s << endl;
					//cout << xqi_e << endl;
				}
			}
			else
			{
				PointT fake_s;
				fake_s = xq;
				fake_s.x = xq.x - 5;
				xqi_s = getNearestpoint(cloud, fake_s);
				xqi_e = xqi_s;
				xqi_e.x = xqi_s.x + rect.x / 10;
				xqi_e = getNearestpoint(cloud, xqi_e);
				cout << "xq fakes" << endl;

			}
			//fq
			if (fqIndex != -1 && Isfake == 1)
			{
				PointCloudT::Ptr fq_cloud(new PointCloudT);
				for (int i = 0; i < facetclusterrgb[fqIndex].indices.size(); i++)
				{
					fq_cloud->push_back(facet_cloud->points[facetclusterrgb[fqIndex].indices[i]]);
				}
				_BoundRect fqrect = getBoundRect(fq_cloud);
				PointT fq_LT;
				fq_LT.x = fqrect.centerpoint.x - fqrect.x / 2;
				fq_LT.y = fqrect.centerpoint.y - fqrect.y / 2;
				fq_LT.z = fqrect.centerpoint.z + fqrect.z / 2;
				fuqi_s = getNearestpoint(fq_cloud, fq_LT);
				PointT fq_RT;
				fq_RT.x = fqrect.centerpoint.x + fqrect.x / 2;
				fq_RT.y = fqrect.centerpoint.y - fqrect.y / 2;
				fq_RT.z = fqrect.centerpoint.z + fqrect.z / 2;
				fuqi_e = getNearestpoint(fq_cloud, fq_RT);
				/*double minfqx = 999, maxfqx = -999;
				for (int i = 0; i < fq_cloud->points.size(); i++)
				{
					if (fq_cloud->points[i].x < minfqx)
					{
						minfqx = fq_cloud->points[i].x;
						fuqi_s = fq_cloud->points[i];
					}
					if (fq_cloud->points[i].x > maxfqx)
					{
						maxfqx = fq_cloud->points[i].x;
						fuqi_e = fq_cloud->points[i];
					}
				}*/
				PointT temp;
				temp.x = (fuqi_s.x + fuqi_e.x) / 2;
				temp.y = (fuqi_s.y + fuqi_e.y) / 2;
				temp.z = (fuqi_s.z + fuqi_e.z) / 2;
				if (CalTwoPointDistance(temp, fq) > 40)
				{
					PointT fake_s;
					fake_s = fq;
					fake_s.x = fq.x - 10;
					fuqi_s = getNearestpoint(cloud, fake_s);
					fuqi_e = fuqi_s;
					fuqi_e.x = fuqi_s.x + rect.x / 10;
					fuqi_e = getNearestpoint(cloud, fuqi_e);
					cout << "fuqi fake1" << endl;
				}
			}
			else
			{
				PointT fake_s;
				fake_s = fq;
				fake_s.x = fq.x - 10;
				fuqi_s = getNearestpoint(cloud, fake_s);
				fuqi_e = fuqi_s;
				fuqi_e.x = fuqi_s.x + rect.x / 10;
				fuqi_e = getNearestpoint(cloud, fuqi_e);
				cout << "fuqi fake2" << endl;
			}
		}
		if (xqi_s.x == 0 && xqi_s.y == 0 && xqi_s.z == 0)
		{
			getfakepelvicfin(cloud);
			getfakepectoralfin(cloud);
			cout << " xqi fuqi 0" << endl;
		}
		

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return true;
}

int PreTreatment::getheadpoints(PointCloudT::Ptr cloud)
{
	try
	{
		PointCloudT::Ptr head_cloud(new PointCloudT);
		double headminx = 999;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloud->points[i].x < headminx)
			{
				headminx = cloud->points[i].x;
				head_s = cloud->points[i];
			}
		}
		double head_radius = 0;
		PointT fakehead_e;
		fakehead_e.x = xqi_s.x - 2;
		fakehead_e.y = xqi_s.y;
		fakehead_e.z = xqi_s.z;
		head_radius = CalTwoPointDistance(head_s, fakehead_e);
		double headmaxx = -999;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (CalTwoPointDistance(head_s, cloud->points[i]) < head_radius)
			{
				head_cloud->push_back(cloud->points[i]);
				//if (cloud->points[i].x > headmaxx&&cloud->points[i].z > 0)
				//{
					//headmaxx = cloud->points[i].x;
					//head_e = cloud->points[i];
				//}
			}
		}
		_BoundRect headrect = getBoundRect(head_cloud);
		PointT head_RM;
		head_RM.x = headrect.centerpoint.x + headrect.x / 2;
		head_RM.y = headrect.centerpoint.y;
		head_RM.z = headrect.centerpoint.z + headrect.z / 2;
		//head_e.y = rect.centerpoint.y;
		//head_e.z = rect.centerpoint.z;
		head_e = getNearestpoint(head_cloud, head_RM);
		float headmaxy = -999;
		for (int i = 0; i < head_cloud->points.size(); i++)
		{
			if (head_cloud->points[i].y > headmaxy)
			{
				headmaxy = head_cloud->points[i].y;
				head_top = head_cloud->points[i];
			}
		}

		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

int PreTreatment::getTailHandlepoints(PointCloudT::Ptr cloud)
{
	try
	{
		PointCloudT::Ptr tailhandle_cloud(new PointCloudT);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (cloud->points[i].x > tunqi_s.x + 10 &&cloud->points[i].x < weiqi_s.x)
			{
				tailhandle_cloud->push_back(cloud->points[i]);
			}
		}
		tailhandle_cloud = SortByX(tailhandle_cloud);
		int startx = tailhandle_cloud->points[0].x - 1;
		int endx = tailhandle_cloud->points.back().x + 1;
		int steps = startx;
		double diffy = 999;
		int n = 0;
		while (steps < endx)
		{
			double miny = 999, maxy = -999;
			PointT top_p, bottom_p;
			while (tailhandle_cloud->points[n].x > steps&&tailhandle_cloud->points[n].x < steps + 1)
			{
				if (tailhandle_cloud->points[n].y > maxy)
				{
					maxy = tailhandle_cloud->points[n].y;
					top_p = tailhandle_cloud->points[n];
				}
				if (tailhandle_cloud->points[n].y < miny)
				{
					miny = tailhandle_cloud->points[n].y;
					bottom_p = tailhandle_cloud->points[n];
				}
				n++;
				if (n >= tailhandle_cloud->points.size())
				{
					break;
				}
			}
			if (abs(maxy - miny) < diffy)
			{
				diffy = abs(maxy - miny);
				weib_top = top_p;
				weib_bottom = bottom_p;
			}
			steps += 1;
		}
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 1;
}

int PreTreatment::getfishbody(PointCloudT::Ptr cloud)
{
	_BoundRect beiqi_rect = getBoundRect(beiqi_cloud);
	_BoundRect tunqi_rect = getBoundRect(tunqi_cloud);
	PointCloudT::Ptr body_cloud(new PointCloudT);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x >= beiqi_rect.centerpoint.x - beiqi_rect.x / 2-5 && cloud->points[i].y>=beiqi_rect.centerpoint.y - beiqi_rect.y / 2)
		{
			continue;
		}
		if (cloud->points[i].x > fuqi_s.x)
		{
			continue;
		}
		if (cloud->points[i].z < rect.centerpoint.z - rect.z / 5)
		{
			continue;
		}
		//if (cloud->points[i].x >= weiqi_s.x)
		//{
			//continue;
		//}
		body_cloud->push_back(cloud->points[i]);
	}
	float topy = -999, bottomy = 999;
	for (int i = 0; i < body_cloud->points.size(); i++)
	{
		if (body_cloud->points[i].y > topy)
		{
			topy = body_cloud->points[i].y;
			bei_top = body_cloud->points[i];
		}
		if (body_cloud->points[i].y < bottomy)
		{
			bottomy = body_cloud->points[i].y;
			fu_bottom = body_cloud->points[i];
		}
	}
	return 1;
}

int PreTreatment::getfisheyes(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr head_cloud(new PointCloudT);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x < head_e.x-(head_e.x-head_s.x)/7)
		{
			head_cloud->push_back(cloud->points[i]);
		}
	}
	
	PointCloudT::Ptr head_cloud_c(new PointCloudT);
	head_cloud_c = segmentbycurvature(head_cloud,200);

	//使用kdtree分布判断眼睛 没改完
	//PointCloudT::Ptr cloud_eye(new PointCloudT);
	double avgz = 0;
	for (int i = 0; i < head_cloud_c->points.size(); ++i)
	{
		avgz += head_cloud_c->points[i].z;
	}
	avgz /= head_cloud_c->points.size();
	PointCloudT::Ptr head_cloud_eyes(new PointCloudT);
	for (int i = 0; i < head_cloud_c->points.size(); ++i)
	{
		if (head_cloud_c->points[i].z > avgz*0.9)
		{
			head_cloud_eyes->push_back(head_cloud_c->points[i]);
		}
	}
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_circle(new pcl::PointCloud<PointT>());

	//pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.3);
	seg.setMaxIterations(1000);
	seg.setInputCloud(head_cloud_eyes->makeShared());
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(head_cloud_eyes);
	extract.setIndices(inliers);        //提取内点的索引并存储在其中
	extract.setNegative(false);
	extract.filter(*cloud_circle);
	float mineyex = 999, maxeyex = -999;
	for (int i = 0; i < cloud_circle->points.size(); i++)
	{
		if (cloud_circle->points[i].x < mineyex)
		{
			mineyex = cloud_circle->points[i].x;
			eye_s = cloud_circle->points[i];
		}
		if (cloud_circle->points[i].x > maxeyex)
		{
			maxeyex = cloud_circle->points[i].x;
			eye_e = cloud_circle->points[i];
		}
	}
	//纠正可能出错的眼睛关键点
	if ((eye_s.x + eye_e.x) / 2 < minx + 5 || (eye_s.x + eye_e.x) / 2 > head_e.x - 5 || (eye_s.y + eye_e.y) / 2 < rect.centerpoint.y - 5||eye_s.x<head_s.x+10)
	{
		float head_length = head_e.x - head_s.x;
		eye_s.x = minx + head_length / 3;
		eye_s.y = rect.centerpoint.y + 7;
		eye_s.z = rect.centerpoint.z;
		eye_s = getNearestpoint(head_cloud,eye_s);
		eye_e = eye_s;
		eye_e.x = eye_s.x + 8;
		eye_e = getNearestpoint(head_cloud, eye_e);
		cout << "fake eye!!!" << endl;
	}
	return 1;
}

int PreTreatment::gettwofinscenter(PointCloudT::Ptr cloud, PointT &p1, PointT &p2)
{
	try
	{
		//聚两类
		p1 = cloud->points[0];
		p2 = cloud->points[1];
		vector<PointT> v1, v2;
		PointT oldp1, oldp2;
		double d1 = 99, d2 = 99;
		while (1)
		{
			for (int i = 0; i < cloud->points.size(); i++)
			{
				float d1 = CalTwoPointDistance(cloud->points[i], p1);
				float d2 = CalTwoPointDistance(cloud->points[i], p2);
				if (d1 < d2)
				{
					v1.push_back(cloud->points[i]);
				}
				else
				{
					v2.push_back(cloud->points[i]);
				}
			}
			oldp1 = p1;
			oldp2 = p2;
			//新的中心点
			p1.x = 0;
			p1.y = 0;
			p1.z = 0;
			for (int i = 0; i < v1.size(); i++)
			{
				p1.x += v1[i].x;
				p1.y += v1[i].y;
				p1.z += v1[i].z;
			}
			p1.x /= v1.size();
			p1.y /= v1.size();
			p1.z /= v1.size();
			p2.x = 0;
			p2.y = 0;
			p2.z = 0;
			for (int i = 0; i < v2.size(); i++)
			{
				p2.x += v2[i].x;
				p2.y += v2[i].y;
				p2.z += v2[i].z;
			}
			p2.x /= v2.size();
			p2.y /= v2.size();
			p2.z /= v2.size();
			d1 = abs(p1.x - oldp1.x) + abs(p1.y - oldp1.y) + abs(p1.z - oldp1.z);
			d2 = abs(p2.x - oldp2.x) + abs(p2.y - oldp2.y) + abs(p2.z - oldp2.z);
			if (d1 < 1 && d2 < 1)
			{
				break;
			}
			v1.clear();
			v2.clear();
		}
		
		PointCloudT::Ptr cloud1(new PointCloudT);
		//找最左边的点
		double minx1 = 9999, minx2 = 9999;
		for (int i = 0; i < v1.size(); i++)
		{
			cloud1->push_back(v1[i]);
			if (v1[i].x < minx1)
			{
				p1 = v1[i];
				minx1 = v1[i].x;
			}
		}
		PointCloudT::Ptr cloud2(new PointCloudT);
		for (int i = 0; i < v2.size(); i++)
		{
			cloud2->push_back(v2[i]);
			if (v2[i].x < minx2)
			{
				p2 = v2[i];
				minx2 = v2[i].x;
			}
		}
		if (p1.x > p2.x)
		{
			swap(p1, p2);
		}
		_BoundRect rect1, rect2;
		rect1 = getBoundRect(cloud1);
		rect2 = getBoundRect(cloud2);
		if (rect1.y < 15 && rect2.y < 15)
		{
			return 1;
		}
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 0;
}

int PreTreatment::gettwofinscenter(PointCloudT::Ptr cloud, PointT &p1, PointT &p2, double &p1Length, double &p2Length)
{
	try
	{
		//聚两类
		p1 = cloud->points[0];
		p2 = cloud->points[1];
		vector<PointT> v1, v2;
		PointT oldp1, oldp2;
		double d1 = 99, d2 = 99;
		while (1)
		{
			for (int i = 0; i < cloud->points.size(); i++)
			{
				float d1 = CalTwoPointDistance(cloud->points[i], p1);
				float d2 = CalTwoPointDistance(cloud->points[i], p2);
				if (d1 < d2)
				{
					v1.push_back(cloud->points[i]);
				}
				else
				{
					v2.push_back(cloud->points[i]);
				}
			}
			oldp1 = p1;
			oldp2 = p2;
			//新的中心点
			p1.x = 0;
			p1.y = 0;
			p1.z = 0;
			for (int i = 0; i < v1.size(); i++)
			{
				p1.x += v1[i].x;
				p1.y += v1[i].y;
				p1.z += v1[i].z;
			}
			p1.x /= v1.size();
			p1.y /= v1.size();
			p1.z /= v1.size();
			p2.x = 0;
			p2.y = 0;
			p2.z = 0;
			for (int i = 0; i < v2.size(); i++)
			{
				p2.x += v2[i].x;
				p2.y += v2[i].y;
				p2.z += v2[i].z;
			}
			p2.x /= v2.size();
			p2.y /= v2.size();
			p2.z /= v2.size();
			d1 = abs(p1.x - oldp1.x) + abs(p1.y - oldp1.y) + abs(p1.z - oldp1.z);
			d2 = abs(p2.x - oldp2.x) + abs(p2.y - oldp2.y) + abs(p2.z - oldp2.z);
			if (d1 < 1 && d2 < 1)
			{
				break;
			}
			v1.clear();
			v2.clear();
		}

		PointCloudT::Ptr cloud1(new PointCloudT);
		//找最左边的点
		double minx1 = 9999, minx2 = 9999;
		double maxx1 = -9999, maxx2 = -9999;
		for (int i = 0; i < v1.size(); i++)
		{
			cloud1->push_back(v1[i]);
			if (v1[i].x < minx1)
			{
				p1 = v1[i];
				minx1 = v1[i].x;
			}
			if (v1[i].x > maxx1)
			{
				maxx1 = v1[i].x;
			}
		}
		PointCloudT::Ptr cloud2(new PointCloudT);
		for (int i = 0; i < v2.size(); i++)
		{
			cloud2->push_back(v2[i]);
			if (v2[i].x < minx2)
			{
				p2 = v2[i];
				minx2 = v2[i].x;
			}
			if (v2[i].x > maxx2)
			{
				maxx2 = v2[i].x;
			}
		}
		p1Length = maxx1 - minx1;
		p2Length = maxx2 - minx2;
		if (p1.x > p2.x)
		{
			swap(p1, p2);
			swap(p1Length, p2Length);
		}
		_BoundRect rect1, rect2;
		rect1 = getBoundRect(cloud1);
		rect2 = getBoundRect(cloud2);
		if (rect1.y < 15 && rect2.y < 15)
		{
			return 1;
		}
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return 0;
}

int PreTreatment::getfakekeypoints(PointCloudT::Ptr cloud)
{
	getfakedorsalfin(cloud);
	getfakeanalfin(cloud);
	getfaketailfin(cloud);
	getfakepectoralfin(cloud);
	getfakepelvicfin(cloud);
	getheadpoints(cloud);
	getTailHandlepoints(cloud);
	cout << "all fake" << endl;
	return 1;
}

int PreTreatment::getfakedorsalfin(PointCloudT::Ptr cloud)
{
	PointT fake_s;
	fake_s.x = rect.centerpoint.x;
	fake_s.y = rect.centerpoint.y + rect.y / 3;
	fake_s.z = rect.centerpoint.z;
	fake_s = getNearestpoint(cloud, fake_s);
	beiqi_s = fake_s;
	PointT fake_e;
	fake_e = fake_s;
	fake_e.x += full_length / 15;
	fake_e = getNearestpoint(cloud, fake_e);
	beiqi_e = fake_e;
	return 1;
}

int PreTreatment::getfakeanalfin(PointCloudT::Ptr cloud)
{
	PointT fake_s;
	PointT fake_e;
	fake_s.x = rect.centerpoint.x + rect.x / 6.5;
	fake_s.y = rect.centerpoint.y - rect.y / 2;
	fake_s.z = rect.centerpoint.z;
	fake_s = getNearestpoint(cloud, fake_s);
	tunqi_s = fake_s;
	fake_e = fake_s;
	fake_e.x += full_length / 15;
	fake_e = getNearestpoint(cloud, fake_e);
	tunqi_e = fake_e;
	return 1;
}

int PreTreatment::getfaketailfin(PointCloudT::Ptr cloud)
{
	weiqi_s.x = maxx - full_length / 7;
	weiqi_s.y = 0;
	weiqi_s.z = 0;
	weiqi_e.x = maxx;
	weiqi_e.y = 0;
	weiqi_e.z = 0;
	weiqi_s = getNearestpoint(cloud, weiqi_s);
	weiqi_e = getNearestpoint(cloud, weiqi_e);
	return 1;
}

int PreTreatment::getfakepectoralfin(PointCloudT::Ptr cloud)
{
	xqi_e.x = rect.centerpoint.x - rect.x / 4;
	xqi_e.y = rect.centerpoint.y - rect.y / 4;
	xqi_e.z = rect.centerpoint.z + rect.z / 4;
	xqi_e = getNearestpoint(cloud, xqi_e);
	xqi_s.x = xqi_e.x - rect.x / 7;
	xqi_s.y = xqi_e.y + rect.y / 8;
	xqi_s.z = xqi_e.z;
	xqi_s = getNearestpoint(cloud, xqi_s);
	return 1;
}

int PreTreatment::getfakepelvicfin(PointCloudT::Ptr cloud)
{
	fuqi_s.x = rect.centerpoint.x - rect.x / 16;
	fuqi_s.y = rect.centerpoint.y - rect.y * 3 / 8;
	fuqi_s.z = rect.centerpoint.z;
	fuqi_e.x = fuqi_s.x + rect.x / 8;
	fuqi_e.y = fuqi_s.y;
	fuqi_e.z = fuqi_s.z;
	return 1;
}

bool PreTreatment::RegionGrowingRGBSeg(PointCloudT::Ptr cloud, PointCloudT::Ptr &colored_cloud, vector<pcl::PointIndices> &clusters)
{
	try
	{
		pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
		normal_estimator.compute(*normals);
		pcl::IndicesPtr indices(new std::vector <int>);
		//pcl::PassThrough<PointT> pass;
		//pass.setInputCloud(cloud);
		//pass.setFilterFieldName("z");
		//pass.setFilterLimits(0.0, 10);
		//pass.filter(*indices);
		//用于存放点云团的容器
		//std::vector <pcl::PointIndices> clusters;
		//颜色分割器
		pcl::RegionGrowingRGB<PointT> reg;
		reg.setInputCloud(cloud);
		//点云经过了滤波器的预处理，提取了indices
		//reg.setIndices(indices);//设置输入点云的索引
		reg.setSearchMethod(tree);//设置点云的搜索机制
		reg.setDistanceThreshold(1.5);//距离阈值，判断两点是否相邻
									//点与点之间颜色容差
		reg.setPointColorThreshold(5);//两点颜色阈值，是否属于一类
		reg.setRegionColorThreshold(5);//设置两类区域区域颜色阈值，用于判断两类区域是否聚类合并
		reg.setMinClusterSize(50);//设置一个聚类的最少点数目为600
		reg.setSmoothModeFlag(true);
		reg.setCurvatureTestFlag(true);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(2.0 / 180.0*M_PI);
		reg.setCurvatureThreshold(1.0);
		reg.extract(clusters);
		colored_cloud = reg.getColoredCloudRGBA();
		if (colored_cloud->empty())
		{
			cout << "rgb error" << endl;
			return false;
		}
		cout << "rgb success" << endl;
		
	}
	catch (exception e)
	{
		cout << e.what() << endl;
	}
	return true;
}

pcl::PointXYZRGB PreTreatment::RestorePoint(pcl::PointXYZRGB point)
{
	pcl::PointXYZRGB resultpoint;
	resultpoint.x = point.x*reduction(0, 0) + point.y*reduction(1, 0) + point.z*reduction(2, 0);
	resultpoint.y = point.x*reduction(0, 1) + point.y*reduction(1, 1) + point.z*reduction(2, 1);
	resultpoint.z = point.x*reduction(0, 2) + point.y*reduction(1, 2) + point.z*reduction(2, 2);
	resultpoint.x += reduction(0, 3);
	resultpoint.y += reduction(1, 3);
	resultpoint.z += reduction(2, 3);
	resultpoint.rgb = point.rgb;
	return resultpoint;
}

map<string, pcl::PointXYZRGB> PreTreatment::RestoreAllPoints(map<string, pcl::PointXYZRGB> m)
{
	map<string, pcl::PointXYZRGB> result;
	map<string, pcl::PointXYZRGB>::iterator it;
	for (it = m.begin(); it != m.end(); it++)
	{
		string s = it->first;
		pcl::PointXYZRGB p = RestorePoint(it->second);
		result.insert(pair<string, pcl::PointXYZRGB>(s, p));
	}
	return result;
}

int PreTreatment::gettail(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_tail(new PointCloudT);
	PointCloudT::Ptr cloud_reg(new PointCloudT);
	vector<pcl::PointIndices> clusters;
	cloud_reg = regiongrowingseg(cloud, clusters);
	float maxtailx = -999;
	int tailIndex = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		float tail_avgx = 0;
		for (int j = 0; j < clusters[i].indices.size(); j++)
		{
			tail_avgx += cloud->points[clusters[i].indices[j]].x;
		}
		tail_avgx /= clusters[i].indices.size();
		if (tail_avgx - minx > maxtailx&&clusters[i].indices.size() > 100)
		{
			maxtailx = tail_avgx - minx;
			tailIndex = i;
		}
	}
	int tailxs = 999;
	int tailxe = -999;
	for (int i = 0; i < clusters[tailIndex].indices.size(); i++)
	{
		cloud_tail->push_back(cloud->points[clusters[tailIndex].indices[i]]);
		if (cloud->points[clusters[tailIndex].indices[i]].x < tailxs)
		{
			tailxs = cloud->points[clusters[tailIndex].indices[i]].x;
		}
		if (cloud->points[clusters[tailIndex].indices[i]].x > tailxe)
		{
			tailxe = cloud->points[clusters[tailIndex].indices[i]].x;
		}
	}
	cloud_tail = SortByX(cloud_tail);
	vector<float> diffy(tailxe+1);
	int n = 0;
	int step = tailxs;
	while(step<tailxe)
	{
		float maxy = -999, miny = 999;
		while (cloud_tail->points[n].x >= step&&cloud_tail->points[n].x < step+3)
		{
			if (cloud_tail->points[n].y > maxy)
			{
				maxy = cloud_tail->points[n].y;
			}
			if (cloud_tail->points[n].y < miny)
			{
				miny = cloud_tail->points[n].y;
			}
			n++;
			if (n >= cloud_tail->points.size())
			{
				break;
			}
		}
		diffy[step] = abs(maxy - miny);
		step+=3;
	}
	vector<float> dy(tailxe + 1);
	for (int i = tailxs; i < tailxe; i+=3)
	{
		dy[i] = diffy[i + 3] - diffy[i];
	}
	for (int i = tailxs; i < tailxe; i+=3)
	{
		if (dy[i + 3] - dy[i] > 4)
		{
			tailxs = i+6;
			break;
		}
		//cout << i << ": " << dy[i] << endl;
	}
	cout << tailxs << endl;
	PointCloudT::Ptr real_tail(new PointCloudT);
	for (int i = 0; i < cloud_tail->points.size(); i++)
	{
		if (cloud_tail->points[i].x > tailxs)
		{
			real_tail->push_back(cloud_tail->points[i]);
		}
	}
	cout << real_tail->points.size() << endl;
	//clusters.clear();
	
	_BoundRect tailrect;
	tailrect = getBoundRect(real_tail);
	PointT tail_LM;
	tail_LM.x = tailrect.centerpoint.x - tailrect.x / 2;
	tail_LM.y = tailrect.centerpoint.y;
	tail_LM.z = tailrect.centerpoint.z + tailrect.z / 2;
	weiqi_s = getNearestpoint(real_tail, tail_LM);
	if (weiqi_s.x < tunqi_e.x)
	{
		weiqi_s.x = weiqi_e.x - full_length / 7;
		weiqi_s.y = 0;
		weiqi_s.z = 0;
		weiqi_s = getNearestpoint(cloud, weiqi_s);
	}
	return 1;
}

PointCloudT::Ptr PreTreatment::getfullfish(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr plane_cloud(new PointCloudT);
	Vector4f abc;
	Eigen::Matrix3f axismat;
	axismat << 0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	Eigen::Matrix3f rotmat;
	rotmat = rot.transpose().inverse()*axismat;
	abc(3) = plane_coeff[3] + plane_coeff[0] * avg(0, 0) + plane_coeff[1] * avg(0, 1) + plane_coeff[2] * avg(0, 2);
	abc(0) = plane_coeff[0] * rotmat(0, 0) + plane_coeff[1] * rotmat(1, 0) + plane_coeff[2] * rotmat(2, 0);
	abc(1) = plane_coeff[0] * rotmat(0, 1) + plane_coeff[1] * rotmat(1, 1) + plane_coeff[2] * rotmat(2, 1);
	abc(2) = plane_coeff[0] * rotmat(0, 2) + plane_coeff[1] * rotmat(1, 2) + plane_coeff[2] * rotmat(2, 2);
	if (ismirr)
	{
		abc(0) = abc(0);
		abc(1) = abc(1);
		abc(2) = -abc(2);
	}
	if (isturnx)
	{
		abc(0) = abc(0);
		abc(1) = -abc(1);
		abc(2) = -abc(2);
		//cout << "transform_z success" << endl;
	}
	if (isturnz)
	{
		abc(0) = -abc(0);
		abc(1) = -abc(1);
		abc(2) = abc(2);
		//cout << "transform_x success" << endl;
	}
	//draw plane
	for (int i = -150; i < 150; ++i)
	{
		for (int j = -150; j < 150; ++j)
		{
			PointT tempPoint;
			tempPoint.x = i;
			tempPoint.y = j;
			tempPoint.z = -(abc(0)*i + abc(1)*j + abc(3) / abc(2));
			plane_cloud->push_back(tempPoint);
		}
	}

	PointCloudT::Ptr cloud_sortx = SortByX(cloud);
	int steps = minx - 1;
	int n = 0;
	PointCloudT::Ptr cloud_mirr(new PointCloudT);
	while (steps < maxx)
	{
		vector<PointT> stepPoints;
		while (cloud_sortx->points[n].x >= steps&&cloud_sortx->points[n].x <= steps + 1)
		{
			stepPoints.push_back(cloud_sortx->points[n]);
			n++;
			if (n >= cloud_sortx->points.size())
			{
				break;
			}
		}
		//cout << n << endl;
		//cout << "stepPoints.size" << endl;
		//cout << stepPoints.size() << endl;
		//对y排序
		for (int i = 0; i < stepPoints.size(); i++)
		{
			for (int j = 0; j < stepPoints.size() - 1 - i; ++j)
			{
				if (stepPoints[j].y > stepPoints[j + 1].y)
				{
					swap(stepPoints[j], stepPoints[j + 1]);
				}
			}
		}
		//求两边缘的点集
		vector<PointT> minyPoints;
		vector<PointT> maxyPoints;
		for (int i = 0; i < stepPoints.size(); i++)
		{
			if (stepPoints[i].y >= stepPoints[0].y&&stepPoints[i].y <= stepPoints[0].y + 1)
			{
				minyPoints.push_back(stepPoints[i]);
			}
			if (stepPoints[i].y >= stepPoints[stepPoints.size()-1].y - 1 && stepPoints[i].y <= stepPoints[stepPoints.size()-1].y)
			{
				maxyPoints.push_back(stepPoints[i]);
			}
		}
		//求两边缘的最高点
		PointT maxzpoint1, maxzpoint2;
		int maxz = -999;
		for (int i = 0; i < minyPoints.size(); i++)
		{
			if (minyPoints[i].z > maxz)
			{
				maxz = minyPoints[i].z;
				maxzpoint1 = minyPoints[i];
			}
		}
		maxz = -999;
		for (int i = 0; i < maxyPoints.size(); i++)
		{
			if (maxyPoints[i].z > maxz)
			{
				maxz = maxyPoints[i].z;
				maxzpoint2 = maxyPoints[i];
			}
		}
		maxzpoint1.x = steps + 1;
		maxzpoint2.x = steps + 1;
		//求和平面的交点
		Vector3d mirrpoint1, mirrpoint2;
		double t;
		Vector3d p1;
		Vector3d p2;
		p1 << maxzpoint1.x, maxzpoint1.y, maxzpoint1.z;
		p2 << maxzpoint2.x, maxzpoint2.y, maxzpoint2.z;
		//cout << "p1 p2" << endl;
		//cout << p1 << endl << p2 << endl;
		Vector3d p0;//平面随机一点
		p0 << 0, 0, -abc(3) / abc(2);
		Vector3d n;
		n << abc(0), abc(1), abc(2);
		t = (p0 - p1).dot(n) / n.dot(n);
		mirrpoint1 = t*n + p1;
		t = (p0 - p2).dot(n) / n.dot(n);
		mirrpoint2 = t*n + p2;
		mirrpoint1 = (p1 + mirrpoint1) / 2;
		mirrpoint2 = (p2 + mirrpoint2) / 2;
		Vector3d sf_normal;//对称面的法向量
		sf_normal << 0, mirrpoint2(2) - mirrpoint1(2), mirrpoint1(1) - mirrpoint2(1);
		//cout << "mirrpoint" << endl;
		//cout << mirrpoint1 << endl << mirrpoint2 << endl;
		//cout << sf_normal << endl;
		for (int i = 0; i < stepPoints.size(); i++)
		{
			if (steps < weiqi_s.x)
			{
				if ((stepPoints[i].z > mirrpoint1(2) && i <= stepPoints.size() / 2) || (stepPoints[i].z > mirrpoint2(2) && i >= stepPoints.size() / 2))
				{
					cloud_mirr->push_back(stepPoints[i]);
					Vector3d pt;
					pt << stepPoints[i].x, stepPoints[i].y, stepPoints[i].z;
					t = (mirrpoint1 - pt).dot(sf_normal) / sf_normal.dot(sf_normal);
					Vector3d pt_;
					pt_ = 2 * t*sf_normal + pt;
					PointT mirrpoint;
					mirrpoint.x = pt_(0);
					mirrpoint.y = pt_(1);
					mirrpoint.z = pt_(2);
					mirrpoint.r = 0;
					mirrpoint.g = 0;
					mirrpoint.b = 255;
					cloud_mirr->push_back(mirrpoint);
				}
			}
			else
			{
				cloud_mirr->push_back(stepPoints[i]);
				Vector3d pt;
				pt << stepPoints[i].x, stepPoints[i].y, stepPoints[i].z;
				t = (mirrpoint1 - pt).dot(sf_normal) / sf_normal.dot(sf_normal);
				Vector3d pt_;
				pt_ = 2 * t*sf_normal + pt;
				PointT mirrpoint;
				mirrpoint.x = pt_(0);
				mirrpoint.y = pt_(1);
				mirrpoint.z = pt_(2);
				mirrpoint.r = 0;
				mirrpoint.g = 0;
				mirrpoint.b = 255;
				cloud_mirr->push_back(mirrpoint);
			}
		}
		steps += 1;
	}
	return cloud_mirr;
}

PointCloudT::Ptr PreTreatment::getfullfishbyfacet(PointCloudT::Ptr cloud)
{
	//overseg
	if (cloud->empty())
	{
		cout << "facet inputcloud empty" << endl;
		return false;
	}
	float voxel_resolution = 0.8f;
	float seed_resolution = 6.0f;
	float color_importance = 0.1f;
	float spatial_importance = 0.4f;
	float normal_importance = 1.0f;
	//生成结晶器
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(false);
	//输入点云和设置参数
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	super.extract(supervoxel_clusters);
	cout << "super " << supervoxel_clusters.size() << endl;
	if (supervoxel_clusters.size() == 0)
	{
		cout << "supervoxel error size=0" << endl;
		return false;
	}
	//std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	//super.getSupervoxelAdjacency(supervoxel_adjacency);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;

	Vector4f abc;
	Eigen::Matrix3f axismat;
	axismat << 0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	Eigen::Matrix3f rotmat;
	rotmat = rot.transpose().inverse()*axismat;
	abc(3) = plane_coeff[3] + plane_coeff[0] * avg(0, 0) + plane_coeff[1] * avg(0, 1) + plane_coeff[2] * avg(0, 2);
	abc(0) = plane_coeff[0] * rotmat(0, 0) + plane_coeff[1] * rotmat(1, 0) + plane_coeff[2] * rotmat(2, 0);
	abc(1) = plane_coeff[0] * rotmat(0, 1) + plane_coeff[1] * rotmat(1, 1) + plane_coeff[2] * rotmat(2, 1);
	abc(2) = plane_coeff[0] * rotmat(0, 2) + plane_coeff[1] * rotmat(1, 2) + plane_coeff[2] * rotmat(2, 2);
	if (isturnx)
	{
		abc(0) = abc(0);
		abc(1) = -abc(1);
		abc(2) = -abc(2);
		//cout << "transform_z success" << endl;
	}
	if (isturnz)
	{
		abc(0) = -abc(0);
		abc(1) = -abc(1);
		abc(2) = abc(2);
		//cout << "transform_x success" << endl;
	}
	PointCloudT::Ptr cloud_mirr(new PointCloudT);
	for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
	{
		//直线参数
		Vector3d p1;
		p1 << it_clusters->second->centroid_.x, it_clusters->second->centroid_.y, it_clusters->second->centroid_.z;
		Vector3d u;
		u << it_clusters->second->normal_.normal_x, it_clusters->second->normal_.normal_y, it_clusters->second->normal_.normal_z;
		Vector3d p0;//平面随机一点
		p0 << 0, 0, -abc(3) / abc(2);
		Vector3d n;
		n << abc(0), abc(1), abc(2);
		double t;
		t = (p0 - p1).dot(n) / u.dot(n);
		Vector3d mirrpoint;
		mirrpoint = (p1 + t*u + p1) / 2;
		for (int i = 0; i < it_clusters->second->voxels_->points.size(); i++)
		{
			cloud_mirr->push_back(it_clusters->second->voxels_->points[i]);
			Vector3d pt;
			pt << it_clusters->second->voxels_->points[i].x, it_clusters->second->voxels_->points[i].y, it_clusters->second->voxels_->points[i].z;
			Vector3d sf_normal = n;
			Vector3d pt_;
			t = (mirrpoint - pt).dot(sf_normal) / sf_normal.dot(sf_normal);
			pt_ = 2 * t*u + pt;
			PointT pointm;
			pointm.x = pt_(0);
			pointm.y = pt_(1);
			pointm.z = pt_(2);
			pointm.r = 255;
			pointm.g = 255;
			pointm.b = 255;
			cloud_mirr->push_back(pointm);
		}
	}
	cout << cloud_mirr->points.size() << endl;
	return cloud_mirr;
}

PointCloudT::Ptr PreTreatment::getfishbodyformesh(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_body(new PointCloudT);
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x < weiqi_s.x)
		{
			cloud_body->push_back(cloud->points[i]);
		}
	}
	return cloud_body;
}

void PreTreatment::Triagulation(PointCloudT::Ptr cloud)
{
	PointCloudXYZ::Ptr cloudxyz(new PointCloudXYZ);
	cloud = DownSample(cloud);
	RGBA2XYZ(cloud, cloudxyz);
	pcl::NormalEstimation<Pointxyz, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<Pointxyz>::Ptr tree(new pcl::search::KdTree<Pointxyz>);
	tree->setInputCloud(cloudxyz);
	n.setInputCloud(cloudxyz);
	n.setSearchMethod(tree);
	n.setKSearch(30);
	n.compute(*normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudxyz, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云,只能用pointcloudxyz
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);//利用有向点云构造tree
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	pcl::PolygonMesh triangles;//存储最终三角化的网络模型
	gp3.setSearchRadius(10);
	gp3.setMu(2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(300);//设置样本点最多可以搜索的邻域数目100 。
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	gp3.setMinimumAngle(M_PI / 18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	gp3.setMaximumAngle(2 * M_PI / 3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。
	
	gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云
	gp3.setSearchMethod(tree2);           //设置搜索方式tree2
	gp3.reconstruct(triangles);           //重建提取三角化
	pcl::io::savePLYFile("outputtest/3D/trimesh.ply", triangles);

	cout << "mls success" << endl;
}

void PreTreatment::RGBA2XYZ(PointCloudT::Ptr cloudrgb, PointCloudXYZ::Ptr &cloudxyz)
{
	cloudxyz->clear();
	for (int i = 0; i < cloudrgb->points.size(); i++)
	{
		Pointxyz point;
		point.x = cloudrgb->points[i].x;
		point.y = cloudrgb->points[i].y;
		point.z = cloudrgb->points[i].z;
		cloudxyz->push_back(point);
	}
}

void PreTreatment::test()
{
	PointCloudT::Ptr rgbcloud(new PointCloudT);
	if (pcl::io::loadPCDFile<PointT>("outputtest/3D/cloud_full.pcd", *rgbcloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return ;
	}
	Triagulation(rgbcloud);
	cout << surface_area << endl;
}

pcl::PolygonMesh PreTreatment::PoissonRemesh(PointCloudT::Ptr cloud)
{
	PointCloudXYZ::Ptr cloudxyz(new PointCloudXYZ);

	RGBA2XYZ(cloud, cloudxyz);

	
	// 计算法向量
	pcl::NormalEstimation<Pointxyz, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
	pcl::search::KdTree<Pointxyz>::Ptr tree(new pcl::search::KdTree<Pointxyz>);
	tree->setInputCloud(cloudxyz);
	n.setInputCloud(cloudxyz);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //计算法线，结果存储在normals中

						 //将点云和法线放到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudxyz, *normals, *cloud_with_normals);

	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//创建Poisson对象，并设置参数
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setConfidence(true); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
						   //pn.setIndices();

						   //设置搜索方法和输入点云
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	//创建多变形网格，用于存储结果
	pcl::PolygonMesh mesh;
	//执行重构
	pn.performReconstruction(mesh);

	//保存网格图
	pcl::io::savePLYFile("outputtest/3D/possion.ply", mesh);
	cout << "possion" << endl;
	return mesh;
}

double PreTreatment::getVolume(PointCloudT::Ptr cloud)
{
	//cout << "volume: " << cloud->points.size() << endl;
	double vol;
	pcl::PolygonMesh mesh = PoissonRemesh(cloud);
	//Triagulation(cloud_fullfish);
	vtkSmartPointer<vtkPolyData> vtkmesh = vtkSmartPointer<vtkPolyData>::New();
	pcl::VTKUtils::mesh2vtk(mesh, vtkmesh);
	vtkSmartPointer<vtkTriangleFilter> triFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	triFilter->SetInputData(vtkmesh);
	triFilter->Update();
	vtkSmartPointer<vtkMassProperties> massProp = vtkSmartPointer<vtkMassProperties>::New();
	massProp->SetInputData(triFilter->GetOutput());
	vol = massProp->GetVolume();
	return vol;
	//surface_area = massProp->GetSurfaceArea();
	//cout << volume << ' ' << surface_area << endl;
}

void PreTreatment::getSurfaceArea(PointCloudT::Ptr cloud)
{

	//cout << cloud->points.size() << endl;
	pcl::PolygonMesh mesh = PoissonRemesh(cloud);
	//Triagulation(cloud_fullfish);
	vtkSmartPointer<vtkPolyData> vtkmesh = vtkSmartPointer<vtkPolyData>::New();
	pcl::VTKUtils::mesh2vtk(mesh, vtkmesh);
	vtkSmartPointer<vtkTriangleFilter> triFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	triFilter->SetInputData(vtkmesh);
	triFilter->Update();
	vtkSmartPointer<vtkMassProperties> massProp = vtkSmartPointer<vtkMassProperties>::New();
	massProp->SetInputData(triFilter->GetOutput());
	//volume = massProp->GetVolume();
	surface_area = massProp->GetSurfaceArea();
	//cout << volume << ' ' << surface_area << endl;
}

void PreTreatment::getPhenotype()
{
	full_length = weiqi_e.x - head_s.x;
	body_length = weiqi_s.x - head_s.x;
	head_length = head_e.x - head_s.x;
	body_height = bei_top.y - fu_bottom.y;
	caudal_peduncle_height = weib_top.y - weib_bottom.y;
	caudal_peduncle_length = weiqi_s.x - tunqi_e.x;
	dorsal_proboscis_dis = beiqi_s.x - head_s.x;
	proboscis_length = eye_s.x - head_s.x;
	eye_diameter = eye_e.x - eye_s.x;
	head_behind_eye_length = head_e.x - eye_e.x;
	anal_pelvic_dis = tunqi_s.x - fuqi_s.x;
	dorsal_length = CalTwoPointDistance(beiqi_s, beiqi_e);
	pectoral_length = CalTwoPointDistance(xqi_s, xqi_e);
	pelvic_length = CalTwoPointDistance(fuqi_s, fuqi_e);
	anal_length = CalTwoPointDistance(tunqi_s, tunqi_e);
	tail_length = weiqi_e.x - weiqi_s.x;
	dorsal_tail_dis = weiqi_s.x - beiqi_s.x;
}

double PreTreatment::getfull_length()
{
	
	return full_length;
}

double PreTreatment::getbody_length()
{
	
	return body_length;
}

double PreTreatment::gethead_length()
{
	
	return head_length;
}

double PreTreatment::getbody_height()
{
	
	return body_height;
}

double PreTreatment::getcaudal_peduncle_height()
{
	
	return caudal_peduncle_height;
}

double PreTreatment::getcaudal_peduncle_length()
{
	
	return caudal_peduncle_length;
}

double PreTreatment::getdorsal_proboscis_dis()
{
	
	return dorsal_proboscis_dis;
}

double PreTreatment::getproboscis_length()
{
	
	return proboscis_length;
}

double PreTreatment::geteye_diameter()
{
	
	return eye_diameter;
}

double PreTreatment::gethead_behind_eye_length()
{
	
	return head_behind_eye_length;
}

double PreTreatment::getanal_pelvic_dis()
{
	
	return anal_pelvic_dis;
}

double PreTreatment::getdorsal_length()
{
	
	return dorsal_length;
}

double PreTreatment::getpectoral_length()
{
	
	return pectoral_length;
}

double PreTreatment::getpelvic_length()
{
	
	return pelvic_length;
}

double PreTreatment::getanal_length()
{
	
	return anal_length;
}

double PreTreatment::gettail_length()
{
	
	return tail_length;
}

double PreTreatment::getdorsal_tail_dis()
{
	
	return dorsal_tail_dis;
}

double PreTreatment::getsurface_area()
{
	return surface_area;
}

//double PreTreatment::getvolume()
//{
	//return volume;
//}

void PreTreatment::InsertData()
{
	string sql = "insert into fish values (\"" +
		fishname + "\"," + to_string(full_length) + ',' + to_string(body_length) + ',' + to_string(head_length) + ',' + to_string(body_height) + ',' + to_string(caudal_peduncle_height) + ',' +
		to_string(caudal_peduncle_length) + ',' + to_string(dorsal_proboscis_dis) + ',' + to_string(proboscis_length) + ',' + to_string(eye_diameter) + ',' + to_string(head_behind_eye_length) + ',' + to_string(anal_pelvic_dis) + ',' + to_string(dorsal_length) + ',' +
		to_string(pectoral_length) + ',' + to_string(pelvic_length) + ',' + to_string(anal_length) + ',' + to_string(tail_length) + ',' + to_string(dorsal_tail_dis) + ',' + to_string(surface_area) + ',' + to_string(volume) + ',' +
		"NULL)";
	FishSql fish;
	fish.ConnectDatabase("***", "***", "*****", "***");
	fish.InsertData(sql.c_str());
	fish.FreeConnect();
}

int PreTreatment::CPCSeg(PointCloudT::Ptr cloud)
{
	clock_t start, end;
	start = clock();
	PointCloudT::Ptr smallcloud(new PointCloudT);
	smallcloud = PointCloudScale(0.01, cloud);
	float smoothness_threshold = 0.1;
	float voxel_resolution = 0.008f;
	float seed_resolution = 0.06f;
	float color_importance = 0.1f;
	float spatial_importance = 0.4f;
	float normal_importance = 1.0f;
	//cout << smallcloud->points.size() << endl;
	//生成结晶器
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(false);
	//输入点云和设置参数
	super.setInputCloud(smallcloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	super.extract(supervoxel_clusters);
	cout << "super " << supervoxel_clusters.size() << endl;
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	//cpc
	pcl::CPCSegmentation<PointT> cpcseg;
	cpcseg.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	using SegLabel2ClusterMap = std::map<std::uint32_t, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>;
	SegLabel2ClusterMap seg_to_edge_points_map;
	std::map<uint32_t, std::set<uint32_t> > segment_adjacency_map;
	cpcseg.getSegmentAdjacencyMap(segment_adjacency_map);
	cpcseg.setCutting(25, 400, 0.16, true, true, true);
	cpcseg.setRANSACIterations(10000);
	cpcseg.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	cpcseg.segment();
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared();
	cpcseg.relabelCloud(*cpc_labeled_cloud);
	pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList sv_adjacency_list;
	cpcseg.getSVAdjacencyList(sv_adjacency_list);
	cout << cpc_labeled_cloud->points.size() << endl;
	int label_max = 0;
	for (int i = 0; i< cpc_labeled_cloud->size(); i++) {
		if (cpc_labeled_cloud->points[i].label>label_max)
			label_max = cpc_labeled_cloud->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
	ColoredCloud1->height = 1;
	ColoredCloud1->width = cpc_labeled_cloud->size();
	ColoredCloud1->resize(cpc_labeled_cloud->size());
	for (int i = 0; i < label_max; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		for (int j = 0; j < cpc_labeled_cloud->size(); j++) {
			if (cpc_labeled_cloud->points[j].label == i) {
				ColoredCloud1->points[j].x = cpc_labeled_cloud->points[j].x;
				ColoredCloud1->points[j].y = cpc_labeled_cloud->points[j].y;
				ColoredCloud1->points[j].z = cpc_labeled_cloud->points[j].z;
				ColoredCloud1->points[j].r = color_R;
				ColoredCloud1->points[j].g = color_G;
				ColoredCloud1->points[j].b = color_B;
			}
		}
	}
	end = clock();		//程序结束用时
	double endtime = (double)(end - start) / CLOCKS_PER_SEC;
	cout << "Total time:" << endtime * 1000 << "ms" << endl;	//ms为单位
	ColoredCloud1 = PointCloudScale(100, ColoredCloud1);

	cout << "cpc success" << endl;
	return 1;
}

// 对比
void PreTreatment::CompareCloud(PointCloudT::Ptr cloud)
{

	//颜色区域生长分割
	clock_t start, end;
	start = clock();
	PointCloudT::Ptr CRGS_cloud(new PointCloudT);
	vector<pcl::PointIndices> clusters;
	RegionGrowingRGBSeg(cloud, CRGS_cloud, clusters);
	end = clock();		//程序结束用时
	double endtime = (double)(end - start) / CLOCKS_PER_SEC;
	cout << "Total time:" << endtime << endl;		//s为单位
	cout << "Total time:" << endtime * 1000 << "ms" << endl;	//ms为单位

	//CPC分割
	CPCSeg(cloud);
}

PointCloudT::Ptr PreTreatment::gethalffish(PointCloudT::Ptr cloud, PointT heade, PointT tails)
{
	//获取平面
	PointCloudT::Ptr plane_cloud(new PointCloudT);
	Vector4f abc;
	Eigen::Matrix3f axismat;
	axismat << 0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	Eigen::Matrix3f rotmat;
	rotmat = rot.transpose().inverse()*axismat;
	abc(3) = plane_coeff[3] + plane_coeff[0] * avg(0, 0) + plane_coeff[1] * avg(0, 1) + plane_coeff[2] * avg(0, 2);
	abc(0) = plane_coeff[0] * rotmat(0, 0) + plane_coeff[1] * rotmat(1, 0) + plane_coeff[2] * rotmat(2, 0);
	abc(1) = plane_coeff[0] * rotmat(0, 1) + plane_coeff[1] * rotmat(1, 1) + plane_coeff[2] * rotmat(2, 1);
	abc(2) = plane_coeff[0] * rotmat(0, 2) + plane_coeff[1] * rotmat(1, 2) + plane_coeff[2] * rotmat(2, 2);
	if (ismirr)
	{
		abc(0) = abc(0);
		abc(1) = abc(1);
		abc(2) = -abc(2);
	}
	if (isturnx)
	{
		abc(0) = abc(0);
		abc(1) = -abc(1);
		abc(2) = -abc(2);
		//cout << "transform_z success" << endl;
	}
	if (isturnz)
	{
		abc(0) = -abc(0);
		abc(1) = -abc(1);
		abc(2) = abc(2);
		//cout << "transform_x success" << endl;
	}
	//draw plane

	//PointCloudT::Ptr cloud_sortx = SortByX(cloud);
	//判断法线方向
	double t_w;
	Vector3d p1_w;
	Vector3d p0_w;//平面随机一点
	p1_w << 0, 0, 0;
	p0_w << 0, 0, -abc(3) / abc(2);
	Vector3d n_w;
	n_w << abc(0), abc(1), abc(2);
	t_w = (p0_w - p1_w).dot(n_w) / n_w.dot(n_w);
	PointCloudT::Ptr head_cloud_half(new PointCloudT);
	PointCloudT::Ptr body_cloud_half(new PointCloudT);
	double maxDisToPlane;//用于求头的对称平面
	maxDisToPlane = -9999;
	//投影到平面
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x > heade.x)
		{
			if (cloud->points[i].x < tails.x)
			{
				double t;
				Vector3d projectP;
				Vector3d p1;
				Vector3d p0;//平面随机一点
				p1 << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
				p0 << 0, 0, -abc(3) / abc(2);
				Vector3d n;
				n << abc(0), abc(1), abc(2);
				t = (p0 - p1).dot(n) / n.dot(n);
				projectP = t*n + p1;
				body_cloud_half->push_back(cloud->points[i]);
				PointT p;
				p.x = projectP(0);
				p.y = projectP(1);
				p.z = projectP(2);
				body_cloud_half->push_back(p);
			}
		}
		else
		{
			head_cloud_half->push_back(cloud->points[i]);
			double headToPlaneDis;
			headToPlaneDis = std::abs(abc(0)*cloud->points[i].x + abc(1)*cloud->points[i].y + abc(2)*cloud->points[i].z + abc(3)) / sqrt(abc(0)*abc(0) + abc(1)*abc(1) + abc(2)*abc(2));
			if (headToPlaneDis > maxDisToPlane)
			{
				maxDisToPlane = headToPlaneDis;
			}
		}
	}
	getheadPheno(head_cloud_half, abc);
	fish_width = maxDisToPlane;
	cout << "width: " << maxDisToPlane << endl;

	//补全头部
	//头部最高点到平面的中点
	//double headToPlaneDis;
	//headToPlaneDis = std::abs(abc(0)*headMaxy.x + abc(1)*headMaxy.y + abc(2)*headMaxy.z + abc(3)) / sqrt(abc(0)*abc(0) + abc(1)*abc(1) + abc(2)*abc(2));
	double abc3 = abc(3) - maxDisToPlane / 2;
	PointCloudT::Ptr head_cloud_full(new PointCloudT);
	for (int i = 0; i < head_cloud_half->points.size(); i++)
	{
		double t = 0;
		Vector3d p1;
		Vector3d p0;//平面随机一点
		p1 << head_cloud_half->points[i].x, head_cloud_half->points[i].y, head_cloud_half->points[i].z;
		p0 << 0, 0, -abc3 / abc(2);
		Vector3d n;
		n << abc(0), abc(1), abc(2);
		t = (p0 - p1).dot(n) / n.dot(n);
		double temp;
		if (t_w < 0)
		{
			temp = -t;
		}
		if (temp > 0)
		{
			Vector3d mirrPoint;
			mirrPoint = 2 * t*n + p1;
			PointT p;
			p.x = mirrPoint(0);
			p.y = mirrPoint(1);
			p.z = mirrPoint(2);
			head_cloud_full->push_back(p);
			head_cloud_full->push_back(head_cloud_half->points[i]);
		}
	}

	PointCloudT::Ptr cloud_full(new PointCloudT);
	for (int i = 0; i < body_cloud_half->points.size(); i++)
	{
		cloud_full->push_back(body_cloud_half->points[i]);
	}
	for (int i = 0; i < head_cloud_full->points.size(); i++)
	{
		cloud_full->push_back(head_cloud_full->points[i]);
	}

	return cloud_full;

}

void PreTreatment::getheadPheno(PointCloudT::Ptr cloud, Vector4f abc)
{
	double maxDisToPlane;//head_width
	maxDisToPlane = -9999;
	double headminx = 9999;
	double headmaxx = -9999;
	double headmaxy = -9999;
	double headminy = 9999;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		double headToPlaneDis;
		headToPlaneDis = std::abs(abc(0)*cloud->points[i].x + abc(1)*cloud->points[i].y + abc(2)*cloud->points[i].z + abc(3)) / sqrt(abc(0)*abc(0) + abc(1)*abc(1) + abc(2)*abc(2));
		if (headToPlaneDis > maxDisToPlane)
		{
			maxDisToPlane = headToPlaneDis;
		}
		if (cloud->points[i].x < headminx)
		{
			headminx = cloud->points[i].x;
		}
		if (cloud->points[i].y < headminy)
		{
			headminy = cloud->points[i].y;
		}
		if (cloud->points[i].y > headmaxy)
		{
			headmaxy = cloud->points[i].y;
		}
		if (cloud->points[i].x > headmaxx)
		{
			headmaxx = cloud->points[i].x;
		}
	}
	head_length_3d = headmaxx - headminx;
	head_height = headmaxy - headminy;
	head_width = maxDisToPlane;
	cout << "head_length: " << head_length_3d << endl;
	cout << "head_height: " << head_height << endl;
	cout << "head_width: " << maxDisToPlane << endl;
}

PointCloudT::Ptr PreTreatment::getFullFishBySection(PointCloudT::Ptr cloud, PointT heade, PointT dorsals, PointT ventrale, PointT tails, PointT taile)
{
	//获取平面
	PointCloudT::Ptr plane_cloud(new PointCloudT);
	Vector4f abc;
	Eigen::Matrix3f axismat;
	axismat << 0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	Eigen::Matrix3f rotmat;
	rotmat = rot.transpose().inverse()*axismat;
	abc(3) = plane_coeff[3] + plane_coeff[0] * avg(0, 0) + plane_coeff[1] * avg(0, 1) + plane_coeff[2] * avg(0, 2);
	abc(0) = plane_coeff[0] * rotmat(0, 0) + plane_coeff[1] * rotmat(1, 0) + plane_coeff[2] * rotmat(2, 0);
	abc(1) = plane_coeff[0] * rotmat(0, 1) + plane_coeff[1] * rotmat(1, 1) + plane_coeff[2] * rotmat(2, 1);
	abc(2) = plane_coeff[0] * rotmat(0, 2) + plane_coeff[1] * rotmat(1, 2) + plane_coeff[2] * rotmat(2, 2);
	if (ismirr)
	{
		abc(0) = abc(0);
		abc(1) = abc(1);
		abc(2) = -abc(2);
	}
	if (isturnx)
	{
		abc(0) = abc(0);
		abc(1) = -abc(1);
		abc(2) = -abc(2);
		//cout << "transform_z success" << endl;
	}
	if (isturnz)
	{
		abc(0) = -abc(0);
		abc(1) = -abc(1);
		abc(2) = abc(2);
		//cout << "transform_x success" << endl;
	}
	//draw plane
	for (int i = -150; i < 150; ++i)
	{
	for (int j = -150; j < 150; ++j)
	{
	PointT tempPoint;
	tempPoint.x = i;
	tempPoint.y = j;
	tempPoint.z = -(abc(0)*i + abc(1)*j + abc(3) / abc(2));
	plane_cloud->push_back(tempPoint);
	}
	}

	return plane_cloud;
}

void PreTreatment::get3DPheno(PointCloudT::Ptr cloud, PointT dorsals, PointT tails, PointT heade)
{
	PointCloudT::Ptr half_cloud(new PointCloudT);
	half_cloud = PreProcess(cloud);

	//dorsals.x = -14.06;
	//tails.x = 108.5;
	PointCloudT::Ptr cloud_full(new PointCloudT);
	cloud_full = gethalffish(half_cloud, heade, tails);
	//鱼头
	//heade.x = -82.92;
	PointCloudT::Ptr cloud_head_full(new PointCloudT);
	for (int i = 0; i < cloud_full->points.size(); i++)
	{
		if (cloud_full->points[i].x <= heade.x - 8)
		{
			cloud_head_full->push_back(cloud_full->points[i]);
		}
	}

	//Triagulation(cloud_full);
	head_volume = getVolume(cloud_head_full);
	all_volume = getVolume(cloud_full);

	cout << "head_volume: " << head_volume << endl;
	cout << "all_volume: " << all_volume << endl;
	//cout << "pre" << endl;
}

map<string, ThreeKeys> PreTreatment::getTXT(string filename)
{
	ifstream file(filename);
	map<string, ThreeKeys> fourmap;
	string str;
	while (getline(file, str))
	{
		vector<string> ss;
		int i = 0;
		while (i < str.size())
		{
			string temp;
			while (i < str.size()&&str[i] != ' ')
			{
				temp.push_back(str[i]);
				i++;
			}
			ss.push_back(temp);
			//cout << temp << endl;
			i++;
		}
		ThreeKeys afish;
		string name = ss[0];
		afish.dorsalsx = stod(ss[2]);
		afish.tailsx = stod(ss[3]);
		afish.headex = stod(ss[4]);
		fourmap.insert(pair<string, ThreeKeys>(name, afish));
		//cout << name << ": " << afish.dorsalsx << ' ' << afish.tailsx << ' ' << afish.headex << endl;
	}
	return fourmap;
}

void PreTreatment::get3DPhenoByExcel(PointCloudT::Ptr cloud, string name)
{
	string filename = "C:/Users/12401/Desktop/论文/数据分析/four.txt";
	map<string, ThreeKeys> fourmap;
	fourmap = getTXT(filename);
	auto iter = fourmap.find(name);
	PointT dorsals, tails, heade;
	if (iter != fourmap.end())
	{
		dorsals.x = iter->second.dorsalsx;
		tails.x = iter->second.tailsx;
		heade.x = iter->second.headex;
	}
	get3DPheno(cloud, dorsals, tails, heade);
	string outfile = "C:/Users/12401/Desktop/论文/数据分析/new3d.txt";
	ofstream out(outfile, ofstream::app);
	out << name << ' ' << head_volume << ' ' << all_volume << ' ' << head_length_3d << ' ' << head_width << ' ' << head_height << ' ' << fish_width << endl;
	//cout << fourmap.size() << endl;
	//cout << "onefish complete"<<endl;
}

map<string, vector<PointT>> PreTreatment::getKeypointbyTXT(string filename)
{
	map<string, vector<PointT>> m;
	ifstream file(filename);
	string str;
	while (getline(file, str))
	{
		vector<string> ss;
		int i = 0;
		while (i < str.size())
		{
			string temp;
			while (i < str.size() && str[i] != ' ')
			{
				temp.push_back(str[i]);
				i++;
			}
			ss.push_back(temp);
			i++;
		}
		vector<PointT> vecPt;
		for (int j = 2; j < ss.size(); j += 3)
		{
			PointT tempPt;
			tempPt.x = stof(ss[j]);
			tempPt.y = stof(ss[j + 1]);
			tempPt.z = stof(ss[j + 2]);
			vecPt.push_back(tempPt);
		}
		m.insert(pair<string, vector<PointT>>(ss[0], vecPt));
	}
	return m;
}


vector<double> PreTreatment::get3DPhenoByTXT(PointCloudT::Ptr cloud, string name)
{
	string filename = "C:/Users/12401/Desktop/论文/数据分析/keypoint.txt";
	map<string, vector<PointT>> m;
	m = getKeypointbyTXT(filename);
	auto iter = m.find(name);
	
	//cout << heade << endl;
	//头部弧线
	PointT heade = iter->second[11];
	ArcPara head_arc;
	head_arc = getArc(cloud, heade);
	cout << head_arc.arc_height << ' ' << head_arc.arc_width << ' ' << head_arc.arc_path << endl;
	cout << "headarc over" << endl;
	//背鳍弧线
	PointT dorsals = iter->second[0];
	ArcPara bei_arc;
	bei_arc = getArc(cloud, dorsals);
	cout << bei_arc.arc_height << ' ' << bei_arc.arc_width << " beihu: " << bei_arc.arc_path << endl;
	cout << "bei_arc over" << endl;
	//尾柄弧线
	PointT caudals = iter->second[12];
	PointT caudale = iter->second[13];
	PointCloudT::Ptr new_cloud(new PointCloudT);
	new_cloud = getcaudalCloud(cloud, caudale);
	ArcPara caudal_arc;
	caudal_arc = getArc(new_cloud, caudals);
	cout << caudal_arc.arc_height << ' ' << caudal_arc.arc_width << "weihu: " << caudal_arc.arc_path << endl;
	cout << "caudal_arc over" << endl;
	//角度
	PointT pelvics= iter->second[8];
	double angle;
	angle = getAngleFromDorsalAndPelvic(dorsals, pelvics);
	cout << angle << endl;
	//三角形角度
	TriAngle triAngle;
	PointT pectorals = iter->second[6];
	triAngle = getTriAngle(pectorals, dorsals, pelvics);
	cout << "dorsal: " << triAngle.dorsal_angle << "pectoral: " << triAngle.pectoral_angle << "pelvic: " << triAngle.pelvic_angle << endl;
	//cout << "over" << endl;

	vector<double> result;
	result.push_back(head_arc.arc_height);
	result.push_back(head_arc.arc_width);
	result.push_back(head_arc.arc_path);

	result.push_back(bei_arc.arc_height);
	result.push_back(bei_arc.arc_width);
	result.push_back(bei_arc.arc_path);

	result.push_back(caudal_arc.arc_height);
	result.push_back(caudal_arc.arc_width);
	result.push_back(caudal_arc.arc_path);

	result.push_back(angle);

	result.push_back(triAngle.dorsal_angle);
	result.push_back(triAngle.pectoral_angle);
	result.push_back(triAngle.pelvic_angle);

	return result;

}

//防止臀鳍的干扰
PointCloudT::Ptr PreTreatment::getcaudalCloud(PointCloudT::Ptr cloud, PointT caudale)
{
	PointCloudT::Ptr new_cloud(new PointCloudT);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].y >= caudale.y - 1 )
		{
			new_cloud->push_back(cloud->points[i]);
		}
	}

	return new_cloud;
}

ArcPara PreTreatment::getArc(PointCloudT::Ptr cloud, PointT heade)
{
	ArcPara arc;
	PointCloudT::Ptr head_cloud_arc(new PointCloudT);//头部点云
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x <= heade.x+1 && cloud->points[i].x >= heade.x-1)
		{
			head_cloud_arc->push_back(cloud->points[i]);
		}
	}
	Vector4f abc = getPlaneCoeff();
	PointCloudT::Ptr plane_cloud(new PointCloudT);
	PointCloudT::Ptr head_cloud_arc_half(new PointCloudT);
	//判断法线方向
	double t_w;//
	Vector3d p1_w;
	Vector3d p0_w;//平面随机一点
	p1_w << 0, 0, 0;
	p0_w << 0, 0, -abc(3) / abc(2);
	Vector3d n_w;
	n_w << abc(0), abc(1), abc(2);
	t_w = (p0_w - p1_w).dot(n_w) / n_w.dot(n_w);
	double maxDisToPlane;//用于求头的对称平面
	maxDisToPlane = -9999;
	for (int i = 0; i < head_cloud_arc->points.size(); i++)
	{
		double headToPlaneDis;
		headToPlaneDis = std::abs(abc(0)*head_cloud_arc->points[i].x + abc(1)*head_cloud_arc->points[i].y + abc(2)*head_cloud_arc->points[i].z + abc(3)) / sqrt(abc(0)*abc(0) + abc(1)*abc(1) + abc(2)*abc(2));
		if (headToPlaneDis > maxDisToPlane)
		{
			maxDisToPlane = headToPlaneDis;
		}
	}
	//弧线高
	arc.arc_height = (maxDisToPlane - 2) / 2;
	//头部最高点到平面的中点
	double abc3 = abc(3) - (maxDisToPlane + 2) / 2;
	PointT topMinPt, bottomMinPt;
	double topMin = 9999, bottomMin = 9999;
	double minY = 9999, maxY = -9999;
	
	
	for (int i = 0; i < head_cloud_arc->points.size(); i++)
	{
		double t = 0;
		Vector3d p1;
		Vector3d p0;//平面随机一点
		p1 << head_cloud_arc->points[i].x, head_cloud_arc->points[i].y, head_cloud_arc->points[i].z;
		p0 << 0, 0, -abc3 / abc(2);
		Vector3d n;
		n << abc(0), abc(1), abc(2);
		t = (p0 - p1).dot(n) / n.dot(n);
		double temp;
		if (t_w < 0)
		{
			temp = -t;
		}
		if (temp > 0)
		{
			head_cloud_arc_half->push_back(head_cloud_arc->points[i]);
			//if (head_cloud_arc->points[i].y > th)
			//{
				//if (temp < topMin)
				//{
					//topMin = temp;
					//topMinPt = head_cloud_arc->points[i];
				//}
			//}
			//else
			//{
				//if (temp < bottomMin)
				//{
					//bottomMin = temp;
					//bottomMinPt = head_cloud_arc->points[i];
				//}
			//}
		}
	}

	for (int i = 0; i < head_cloud_arc_half->points.size(); i++)
	{
		if (head_cloud_arc_half->points[i].y > maxY)
		{
			maxY = head_cloud_arc_half->points[i].y;
		}
		if (head_cloud_arc_half->points[i].y < minY)
		{
			minY = head_cloud_arc_half->points[i].y;
		}
	}
	double th = (maxY + minY) / 2;
	cout << "th " << th << endl;
	for (int i = 0; i < head_cloud_arc_half->points.size(); i++)
	{
		double t = 0;
		Vector3d p1;
		Vector3d p0;//平面随机一点
		p1 << head_cloud_arc_half->points[i].x, head_cloud_arc_half->points[i].y, head_cloud_arc_half->points[i].z;
		p0 << 0, 0, -abc3 / abc(2);
		Vector3d n;
		n << abc(0), abc(1), abc(2);
		t = (p0 - p1).dot(n) / n.dot(n);
		double temp;
		if (t_w < 0)
		{
			temp = -t;
		}
		if (temp > 0)
		{
			if (head_cloud_arc_half->points[i].y > th)
			{
				if (temp < topMin)
				{
					topMin = temp;
					topMinPt = head_cloud_arc_half->points[i];
				}
			}
			else
			{
				if (temp < bottomMin)
				{
					bottomMin = temp;
					bottomMinPt = head_cloud_arc_half->points[i];
				}
			}
		}
	}

	cout << topMinPt << bottomMinPt << endl;
	//弧线宽
	//投影
	arc.arc_width = sqrt(pow(topMinPt.y - bottomMinPt.y, 2) + pow(topMinPt.z - bottomMinPt.z, 2));
	//计算弧线长度
	pcl::KdTreeFLANN<PointT> tree;
	tree.setInputCloud(head_cloud_arc_half);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	double r = 5.0;
	vector<int> flag(head_cloud_arc_half->points.size());//判断点是否已访问,初始化为0
	vector<double> dis(head_cloud_arc_half->points.size());//各点到起始点的距离,初始化为maxint
	for (int i = 0; i < flag.size(); i++)
	{
		flag[i] = 0;
		dis[i] = INT_MAX;
	}
	//迪杰斯特拉
	//获得起始点索引
	int startIdx = 0;
	int endIdx = 0;
	for (int i = 0; i < head_cloud_arc_half->points.size(); i++)
	{
		if (CalTwoPointDistance(topMinPt, head_cloud_arc_half->points[i]) < 0.1)
		{
			startIdx = i;
		}
		if (CalTwoPointDistance(bottomMinPt, head_cloud_arc_half->points[i]) < 0.1)
		{
			endIdx = i;
		}
	}
	//cout << startIdx << "  " << endIdx << endl;
	stack<int> s;
	s.push(startIdx);
	flag[startIdx] = 1;
	dis[startIdx] = 0;
	vector<int> shortestPath(head_cloud_arc_half->points.size());
	shortestPath[startIdx] = startIdx;
	while (s.size() < dis.size())
	{
		int idx = s.top();
		//更新各节点权值
		if (tree.radiusSearch(head_cloud_arc_half->points[idx], r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				if (pointRadiusSquaredDistance[i] > 0.01) 
				{
					if ((sqrt(pointRadiusSquaredDistance[i]) + dis[idx]) <= dis[pointIdxRadiusSearch[i]])
					{
						dis[pointIdxRadiusSearch[i]] = sqrt(pointRadiusSquaredDistance[i]) + dis[idx];
						shortestPath[pointIdxRadiusSearch[i]] = idx;
					}
				}
			}
		}
		//for (int i = 0; i < shortestPath.size(); i++)
		//{
			//cout << shortestPath[i] << ' ';
		//}
		//cout << endl;
		//获取当前权值最小的点
		int minIdxtemp;
		double minDistemp = INT_MAX;
		for (int i = 0; i < dis.size(); i++)
		{
			if (flag[i] == 0)
			{
				if (dis[i] < minDistemp)
				{
					minDistemp = dis[i];
					minIdxtemp = i;
				}
			}
		}
		s.push(minIdxtemp);
		flag[minIdxtemp] = 1;
		
	}



	//找到最短路径
	for (int i = -150; i < 150; ++i)
	{
		for (int j = -150; j < 150; ++j)
		{
			PointT tempPoint;
			tempPoint.x = i;
			tempPoint.y = j;
			tempPoint.z = -(abc(0)*i + abc(1)*j + abc3 / abc(2));
			plane_cloud->push_back(tempPoint);
		}
	}
	
	arc.arc_path = dis[endIdx];
	return arc;
}

double PreTreatment::getAngleFromDorsalAndPelvic(PointT dorsals, PointT pelvics)
{
	Vector3d ds, ps, tempPt;
	ds << dorsals.x, dorsals.y, dorsals.z;
	ps << pelvics.x, pelvics.y, pelvics.z;
	tempPt << dorsals.x, dorsals.y - 60, dorsals.z;
	double angle;
	angle = acos(((ds - tempPt).dot(ds - ps)) / ((ds - tempPt).norm()*(ds - ps).norm()));
	return angle * 180 / 3.1415926;
}

TriAngle PreTreatment::getTriAngle(PointT pectorals, PointT dorsals, PointT pelvics)
{
	TriAngle triAngle;
	Vector3d ds, ps, pes;
	pes << pectorals.x, pectorals.y, pectorals.z;
	ds << dorsals.x, dorsals.y, dorsals.z;
	ps << pelvics.x, pelvics.y, pelvics.z;
	triAngle.dorsal_angle = acos(((ds - pes).dot(ds - ps)) / ((ds - pes).norm()*(ds - ps).norm())) * 180 / 3.1415926;
	triAngle.pectoral_angle= acos(((pes - ds).dot(pes - ps)) / ((pes - ds).norm()*(pes - ps).norm())) * 180 / 3.1415926;
	triAngle.pelvic_angle= acos(((ps - ds).dot(ps - pes)) / ((ps - ds).norm()*(ps - pes).norm())) * 180 / 3.1415926;
	return triAngle;
}

Vector4f PreTreatment::getPlaneCoeff()
{
	//获取平面
	
	Vector4f abc;
	Eigen::Matrix3f axismat;
	axismat << 0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	Eigen::Matrix3f rotmat;
	rotmat = rot.transpose().inverse()*axismat;
	abc(3) = plane_coeff[3] + plane_coeff[0] * avg(0, 0) + plane_coeff[1] * avg(0, 1) + plane_coeff[2] * avg(0, 2);
	abc(0) = plane_coeff[0] * rotmat(0, 0) + plane_coeff[1] * rotmat(1, 0) + plane_coeff[2] * rotmat(2, 0);
	abc(1) = plane_coeff[0] * rotmat(0, 1) + plane_coeff[1] * rotmat(1, 1) + plane_coeff[2] * rotmat(2, 1);
	abc(2) = plane_coeff[0] * rotmat(0, 2) + plane_coeff[1] * rotmat(1, 2) + plane_coeff[2] * rotmat(2, 2);
	if (ismirr)
	{
		abc(0) = abc(0);
		abc(1) = abc(1);
		abc(2) = -abc(2);
	}
	if (isturnx)
	{
		abc(0) = abc(0);
		abc(1) = -abc(1);
		abc(2) = -abc(2);
		//cout << "transform_z success" << endl;
	}
	if (isturnz)
	{
		abc(0) = -abc(0);
		abc(1) = -abc(1);
		abc(2) = abc(2);
		//cout << "transform_x success" << endl;
	}

	
	return abc;
}

void PreTreatment::validatecloud(PointCloudT::Ptr auto_cloud, PointCloudT::Ptr manual_cloud)
{
	//为manual_cloud建立kdtree
	cout << manual_cloud->points.size() << endl;
	cout << auto_cloud->points.size() << endl;
	pcl::KdTreeFLANN<PointT> tree;
	tree.setInputCloud(manual_cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	double r = 1.0;
	int tt = 0;//true in true
	int ft = 0;// false in true
	int tf = 0;
	for (int i = 0; i < auto_cloud->points.size(); ++i)
	{
		if (tree.radiusSearch(auto_cloud->points[i], r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			int n = 0;
			for ( ;n < pointIdxRadiusSearch.size(); ++n)
			{
				if (pointRadiusSquaredDistance[n] < 0.4)
				{
					tt++;
					break;
				}
			}
			if (n >= pointIdxRadiusSearch.size())
			{
				ft++;
			}
		}
		else
		{
			ft++;
		}
	}
	tf = manual_cloud->points.size() - tt;
	cout << "tt: " << tt << endl;
	cout << "ft: " << ft << endl;
	cout << "tf: " << tf << endl;
}