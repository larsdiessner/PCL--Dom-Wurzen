#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "CylinderSegmentation.h"


CylinderSegmentation::CylinderSegmentation(void)
{}
CylinderSegmentation::~CylinderSegmentation(void)
{}

typedef pcl::PointXYZRGBA PointT;

// output objects
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> CS_out;

void CylinderSegmentation::UseCylinderSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string  cloud_output_path)
{

  // All the objects needed
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {

	pcl::PCDWriter writer;
	writer.write (cloud_output_path, *cloud_cylinder, false);
	std::cout << "PointCloud saved to File: " << cloud_output_path << std::endl;
	CS_out.push_back(cloud_cylinder);
  }
  };

  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> CylinderSegmentation::getOutput(){
	return CS_out;
}

  /*
Quellen:
	//CylinderSegmentation, um Punkte in einer Ebene zu finden
	//http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php#cylinder-segmentation
	*/