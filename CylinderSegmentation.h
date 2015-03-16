#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class CylinderSegmentation
{
public:
	CylinderSegmentation(void);
	~CylinderSegmentation(void);
	void CylinderSegmentation::UseCylinderSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string  cloud_output_path);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getOutput();
};
