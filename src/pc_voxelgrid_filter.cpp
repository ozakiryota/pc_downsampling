#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PcVoxelgridFilter{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_;
        /*tool*/
        pcl::VoxelGrid<pcl::PointXYZI> vg_;
        /*buffer*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ {new pcl::PointCloud<pcl::PointXYZI>};
		/*parameter*/
		double leafsize_;

	public:
		PcVoxelgridFilter();
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void filter(void);
		void publication(std_msgs::Header header);
};

PcVoxelgridFilter::PcVoxelgridFilter()
	: nh_private_("~")
{
	std::cout << "----- pc_voxelgrid_filter -----" << std::endl;
	/*parameter*/
	nh_private_.param("leafsize", leafsize_, 0.5);
	std::cout << "leafsize_ = " << leafsize_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcVoxelgridFilter::callback, this);
	/*publisher*/
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/downsampled", 1);
}

void PcVoxelgridFilter::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *pc_);
    filter();
	publication(msg->header);
}

void PcVoxelgridFilter::filter(void)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
	vg_.setInputCloud(pc_);
	vg_.setLeafSize(leafsize_, leafsize_, leafsize_);
	vg_.filter(*tmp);
	pc_ = tmp;
}

void PcVoxelgridFilter::publication(std_msgs::Header header)
{
    if(!pc_->points.empty()){
        sensor_msgs::PointCloud2 ros_pc;
        pcl::toROSMsg(*pc_, ros_pc);
        ros_pc.header = header;
        pub_.publish(ros_pc);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_voxelgrid_filter");
	
	PcVoxelgridFilter pc_voxelgrid_filter;

	ros::spin();
}