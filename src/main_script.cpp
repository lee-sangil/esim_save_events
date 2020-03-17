#include "common.h"
#include "parser.h"

class Communicator{
	public:
		Communicator(){
			if(Parser::hasOption("-help")){
				std::cout << "Optional -o: Output folder path (default: ros package location).\n"
					"Example: roslaunch esim_save_events esim_save_events.launch -o /path/to/output/" << std::endl;
			}
			
			// Make folders for saving current image
			std::string save_loc;
			if(Parser::hasOption("-o")){
				save_loc = Parser::getStringOption("-o");
			}else{
				std::string pwd = ros::package::getPath("esim_save_events");
				save_loc = pwd + "/dataset/";
			}

			folder_name_image = save_loc + "image/";
			folder_name_depth = save_loc + "depth/";
			folder_name_flow = save_loc + "flow/";

			std::string folder_remove_command;
			std::string folder_create_command;

			folder_remove_command = "rm -rf " + folder_name_image;
			folder_create_command = "mkdir -p " + folder_name_image;
			system(folder_remove_command.c_str());
			system(folder_create_command.c_str());

			folder_remove_command = "rm -rf " + folder_name_depth;
			folder_create_command = "mkdir -p " + folder_name_depth;
			system(folder_remove_command.c_str());
			system(folder_create_command.c_str());

			folder_remove_command = "rm -rf " + folder_name_flow;
			folder_create_command = "mkdir -p " + folder_name_flow;
			system(folder_remove_command.c_str());
			system(folder_create_command.c_str());

			// Make image filename log
			image_log.open((save_loc + "image.txt").c_str());
			depth_log.open((save_loc + "depth.txt").c_str());
			flow_log.open((save_loc + "flow.txt").c_str());
			event_log.open((save_loc + "events.txt").c_str());
			imu_log.open((save_loc + "imu.txt").c_str());
			pose_log.open((save_loc + "pose.txt").c_str());

			image_log << "# gray images" << std::endl;
			image_log << "# timestamp filename" << std::endl;

			depth_log << "# depth images" << std::endl;
			depth_log << "# timestamp filename" << std::endl;

			flow_log << "# flow color images" << std::endl;
			flow_log << "# timestamp filename" << std::endl;
			
			event_log << "# events" << std::endl;
			event_log << "# timestamp x y polarity" << std::endl;

			imu_log << "# imu" << std::endl;
			imu_log << "# acceleration gyroscope" << std::endl;
			imu_log << "# timestamp ax ay az gx gy gz" << std::endl;

			pose_log << "# time x y z qx qy qz qw vx vy vz wx wy wz" << std::endl;
			ROS_INFO("file is successfully opened.");

			sub_image = nh_.subscribe<sensor_msgs::Image>("/image", 1, &Communicator::callback_aps, this );
			sub_depth = nh_.subscribe<sensor_msgs::Image>("/depth", 1, &Communicator::callback_depth, this );
			sub_flow = nh_.subscribe<sensor_msgs::Image>("/flow_color", 1, &Communicator::callback_flow, this );
			sub_imu = nh_.subscribe<sensor_msgs::Imu>("/imu", 1, &Communicator::callback_imu, this );
			sub_event = nh_.subscribe<dvs_msgs::EventArray>("/event", 1, &Communicator::callback_dvs, this );
			sub_pose = nh_.subscribe<geometry_msgs::PoseStamped>("/pose", 1, &Communicator::callback_pose, this );

			ROS_INFO("initialize ROS");
		}
		~Communicator(){
			if( image_log.is_open() ) image_log.close();
			if( depth_log.is_open() ) depth_log.close();
			if( flow_log.is_open() ) flow_log.close();
			if( event_log.is_open() ) event_log.close();
			if( imu_log.is_open() ) imu_log.close();
			if( pose_log.is_open() ) pose_log.close();

			ROS_INFO("file is successfully closed.");
		}
		void callback_aps(const sensor_msgs::Image::ConstPtr& msg_image);
		void callback_depth(const sensor_msgs::Image::ConstPtr& msg_depth);
		void callback_flow(const sensor_msgs::Image::ConstPtr& msg_flow);
		void callback_imu(const sensor_msgs::Imu::ConstPtr& msg_imu);
		void callback_dvs(const dvs_msgs::EventArray::ConstPtr& msg_event);
		void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_pose, sub_image, sub_depth, sub_flow, sub_imu, sub_event;
		std::ofstream image_log, depth_log, flow_log, event_log, imu_log, pose_log;
		std::string folder_name_image, folder_name_depth, folder_name_flow;
};

void Communicator::callback_aps(const sensor_msgs::ImageConstPtr& msg_image){
	cv_bridge::CvImagePtr img_ptr_image;

	try{
		img_ptr_image = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream time;
	time << msg_image->header.stamp.sec << '.' << std::setfill('0') << std::setw(9) << msg_image->header.stamp.nsec;

	cv::Mat image = img_ptr_image->image;

	std::string image_file_name;
	image_file_name = folder_name_image + time.str() + ".png";
	cv::imwrite(image_file_name, image);

	image_log << time.str() << " image/" << time.str() << ".png" << std::endl;
}

void Communicator::callback_depth(const sensor_msgs::ImageConstPtr& msg_depth){
	cv_bridge::CvImagePtr img_ptr_image;

	try{
		img_ptr_image = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream time;
	time << msg_depth->header.stamp.sec << '.' << std::setfill('0') << std::setw(9) << msg_depth->header.stamp.nsec;

	cv::Mat image = img_ptr_image->image;

	std::string image_file_name;
	image_file_name = folder_name_depth + time.str() + ".png";
	cv::imwrite(image_file_name, image);

	depth_log << time.str() << " depth/" << time.str() << ".png" << std::endl;
}

void Communicator::callback_flow(const sensor_msgs::ImageConstPtr& msg_flow){
	cv_bridge::CvImagePtr img_ptr_image;

	try{
		img_ptr_image = cv_bridge::toCvCopy(msg_flow, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream time;
	time << msg_flow->header.stamp.sec << '.' << std::setfill('0') << std::setw(9) << msg_flow->header.stamp.nsec;

	cv::Mat image = img_ptr_image->image;

	std::string image_file_name;
	image_file_name = folder_name_flow + time.str() + ".png";
	cv::imwrite(image_file_name, image);

	flow_log << time.str() << " flow/" << time.str() << ".png" << std::endl;
}

void Communicator::callback_dvs(const dvs_msgs::EventArray::ConstPtr& msg_event){

	if( msg_event->header.stamp.sec != 0 ){
		for( unsigned int e = 0; e < msg_event->events.size(); e++ ){
			event_log << std::setprecision(9) << std::fixed
				<< msg_event->events[e].ts << '\t'
				<< msg_event->events[e].x << '\t'
				<< msg_event->events[e].y << '\t'
				<< msg_event->events[e].polarity+0 << std::endl;
		}
	}
}

void Communicator::callback_imu(const sensor_msgs::Imu::ConstPtr& msg_imu){
	
	std::stringstream time;
	time << msg_imu->header.stamp.sec << '.' << std::setfill('0') << std::setw(9) << msg_imu->header.stamp.nsec;

	imu_log << time.str() << '\t'
		<< std::setprecision(9) << std::fixed
		<< msg_imu->linear_acceleration.x << '\t'
		<< msg_imu->linear_acceleration.y << '\t'
		<< msg_imu->linear_acceleration.z << '\t'
		<< msg_imu->angular_velocity.x << '\t'
		<< msg_imu->angular_velocity.y << '\t'
		<< msg_imu->angular_velocity.z << std::endl;
}

void Communicator::callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose){
	
	std::stringstream time;
	time << msg_pose->header.stamp.sec << '.' << std::setfill('0') << std::setw(9) << msg_pose->header.stamp.nsec;
	
	pose_log << time.str() << '\t'
		<< std::setprecision(9) << std::fixed
		<< msg_pose->pose.position.x << '\t'
		<< msg_pose->pose.position.y << '\t'
		<< msg_pose->pose.position.z << '\t'
		<< msg_pose->pose.orientation.x << '\t'
		<< msg_pose->pose.orientation.y << '\t'
		<< msg_pose->pose.orientation.z << '\t'
		<< msg_pose->pose.orientation.w << std::endl;
}

int main(int argc, char * argv[]){

	ros::init(argc, argv, "esim_save_events");
	Parser::init(argc, argv);

	Communicator comm_;

	while( ros::ok() ) {
		ros::spinOnce();
		if( cv::waitKey(1) == 'q') break;
	}

	return 0;
}
