#include "vsido_birateral/vsido_birateral.h"

int main( int argc, char** argv )
{
//    ros::init(argc, argv, "vsido_birateral");
    ros::init(argc, argv, "vsido_birateral");    
    ros::NodeHandle n;
    bool success = true;
    boost::asio::io_service io_service;
    VSidoBirateral vb(&n,io_service,success);

    boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

	ros::spin();
}
