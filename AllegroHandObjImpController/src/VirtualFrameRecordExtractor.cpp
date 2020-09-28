/*VIRTUALFRAMERECORDEXTRACTOR CPP*/
/*==========================================================================
 * 
 * A tool to extract ros bag of virtual frame pose type to plain text file
 *
 *========================================================================*/

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <boost/foreach.hpp>

#include "geometry_msgs/PoseStamped.h"

#define DATA_TOPIC      "/kuka_allegro/obj_pose"

int main(int argc, char** argv)
{
    //extract parms
    if(argc != 3)
    {
        printf("Please specify name of bag file as well as output text file.\n");
        exit(1);
    }

    std::string bagFileName = argv[1];
    std::string textFileName = argv[2];

    //prepare bag structure
    rosbag::Bag bag;
    bag.open(bagFileName.c_str(), rosbag::bagmode::Read);

    //construct a view for message query
    rosbag::View view(bag, rosbag::TopicQuery(DATA_TOPIC));

    FILE* pTextFile = fopen(textFileName.c_str(), "w");

    //extract messages
    if(pTextFile != NULL)
    {
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::PoseStamped::ConstPtr pData = m.instantiate< geometry_msgs::PoseStamped >();
            if(pData != NULL)
            {
                //extract dimensions interest us
                fprintf(pTextFile, "%lf %lf %lf %lf %lf %lf %lf\n", pData->pose.position.x, pData->pose.position.y, pData->pose.position.z, //position components
                        pData->pose.orientation.w, pData->pose.orientation.x, pData->pose.orientation.y, pData->pose.orientation.z);
            }
            else
            {
                printf("Fail to instantiate the data.\n");
            }
        }
    }
    else
    {
        printf("Fail to create file %s.\n", textFileName.c_str());
    }

    fclose(pTextFile);
    bag.close();

    return 0;
}

