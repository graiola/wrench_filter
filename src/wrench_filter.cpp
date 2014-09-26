#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/WrenchStamped.h"

class PoseFilter
{
public:
    PoseFilter(std::string topic_name,std::string ref_frame):
        n_private_("~"),
        tf_(),
        target_frame_(ref_frame),
        topic_name_(topic_name)
        
    {
        ROS_INFO("Filter subscribed to topic:  %s", topic_name_.c_str());
        pose_sub_.subscribe(n_,topic_name_, 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::WrenchStamped>(pose_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&PoseFilter::msgCallback, this, _1) );
        publisher_ = n_.advertise<geometry_msgs::WrenchStamped>(topic_name_ + "_filt", 10);

    }

private:
    message_filters::Subscriber<geometry_msgs::WrenchStamped> pose_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::WrenchStamped> * tf_filter_;
    ros::NodeHandle n_, n_private_;
    std::string target_frame_;
    std::string topic_name_;
    ros::Publisher publisher_;


    //  Callback to register with tf::MessageFilter to be called when WrenchStamped are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::WrenchStamped>& wrench)
    {
        try
        {
           
            publisher_.publish(wrench);

        }
        catch (tf::TransformException &ex)
        {
            printf ("Failure %s\n", ex.what()); //Print exception which was caught
        }
    };
};


/*
class PoseFilterHead
{
public:
    PoseFilterHead(std::string topic_name,std::string ref_frame) : tf_(),  target_frame_(ref_frame), topic_name_(topic_name)
    {
        ROS_INFO("Topic is:  %s", topic_name_.c_str());
        pose_sub_.subscribe(n_,topic_name_, 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::Vector3Stamped>(pose_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&PoseFilterHead::msgCallback, this, _1) );
        publisher_ = n_.advertise<geometry_msgs::Vector3>(topic_name_ + "_filt", 10);
    }

private:
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> pose_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::Vector3Stamped> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    std::string topic_name_;
    ros::Publisher publisher_;

    //  Callback to register with tf::MessageFilter to be called when Vector3Stamped are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::Vector3Stamped>& pose_ptr)
    {
        geometry_msgs::Vector3Stamped pose_out;
        geometry_msgs::Vector3 vect_out;
        try
        {
            tf_.transformVector(target_frame_, *pose_ptr, pose_out);

            vect_out = pose_out.vector;

            publisher_.publish(vect_out);
        }
        catch (tf::TransformException &ex)
        {
            printf ("Failure %s\n", ex.what()); //Print exception which was caught
        }
    };

};
*/

int main(int argc, char ** argv)
{
    if(argc != 3)
        ROS_ERROR_STREAM("Wrong number of arguments");

    ros::init(argc, argv, "wrench_filter"); //Init ROS

    PoseFilter pd(argv[1],argv[2]);

    ros::spin(); // Run until interupted
};
