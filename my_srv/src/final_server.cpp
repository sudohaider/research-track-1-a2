#include "ros/ros.h"
#include "my_srv/Final.h"
/***
Advertises the random values between a given range on the /final node. 
***/



double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


bool myrandom (my_srv::Final::Request &req, my_srv::Final::Response &res){
    res.target_index = randMToN(req.min, req.max);
    return true;
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "final_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/final", myrandom);
   ros::spin();

   return 0;
}
