#include <ros/ros.h>


// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;
using namespace gtsam;


class PlanerLandmarkSLAM
{

public:

  // constructor
  PlanerLandmarkSLAM();

private:

  ros::NodeHandle nh;
  ros::Subscriber odom_sub_;
  ros::Subscriber flag_sub_;
  ros::Subscriber detect_sub_;

  //factorgraph
  NonlinearFactorGraph graph;

  //check if move
  bool move_;

  //pose id
  int pose_count_;
  int landmark_count_;

  //pose noise model
  noiseModel::Diagonal::shared_ptr odometryNoise_;

  //detection noise model- changed based on noise model
  noiseModel::Diagonal::shared_ptr detectionNoise_;

  //initial estimate
  Values initialEstimate;

  //previous pose saved
  double x_previous_;
  double y_previous_;
  double theta_previous_;

  // rosparam
  std::string save_file_path_;
  std::string odom_sub_topic_;


  void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
  {
 
    if(pose_count_ == 0){
      Pose2 prior(0.0, 0.0, 0.0);  // prior mean is at origin
      auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector3(0.3, 0.3, 0.1));            // 30cm std on x,y, 0.1 rad on theta
      graph.addPrior(++pose_count_, prior, priorNoise);  // add directly to graph
      initialEstimate.insert(pose_count_, Pose2(0.0, 0.0, 0.0)); //create initial estimate
      ROS_INFO("add prior factor.");
    //   pose_count_++;
    }

    else{
      // save odometry information
      double tx = msg->pose.pose.position.x;
      double ty = msg->pose.pose.position.y;
      double tz = msg->pose.pose.position.z;
  
      double qx = msg->pose.pose.orientation.x;
      double qy = msg->pose.pose.orientation.y;
      double qz = msg->pose.pose.orientation.z;
      double qw = msg->pose.pose.orientation.w;
  
      //Calculate theta
      double theta = QuatToYaw(qx, qy, qz, qw);

      //get odometry
      double dx = tx - x_previous_;
      double dy = ty - y_previous_;
      double dtheta = theta - theta_previous_;
      Pose2 odometry(dx, dy, dtheta);

      //check if robot is moving
      move_ = (abs(dx) + abs(dy)) > 1e-3 or abs(dtheta) > 1e-2;

      if (move_)
      {
        //If moving, add between factor to graph
        graph.emplace_shared<BetweenFactor<Pose2> >(pose_count_, pose_count_+1, odometry, odometryNoise_);

        //create initial estimate
        initialEstimate.insert(pose_count_+1, Pose2(tx, ty, theta));

        ROS_INFO("emplace factor between pose_%d - pose_%d", pose_count_, pose_count_+1);
        pose_count_++;
      }
  
      //update
      x_previous_ = tx;
      y_previous_ = ty;
      theta_previous_ = theta;
 
    }
    
    
    
  }

  void DetectionCallBack(const geometry_msgs::Pose::ConstPtr& msg)
  {
    // Get relative pose btw robot-landmark
    float tx = msg->position.x;
    float ty = msg->position.y;
    float tz = 0.00;
    float qx = msg->orientation.x;
    float qy = msg->orientation.y;
    float qz = msg->orientation.z;
    float qw = msg->orientation.w;
    float theta = QuatToYaw(qx, qy, qz, qw);
    Pose2 rel_pose(tx, ty, theta);
    
    //emplace factor based on landmark id
    if(move_)
    {
      graph.emplace_shared<BetweenFactor<Pose2> >(pose_count_, symbol('L',landmark_count_), rel_pose, detectionNoise_);
      ROS_INFO("emplace factor between pose_%d - landmark_%d", pose_count_, landmark_count_);

      initialEstimate.insert(symbol('L', landmark_count_), Pose2(tx, ty, theta));
      landmark_count_++;
    }
  }


  void FlagCallBack(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg->data){

      std::string filename = save_file_path_;
      
      //save graph to save file path;
    }

  }

  float QuatToYaw(double qx, double qy, double qz, double qw) 
  {
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
  }
};

PlanerLandmarkSLAM::PlanerLandmarkSLAM() : nh() {


  nh.param("save_file", save_file_path_, std::string("/home/hyunjae/SAVEFILE"));
  nh.param("odom_sub_topic", odom_sub_topic_, std::string("odom_throttle"));


  ROS_INFO("save_file: %s" , save_file_path_.c_str());
  ROS_INFO("odom_sub_topic: %s", odom_sub_topic_.c_str());


  odometryNoise_ = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));  // 20cm std on x,y, 0.1 rad on theta
  detectionNoise_ = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));  // 20cm std on x,y, 0.1 rad on theta
  pose_count_ = 0;
  landmark_count_=1;
  move_ = false;
  x_previous_=0.0;
  y_previous_=0.0;
  theta_previous_=0.0;


  odom_sub_   =  nh.subscribe(odom_sub_topic_, 10, &PlanerLandmarkSLAM::OdomCallBack, this);
  detect_sub_ =  nh.subscribe("detection_throttle", 10, &PlanerLandmarkSLAM::DetectionCallBack, this);
  flag_sub_   =  nh.subscribe("optimize_flag", 10, &PlanerLandmarkSLAM::FlagCallBack, this);


  ROS_INFO("Graph SLAM Node ready.");
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"landmark_slam");
  PlanerLandmarkSLAM pls;
  ros::spin();
  return 0;
}