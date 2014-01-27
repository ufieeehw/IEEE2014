#include <limits>
#include <memory>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <xv11_driver/LaserMeasurements.h>

#include "message_orderer.hpp"

using Eigen::Vector2d;

static double const pi = boost::math::constants::pi<double>();
boost::random::mt19937 rng;

class IObject {
public:
  virtual ~IObject() {
  };
  virtual double intersectWithRay(Vector2d start, Vector2d direction) const = 0;
};

class Line : public IObject {
  Vector2d point1;
  Vector2d point2;

public:
  Line(Vector2d point1, Vector2d point2) :
    point1(point1), point2(point2) {
  }
  
  double intersectWithRay(Vector2d start, Vector2d direction) const {
    direction = direction.normalized();
        
    Vector2d line_point = point1;
    Vector2d line_direction = (point2 - point1).normalized();
    Vector2d line_perpendicular = Vector2d(line_direction(1), -line_direction(0));
    
    Vector2d start2 = start - line_point;
    
    double signed_distance = start2.dot(line_perpendicular);
    double decrease = direction.dot(line_perpendicular);
    if(decrease == 0) {
      // ray is parallel
      return std::numeric_limits<double>::infinity();
    };
    
    double res = -signed_distance/decrease;
    if(res < 0) {
      // ray travels away from line
      return std::numeric_limits<double>::infinity();
    }
    
    Vector2d hit_point = start + res * direction;
    double distance_along_line = (hit_point - line_point).dot(line_direction);
    if(distance_along_line < 0 || distance_along_line > (point2 - point1).norm()) {
      // ray doesn't hit line between its endpoints
      return std::numeric_limits<double>::infinity();
    }
    
    return res;
  }
};

class RobotParticle {
  Vector2d position;
  double angle;
  double weight;

public:
  RobotParticle(Vector2d position, double angle, double weight) :
    position(position), angle(angle), weight(weight) {
  }
  
  Vector2d getPosition() const {
    return position;
  }
  double getAngle() const {
    return angle;
  }
  double getWeight() const {
    return weight;
  }
  Vector2d getForwardVector() const {
    return Vector2d(cos(angle), sin(angle));
  }
  Vector2d getLeftVector() const {
    return Vector2d(cos(angle+pi/2), sin(angle+pi/2));
  }
  
  RobotParticle predict(geometry_msgs::PoseStamped const &odom_msg) const {
    double dt = 1/360./3.; // XXX
    return RobotParticle(
      position + (
          getForwardVector() * odom_msg.pose.position.x +
             getLeftVector() * odom_msg.pose.position.y
        ) + Vector2d(
          boost::random::normal_distribution<>(0, sqrt(.01*dt))(rng),
          boost::random::normal_distribution<>(0, sqrt(.01*dt))(rng)
        ),
      angle +
        odom_msg.pose.orientation.z * (odom_msg.pose.orientation.w > 0 ? 1 : -1) * 2 + // hackish, valid for small angles
        boost::random::normal_distribution<>(0, sqrt(.01*dt))(rng),
      weight);
  }
  
  RobotParticle update(xv11_driver::LaserMeasurements const &lidar_msg, std::vector<std::unique_ptr<IObject> > const &course) const {
    double laser_angle = lidar_msg.angle_min;
    
    Vector2d start = position;
    Vector2d direction = Vector2d(cos(angle + laser_angle), sin(angle + laser_angle));
    
    double dist = std::numeric_limits<double>::infinity();
    BOOST_FOREACH(std::unique_ptr<IObject> const &objp, course) {
      dist = std::min(dist, objp->intersectWithRay(start, direction));
    }
    
    double measured = lidar_msg.ranges[0];
    
    double new_weight = weight;
    if(!isinf(dist) && !isinf(measured)) {
      new_weight *= .1 + .9*exp(-pow(log(dist/measured), 2));
    }
    
    return RobotParticle(
      position,
      angle,
      new_weight);
  }
};

void fixup(std::vector<RobotParticle> &particles, unsigned int N) {
  double total_weight = 0; BOOST_FOREACH(RobotParticle const &particle, particles) total_weight += particle.getWeight();
  
  BOOST_FOREACH(RobotParticle &particle, particles) {
    particle = RobotParticle(particle.getPosition(), particle.getAngle(), particle.getWeight()/total_weight);
  }
  
  double total_weight2 = 0; BOOST_FOREACH(RobotParticle const &particle, particles) total_weight2 += pow(particle.getWeight(), 2);
  double effective_N = 1/total_weight2;
  if(effective_N < N/2.) {
    std::cout << "resampling" << std::endl;
    std::vector<RobotParticle> new_particles;
    BOOST_FOREACH(RobotParticle const &particle, particles) {
      unsigned int new_count = N * particle.getWeight() + boost::random::uniform_01<>()(rng);
      for(unsigned int i = 0; i < new_count; i++) {
        new_particles.push_back(RobotParticle(particle.getPosition(), particle.getAngle(), 1));
      }
    }
    particles = new_particles;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ieee2014_localization");
  
  ros::NodeHandle nh;
  
  double course_length = (97 - 3/4. * 2) * 0.0254;
  double course_width = (49 - 3/4. * 2) * 0.0254;
  std::vector<std::unique_ptr<IObject> > course;
  course.emplace_back(new Line(Vector2d(-course_length/2, +course_width/2), Vector2d(+course_length/2, +course_width/2)));
  course.emplace_back(new Line(Vector2d(+course_length/2, +course_width/2), Vector2d(+course_length/2, -course_width/2)));
  course.emplace_back(new Line(Vector2d(+course_length/2, -course_width/2), Vector2d(-course_length/2, -course_width/2)));
  course.emplace_back(new Line(Vector2d(-course_length/2, -course_width/2), Vector2d(-course_length/2, +course_width/2)));
  
  unsigned int const N = 200;
  std::vector<RobotParticle> particles;
  for(unsigned int i = 0; i < N; i++) {
    particles.push_back(RobotParticle(
      Vector2d(
        boost::random::uniform_real_distribution<>(-course_length/2, course_length/2)(rng),
        boost::random::uniform_real_distribution<>(-course_width/2, course_width/2)(rng)),
      0,
      1));
  }
  
  ros::Publisher output_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  auto publish = [&particles, &output_pub](ros::Time stamp) {
    Vector2d position = Vector2d::Zero();
    BOOST_FOREACH(RobotParticle &particle, particles) {
      position += particle.getPosition();
    }
    position /= particles.size();
    
    Vector2d forward = Vector2d::Zero();
    BOOST_FOREACH(RobotParticle &particle, particles) {
      forward += particle.getForwardVector();
    }
    
    double angle = atan2(forward(1), forward(0));
    Eigen::Quaterniond orient(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));
    
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "/course";
    tf::pointEigenToMsg((Eigen::Vector3d() << position, 0).finished(), msg.pose.position);
    tf::quaternionEigenToMsg(orient, msg.pose.orientation);
    output_pub.publish(msg);
  };
  
  std::vector<std::unique_ptr<MessageOrderer::ITopicDescriptor> > topic_descriptors;
  topic_descriptors.emplace_back(new MessageOrderer::TopicDescriptor<geometry_msgs::PoseStamped>(
    "odom",
    [&particles, &publish](geometry_msgs::PoseStamped const &msg) {
      BOOST_FOREACH(RobotParticle &particle, particles) {
        particle = particle.predict(msg);
      }
      publish(msg.header.stamp);
    }));
  topic_descriptors.emplace_back(new MessageOrderer::TopicDescriptor<xv11_driver::LaserMeasurements>(
    "lidar",
    [&particles, &course](xv11_driver::LaserMeasurements const &msg) {
      BOOST_FOREACH(RobotParticle &particle, particles) {
        particle = particle.update(msg, course);
      }
    }));
  MessageOrderer mo(nh, topic_descriptors);
  
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("particles", 10);
  ros::Timer fixup_timer = nh.createTimer(ros::Duration(0.1), [&particles, &N, &pub](const ros::TimerEvent&){
    fixup(particles, N);
    
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/course";
    BOOST_FOREACH(RobotParticle const &particle, particles) {
      geometry_msgs::Pose pose;
      pose.position.x = particle.getPosition()(0);
      pose.position.y = particle.getPosition()(1);
      pose.position.z = 0;
      Eigen::Quaterniond orient(Eigen::AngleAxisd(particle.getAngle(), Eigen::Vector3d(0, 0, 1)));
      tf::quaternionEigenToMsg(orient, pose.orientation);
      msg.poses.push_back(pose);
    }
    pub.publish(msg);
  });
  
  ros::spin();
}
