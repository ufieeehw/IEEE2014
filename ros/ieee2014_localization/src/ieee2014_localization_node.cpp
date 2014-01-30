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


using Eigen::Vector2d;

static double const pi = boost::math::constants::pi<double>();
boost::random::mt19937 rng;

boost::random::uniform_real_distribution<> fake_normal_distribution(double mean, double stddev) {
  return boost::random::uniform_real_distribution<>(mean - sqrt(3)*stddev, mean + sqrt(3)*stddev);
}

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

double yaw_from_quaternion(geometry_msgs::Quaternion q) {
  Eigen::Quaterniond orient; tf::quaternionMsgToEigen(q, orient);
  Eigen::Vector3d new_forward = orient._transformVector(Eigen::Vector3d(1, 0, 0));
  return atan2(new_forward(1), new_forward(0));
}
geometry_msgs::Quaternion quaternion_from_yaw(double yaw) {
  Eigen::Quaterniond orient(Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1)));
  geometry_msgs::Quaternion res; tf::quaternionEigenToMsg(orient, res);
  return res;
}

static double sinc(double x) {
  if(fabs(x) < 1e-6) {
    return 1 - x*x/6 + x*x*x*x/120;
  }
  return sin(x) / x;
}
static double cosm1c(double x) {
  if(fabs(x) < 1e-6) {
    return -x/2 + x*x*x/24 - x*x*x*x*x/720;
  }
  return (cos(x) - 1) / x;
}

Eigen::Matrix2d drive_mat(double omega, double dt) {
  return (Eigen::Matrix2d() <<
    sinc(dt*omega), cosm1c(dt*omega),
    -cosm1c(dt*omega), sinc(dt*omega)).finished() * dt;
}

std::pair<Vector2d, double> drive(Vector2d v, double omega, double dt) {
  Vector2d dp = drive_mat(omega, dt) * v;
  double dtheta = dt * omega;
  return std::make_pair(dp, dtheta);
}
std::pair<Vector2d, double> undrive(Vector2d dp, double dtheta, double dt) {
  assert(dt != 0);
  double omega = dtheta/dt;
  Vector2d v = drive_mat(omega, dt).inverse() * dp;
  return std::make_pair(v, omega);
}

class RobotParticle {
  Vector2d position;
  double angle;

public:
  RobotParticle(Vector2d position, double angle) :
    position(position), angle(angle) {
    assert(std::isfinite(position(0)) &&
           std::isfinite(position(1)) &&
           std::isfinite(angle));
  }
  
  Vector2d getPosition() const {
    return position;
  }
  double getAngle() const {
    return angle;
  }
  Vector2d getForwardVector() const {
    return Vector2d(cos(angle), sin(angle));
  }
  Vector2d getLeftVector() const {
    return Vector2d(cos(angle+pi/2), sin(angle+pi/2));
  }
  
  RobotParticle predict(double dt, Vector2d v, double omega) const {
    assert(dt >= 0);
    std::pair<Vector2d, double> dp_and_dtheta = drive(v, omega, dt);
    
    return RobotParticle(
      position + (
          getForwardVector() * dp_and_dtheta.first(0) +
             getLeftVector() * dp_and_dtheta.first(1)
        ) + Vector2d(
          fake_normal_distribution(0, sqrt(.001*dt))(rng),
          fake_normal_distribution(0, sqrt(.001*dt))(rng)
        ),
      angle +
        dp_and_dtheta.second +
        fake_normal_distribution(0, sqrt(.001*dt))(rng));
  }
  
  double update(xv11_driver::LaserMeasurements const &lidar_msg, std::vector<std::unique_ptr<IObject> > const &course) const {
    double laser_angle = lidar_msg.angle_min;
    
    Vector2d start = position;
    Vector2d direction = Vector2d(cos(angle + laser_angle), sin(angle + laser_angle));
    
    double dist = std::numeric_limits<double>::infinity();
    BOOST_FOREACH(std::unique_ptr<IObject> const &objp, course) {
      dist = std::min(dist, objp->intersectWithRay(start, direction));
    }
    
    double measured = lidar_msg.ranges[0];
    
    if(!isinf(dist) && !isinf(measured)) {
      return .1 + .9*exp(-pow(log(dist/measured), 2));
    } else {
      return .1;
    }
  }
};


class ParticleFilter {
  static unsigned int const N = 200;
  
  ros::Time t;
  std::vector<RobotParticle> particles;

public:
  ParticleFilter(ros::Time t, double course_length, double course_width) :
    t(t) {
    for(unsigned int i = 0; i < N; i++) {
      particles.push_back(RobotParticle(
        Vector2d(
          boost::random::uniform_real_distribution<>(-course_length/2, course_length/2)(rng),
          boost::random::uniform_real_distribution<>(-course_width/2, course_width/2)(rng)),
        boost::random::uniform_real_distribution<>(-1, 1)(rng)));
    }
  }
  
  void update(geometry_msgs::PoseStamped const &odom,
              std::vector<xv11_driver::LaserMeasurements> const &lidars,
              std::vector<std::unique_ptr<IObject> > const &course) {
    
    double dt = (odom.header.stamp - t).toSec();
    std::pair<Vector2d, double> v_and_omega = undrive(
      Vector2d(odom.pose.position.x, odom.pose.position.y),
      yaw_from_quaternion(odom.pose.orientation),
      dt);
    
    typedef std::pair<double, RobotParticle> WeightedParticle;
    std::vector<WeightedParticle> weighted_particles;
    BOOST_FOREACH(RobotParticle const &particle, particles) {
      RobotParticle tmp = particle;
      ros::Time tmp_t = t;
      double weight = 1;
      BOOST_FOREACH(xv11_driver::LaserMeasurements const &lidar, lidars) {
        tmp = tmp.predict((lidar.header.stamp - tmp_t).toSec(), v_and_omega.first, v_and_omega.second);
        tmp_t = lidar.header.stamp;
        weight *= tmp.update(lidar, course);
      }
      tmp = tmp.predict((odom.header.stamp - tmp_t).toSec(), v_and_omega.first, v_and_omega.second);
      tmp_t = odom.header.stamp;
      weighted_particles.push_back(std::make_pair(weight, tmp));
    }
    
    double total_weight = 0;
    BOOST_FOREACH(WeightedParticle const &weighted_particle, weighted_particles)
      total_weight += weighted_particle.first;
    
    particles.clear();
    BOOST_FOREACH(WeightedParticle const &weighted_particle, weighted_particles) {
      unsigned int new_count = N * weighted_particle.first/total_weight + boost::random::uniform_01<>()(rng);
      for(unsigned int i = 0; i < new_count; i++) {
        particles.push_back(RobotParticle(weighted_particle.second.getPosition(), weighted_particle.second.getAngle()));
      }
    }
    t = odom.header.stamp;
  }
  
  geometry_msgs::PoseStamped getPose() const {
    Vector2d position = Vector2d::Zero();
    BOOST_FOREACH(RobotParticle const &particle, particles) {
      position += particle.getPosition();
    }
    position /= particles.size();
    
    Vector2d forward = Vector2d::Zero();
    BOOST_FOREACH(RobotParticle const &particle, particles) {
      forward += particle.getForwardVector();
    }
    
    double angle = atan2(forward(1), forward(0));
    
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = "/course";
    tf::pointEigenToMsg((Eigen::Vector3d() << position, 0).finished(), msg.pose.position);
    msg.pose.orientation = quaternion_from_yaw(angle);
    return msg;
  }
  
  geometry_msgs::PoseArray getParticles() const {
    geometry_msgs::PoseArray msg;
    msg.header.stamp = t;
    msg.header.frame_id = "/course";
    BOOST_FOREACH(RobotParticle const &particle, particles) {
      geometry_msgs::Pose pose;
      pose.position.x = particle.getPosition()(0);
      pose.position.y = particle.getPosition()(1);
      pose.position.z = 0;
      pose.orientation = quaternion_from_yaw(particle.getAngle());
      msg.poses.push_back(pose);
    }
    return msg;
  }
};

class MessageHandling {
  static constexpr double const course_length = (97 - 3/4. * 2) * 0.0254;
  static constexpr double const course_width = (49 - 3/4. * 2) * 0.0254;
  std::vector<std::unique_ptr<IObject> > course;
  
  ros::Subscriber odom_sub;
  ros::Subscriber lidar_sub;
  
  ros::Publisher pose_pub;
  ros::Publisher particles_pub;
  
  std::list<geometry_msgs::PoseStamped> odom_msgs;
  std::list<xv11_driver::LaserMeasurements> lidar_msgs;
  
  boost::optional<ParticleFilter> filter;
  
  void handle_odom(geometry_msgs::PoseStamped const &msg) {
    assert(odom_msgs.empty() || msg.header.stamp > odom_msgs.back().header.stamp);
    odom_msgs.push_back(msg);
    think();
  }
  
  void handle_lidar(xv11_driver::LaserMeasurements const &msg) {
    // XXX should expand to one per ray
    assert(lidar_msgs.empty() || msg.header.stamp > lidar_msgs.back().header.stamp);
    lidar_msgs.push_back(msg);
    think();
  }
  
  void think() {
    // if there's an odom message with a lidar message following it
    while(!odom_msgs.empty() && !lidar_msgs.empty() &&
          lidar_msgs.back().header.stamp > odom_msgs.front().header.stamp) {
      {
        // pop the odom message and lidar messages before it, send with list of following lidar messages
        geometry_msgs::PoseStamped odom = odom_msgs.front(); odom_msgs.pop_front();
        
        std::vector<xv11_driver::LaserMeasurements> lidars;
        while(lidar_msgs.front().header.stamp < odom.header.stamp) {
          lidars.push_back(lidar_msgs.front()); lidar_msgs.pop_front();
        }
        
        if(!filter) {
          filter = ParticleFilter(odom.header.stamp, course_length, course_width);
          continue;
        }
        
        filter->update(odom, lidars, course);
      }
      
      {
        ParticleFilter tmp = *filter;
        BOOST_FOREACH(geometry_msgs::PoseStamped const &future_odom, odom_msgs) {
          tmp.update(future_odom, std::vector<xv11_driver::LaserMeasurements>(), course);
        }
        pose_pub.publish(tmp.getPose());
        particles_pub.publish(tmp.getParticles());
      }
    }
  }
  
public:
  MessageHandling(ros::NodeHandle &nh) {
    course.emplace_back(new Line(Vector2d(-course_length/2, +course_width/2), Vector2d(+course_length/2, +course_width/2)));
    course.emplace_back(new Line(Vector2d(+course_length/2, +course_width/2), Vector2d(+course_length/2, -course_width/2)));
    course.emplace_back(new Line(Vector2d(+course_length/2, -course_width/2), Vector2d(-course_length/2, -course_width/2)));
    course.emplace_back(new Line(Vector2d(-course_length/2, -course_width/2), Vector2d(-course_length/2, +course_width/2)));
    
    odom_sub = nh.subscribe("odom", 10, &MessageHandling::handle_odom, this);
    lidar_sub = nh.subscribe("lidar", 10, &MessageHandling::handle_lidar, this);
    
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    particles_pub = nh.advertise<geometry_msgs::PoseArray>("particles", 10);
  }
};
constexpr double const MessageHandling::course_length;
constexpr double const MessageHandling::course_width;


int main(int argc, char **argv) {
  ros::init(argc, argv, "ieee2014_localization");
  
  ros::NodeHandle nh;
  
  MessageHandling mh(nh);
  
  ros::spin();
}
