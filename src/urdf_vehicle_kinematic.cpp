
#include <tricycle_controller/urdf_vehicle_kinematic.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

static double euclideanOfVectors2D(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) );
}

static double distanceOfVectorsX(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2));
}

static double distanceOfVectorsY(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.y-vec2.y,2));
}

static double relativeRotationOfVectors(const urdf::Rotation& rotation_1, const urdf::Rotation& rotation_2)
{
  double roll_1, pitch_1, yaw_1;
  rotation_1.getRPY(roll_1, pitch_1, yaw_1);

  double roll_2, pitch_2, yaw_2;
  rotation_2.getRPY(roll_2, pitch_2, yaw_2);

  return yaw_2 - yaw_1;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const boost::shared_ptr<const urdf::Link>& link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius)
{
  if (!isCylinder(wheel_link))
  {
    ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
    return false;
  }

  wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
  return true;
}

namespace urdf_vehicle_kinematic{
  UrdfVehicleKinematic::UrdfVehicleKinematic(ros::NodeHandle& root_nh, const std::string& base_link):
    base_link_(base_link)
  {
    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
    }
    else
    {
      model_ = urdf::parseURDF(robot_model_str);
      if(!model_)
        ROS_ERROR_STREAM("Could not parse the urdf robot model "<<model_param_name);
    }
  }

  bool UrdfVehicleKinematic::getTransformVector(const std::string& joint_name, const std::string& parent_link_name
                                                , urdf::Vector3 &transform_vector)
  {
    if(model_)
    {
      boost::shared_ptr<const urdf::Joint> joint(model_->getJoint(joint_name));
      if (!joint)
      {
        ROS_ERROR_STREAM(joint_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      transform_vector = joint->parent_to_joint_origin_transform.position;
      while(joint->parent_link_name != parent_link_name)
      {
        boost::shared_ptr<const urdf::Link> link_parent(model_->getLink(joint->parent_link_name));
        if (!link_parent || !link_parent->parent_joint)
        {
          ROS_ERROR_STREAM(joint->parent_link_name
                                 << " couldn't be retrieved from model description or his parent joint");
          return false;
        }
        joint = link_parent->parent_joint;
        transform_vector = transform_vector + joint->parent_to_joint_origin_transform.position;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfVehicleKinematic::getTransformRotation(const std::string& joint_name, const std::string& parent_link_name
                                                , urdf::Rotation &transform_rotation)
  {
    if(model_)
    {
      boost::shared_ptr<const urdf::Joint> joint(model_->getJoint(joint_name));
      if (!joint)
      {
        ROS_ERROR_STREAM(joint_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      transform_rotation = joint->parent_to_joint_origin_transform.rotation;
      return true;

      //FIXME: Implement multi link parsing (+ operation for urdf::Rotation)

      while(joint->parent_link_name != parent_link_name)
      {
        boost::shared_ptr<const urdf::Link> link_parent(model_->getLink(joint->parent_link_name));
        if (!link_parent || !link_parent->parent_joint)
        {
          ROS_ERROR_STREAM(joint->parent_link_name
                                 << " couldn't be retrieved from model description or his parent joint");
          return false;
        }
        joint = link_parent->parent_joint;
        transform_rotation = transform_rotation * joint->parent_to_joint_origin_transform.rotation;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfVehicleKinematic::getDistanceBetweenJoints(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, base_link_, first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, base_link_, second_transform))
      return false;

    distance = euclideanOfVectors(first_transform,
                               second_transform);
    ROS_INFO_STREAM("first_transform : "<<first_transform.x<<","<<first_transform.y);
    ROS_INFO_STREAM("distance "<<distance);
    return true;
  }

  bool UrdfVehicleKinematic::getDistanceBetweenJoints2D(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, base_link_, first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, base_link_, second_transform))
      return false;

    distance = euclideanOfVectors2D(first_transform,
                               second_transform);
    ROS_INFO_STREAM("first_transform : "<<first_transform.x<<","<<first_transform.y);
    ROS_INFO_STREAM("distance "<<distance);
    return true;
  }

  bool UrdfVehicleKinematic::getDistanceBetweenJointsX(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, base_link_, first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, base_link_, second_transform))
      return false;

    distance = distanceOfVectorsX(first_transform,
                               second_transform);
    ROS_INFO_STREAM("first_transform : "<<first_transform.x<<","<<first_transform.y);
    ROS_INFO_STREAM("distance "<<distance);
    return true;
  }

  bool UrdfVehicleKinematic::getDistanceBetweenJointsY(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, base_link_, first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, base_link_, second_transform))
      return false;

    distance = distanceOfVectorsY(first_transform,
                               second_transform);
    ROS_INFO_STREAM("first_transform : "<<first_transform.x<<","<<first_transform.y);
    ROS_INFO_STREAM("distance "<<distance);
    return true;
  }

  bool UrdfVehicleKinematic::getRotationBetweenJoints(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& rotation)
  {
    urdf::Rotation first_transform;
    if(!getTransformRotation(first_joint_name, base_link_, first_transform))
    {
      return false;
    }

    urdf::Rotation second_transform;
    if(!getTransformRotation(second_joint_name, base_link_, second_transform))
    {
      return false;
    }

    rotation = relativeRotationOfVectors(first_transform, second_transform);
    return true;

  }


  bool UrdfVehicleKinematic::getJointRadius(const std::string& joint_name,
                                            double& radius)
  {
    if(model_)
    {
      boost::shared_ptr<const urdf::Joint> joint(model_->getJoint(joint_name));
      // Get wheel radius
      if (!getWheelRadius(model_->getLink(joint->child_link_name), radius))
      {
        ROS_ERROR_STREAM("Couldn't retrieve " << joint_name << " wheel radius");
        return false;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfVehicleKinematic::getJointSteeringLimits(const std::string& joint_name,
                              double& steering_limit)
  {
    if(model_)
    {
      boost::shared_ptr<const urdf::Joint> joint(model_->getJoint(joint_name));
      if(joint->type == urdf::Joint::REVOLUTE)
      {
        const double lower_steering_limit = fabs(joint->limits->lower);
        const double upper_steering_limit = fabs(joint->limits->upper);
        if(lower_steering_limit > upper_steering_limit)
          steering_limit = upper_steering_limit;
        else
          steering_limit = lower_steering_limit;
        ROS_INFO_STREAM("Joint "<<joint_name<<" steering limit is "<<steering_limit*180.0/M_PI<<" in degrees");
        return true;
      }
      ROS_ERROR_STREAM("Couldn't get joint "<<joint_name<<" steering limit, is it of type REVOLUTE ?");
    }
    return false;
  }
}
