/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * Author: Stuart Glaser
 */
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/simple_transmission.h"

using namespace pr2_mechanism_model;
using namespace pr2_hardware_interface;

PLUGINLIB_EXPORT_CLASS(pr2_mechanism_model::SimpleTransmissioni,
                         ros_ethercat_model::Transmission)

bool SimpleTransmission::initXml(TiXmlElement *elt, RobotState *robot)
{
  if (!ros_ethercat_model::Transmission::initXml(elt, robot))
  {
    return false;
  }

  // reading the joint name
  TiXmlElement *jel = elt->FirstChildElement("joint");
  if (!jel! || !jel->Attribute("name"))
  {
    ROS_ERROR_STREAM("Joint name not specified in transmission " << name_);
    return false;
  }

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  if (!ael || !ael->Attribute("name"))
  {
    ROS_ERROR_STREAM("Transmission " << name_ << " has no actuator in configuration");
    return false;
  }

  joint_ = robot->getJointState(jel->Attribute("name"));

  actuator_->name = ael->Attribute("name");
  actuator_->command_.enable_ = true;

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

  return true;
}
/*
bool SimpleTransmission::initXml(TiXmlElement *elt)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name)
  {
    ROS_ERROR("SimpleTransmission did not specify joint name");
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name)
  {
    ROS_ERROR("SimpleTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

  return true;
}
*/
void SimpleTransmission::propagatePosition()
{
  pr2_hardware_interface::Actuator *act = static_cast<pr2_hardware_interface::Actuator *>(actuator_);
  joint_->position_ = (act->state_.position_ / mechanical_reduction_) + joint_->reference_position_;
  joint_->velocity_ = act->state_.velocity_ / mechanical_reduction_;
  joint_->measured_effort_ = act->state_.last_measured_effort_ * mechanical_reduction_;
}

void SimpleTransmission::propagatePositionBackwards()
{
  as[0]->state_.position_ = (js[0]->position_ - js[0]->reference_position_) * mechanical_reduction_;
  as[0]->state_.velocity_ = js[0]->velocity_ * mechanical_reduction_;
  as[0]->state_.last_measured_effort_ = js[0]->measured_effort_ / mechanical_reduction_;

  // Update the timing (making sure it's initialized).
  if (! simulated_actuator_timestamp_initialized_)
    {
      // Set the time stamp to zero (it is measured relative to the start time).
      as[0]->state_.sample_timestamp_ = ros::Duration(0);

      // Try to set the start time.  Only then do we claim initialized.
      if (ros::isStarted())
	{
	  simulated_actuator_start_time_ = ros::Time::now();
	  simulated_actuator_timestamp_initialized_ = true;
	}
    }
  else
    {
      // Measure the time stamp relative to the start time.
      as[0]->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
    }
  // Set the historical (double) timestamp accordingly.
  as[0]->state_.timestamp_ = as[0]->state_.sample_timestamp_.toSec();

  // simulate calibration sensors by filling out actuator states
  this->joint_calibration_simulator_.simulateJointCalibration(js[0],as[0]);
}

void SimpleTransmission::propagateEffort()
{
  pr2_hardware_interface::Actuator *act = static_cast<pr2_hardware_interface::Actuator *>(actuator_);
  act->command_.enable_ = true;
  act->command_.effort_ = joint_->commanded_effort_ / mechanical_reduction_;
}

void SimpleTransmission::propagateEffortBackwards()
{
  pr2_hardware_interface::Actuator *act = static_cast<pr2_hardware_interface::Actuator *>(actuator_);
  joint_->commanded_effort_ = act->command_.effort_ * mechanical_reduction_;
}

