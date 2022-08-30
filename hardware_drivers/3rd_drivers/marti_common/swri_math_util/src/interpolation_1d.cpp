// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#include <swri_math_util/interpolation_1d.h>

namespace swri_math_util
{
Interpolation1D::Interpolation1D()
  :
  interp_type_(ZERO_ORDER_HOLD)
{
}

bool Interpolation1D::appendPoint(double x, double y)
{
  if (x_.size() == 0 || x > x_.back()) {
    // We can accept new points if it is the first point or greater
    // than the last point.
    x_.push_back(x);
    y_.push_back(y);
    return true;
  } else {
    ROS_ERROR("Error appending new point. "
              "X values must be increasing. (%f <= %f)",
              x, x_.back());
    return false;
  } 
}

size_t Interpolation1D::numPoints() const
{
  return x_.size();
}

std::pair<double, double> Interpolation1D::getPoint(size_t index) const
{
  if (index < x_.size()) {
    return std::pair<double, double>(x_[index], y_[index]);
  } else {
    ROS_ERROR("Invalid index in getPoint (index=%zu, numPoints=%zu)",
              index, x_.size());
    return std::pair<double, double>(0.0, 0.0);
  }
}

void Interpolation1D::removePoint(size_t index)
{
  if (index < x_.size()) {
    x_.erase(x_.begin()+index);
    y_.erase(y_.begin()+index);
  } else {
    ROS_ERROR("Invalid index in removePoint (index=%zu, numPoints=%zu)",
              index, x_.size());
  }
}

void Interpolation1D::clear()
{
  x_.clear();
  y_.clear();
}

Interpolation1D::InterpolationType Interpolation1D::interpolationType()
{
  return interp_type_;
}

std::string Interpolation1D::interpolationTypeString() const
{
  if (interp_type_ == ZERO_ORDER_HOLD) {
    return "zero_order_hold";
  } else if (interp_type_ == LINEAR) {
    return "linear";
  } else {
    return "<unknown>";
  }
}

void Interpolation1D::setInterpolationType(InterpolationType type)
{
  interp_type_ = type;
}

bool Interpolation1D::readFromParameter(
  const ros::NodeHandle &node_handle,
  const std::string &param_name,
  bool error_if_missing)
{
  std::string resolved_name = node_handle.resolveName(param_name);

  XmlRpc::XmlRpcValue curve_param;
  if (node_handle.getParam(param_name, curve_param)) {
    return readFromParameter(curve_param, param_name);
  } else {
    if (error_if_missing) {
      ROS_ERROR("Missing required parameter at '%s'.", resolved_name.c_str());
      return false;
    } else {
      return true;
    }
  }  
}

bool Interpolation1D::readFromParameter(
  XmlRpc::XmlRpcValue &curve_param,
  const std::string &param_name)
{
  if (curve_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("Parameter '%s' must be a structure.", param_name.c_str());
    return false;
  }

  bool error_occurred = false;
  
  if (curve_param.hasMember("interpolation_type")) {
    std::string interp_type = static_cast<std::string>(curve_param["interpolation_type"]);

    if (interp_type == "zero_order_hold") {
      setInterpolationType(ZERO_ORDER_HOLD);
    } else if (interp_type == "linear") {
      setInterpolationType(LINEAR);
    } else {
      error_occurred = true;
      ROS_ERROR("Invalid interpolation type '%s' at '%s'.",
                interp_type.c_str(), param_name.c_str());
    }          
  } else {
    ROS_INFO("No 'interpolation_type' found in %s. "
             "Defaulting to zero_order_hold.",
             param_name.c_str());
    setInterpolationType(ZERO_ORDER_HOLD);
  }

  if (!curve_param.hasMember("values")) {
    ROS_INFO("Missing required parameter '%s/values'.",
             param_name.c_str());
    return false;    
  }

  XmlRpc::XmlRpcValue param_value = curve_param["values"];    
  if (param_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Parameter '%s/values' must be an array.", param_name.c_str());
    return false;
  }

  for (int i = 0; i < param_value.size(); i++)
  {
    XmlRpc::XmlRpcValue point_value = param_value[i];

    if (point_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter '%s/values[%d]' must be an array.",
                param_name.c_str(), i);
      error_occurred = true;
      continue;
    }

    if (point_value.size() != 2) {
      ROS_ERROR("Parameter '%s/values[%d]' must be 2 elements.",
                param_name.c_str(), i);
      error_occurred = true;
      continue;
    }

    XmlRpc::XmlRpcValue x_param = point_value[0];
    XmlRpc::XmlRpcValue y_param = point_value[1];
    double x_value = std::numeric_limits<double>::quiet_NaN();
    double y_value = std::numeric_limits<double>::quiet_NaN();
    
    if (x_param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        x_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Parameters '%s/values[%d][0] must be a double or an integer.",
                param_name.c_str(), i);
      error_occurred = true;
    } else {
      x_value = static_cast<double>(x_param);
    }

    if (y_param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        y_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Parameters '%s/values[%d][1] must be a double or an integer.",
                param_name.c_str(), i);
      error_occurred = true;
    } else {
      y_value = static_cast<double>(y_param);
    }
    
    if (!error_occurred) {
      if (!appendPoint(x_value, y_value)) {
        ROS_ERROR("Failed to add point %s/values[%d].",
                  param_name.c_str(), i);
        error_occurred = true;
      }
    }
  }

  if (error_occurred) {
    clear();
    return false;
  } else {
    return true;
  }
}

double Interpolation1D::minX() const
{
  if (x_.size() == 0) {
    return 0.0;
  } else {
    return x_.front();
  }
}

double Interpolation1D::maxX() const
{
  if (x_.size() == 0) {
    return 0.0;
  } else {
    return x_.back();
  }
}
}  // namespace swri_math_util
