/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 */
#include <turbojpeg.h>

#include <fstream>
#include <string>

#include "smart_plugin/traffic_info.h"
#include "xproto/msg_type/protobuf/x3.pb.h"

namespace xproto {

void setAttribute(x3::Attributes* attribute,
                  std::string type, float value,
                  std::string value_string, float score);

// VehicleInfo
void convertVehicleInfo(
    x3::Target* target,
    const xproto::VehicleInfo& vehicle_info,
    float x_ratio, float y_ratio);

// Nonmotor
void convertNonmotor(
    x3::Target* target,
    const xproto::NoMotorVehicleInfo& nomotor_info,
    float x_ratio, float y_ratio);

// Person
void convertPerson(x3::Target* target,
                   const xproto::PersonInfo& person_info,
                   float x_ratio, float y_ratio);

}  // namespace xproto
