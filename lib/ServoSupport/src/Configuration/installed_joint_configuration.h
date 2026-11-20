/*
 * Installed joint configuration: stores measured HOME/OTHER angles for each joint.
 * Its getters supply the measurements and combine them with CAD references to
 * build a JointMapping. RR stays unavailable until its endpoints are captured.
 */
#ifndef INSTALLED_JOINT_CONFIGURATION_H
#define INSTALLED_JOINT_CONFIGURATION_H

#include "servo_configuration.h"
#include "../JointMapping/joint_mapping.h"

typedef struct {
    int available;          //1: both endpoints were captured; 0: not configured
    double servo_home_deg;  //a_0: measured servo angle at CAD home [deg]
    double servo_other_deg; //Measured other physical endpoint [deg]
} InstalledJointCalibration;

//Read-only stored measurements; NULL for invalid indices or missing endpoints.
const InstalledJointCalibration *installed_joint_calibration_for_joint(int leg, int joint);

//Build {q_0, a_0, sigma}. Returns 1 on success, 0 with output unchanged on failure.
int installed_joint_mapping_for_joint(int leg, int joint, JointMapping *mapping);

//These are raw measured endpoints, not limits with clearance from the stops.
//Capturing/exporting again does not automatically update this compiled configuration.
#endif // INSTALLED_JOINT_CONFIGURATION_H
