// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    public PhotonCameraWrapper() throws IOException{
        AprilTagFieldLayout atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

        photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.ROBOT_TO_CAM);
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
