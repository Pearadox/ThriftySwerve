// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class ScoringTrajectories {
    static ArrayList<PathPoint> scoringPathPoints = new ArrayList<PathPoint>(){
        {   
            add(new PathPoint(new Translation2d(1.60, 5.00), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 4.42), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 3.87), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 3.30), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 2.74), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 2.18), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 1.63), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 1.06), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
            add(new PathPoint(new Translation2d(1.60, 0.51), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));
        }
    };
    
    public static Command generateScoreCommand(Pose2d pose, int pathNumber){
        PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
        PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
        PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        pathNumber = DriverStation.getAlliance().equals(Alliance.Red) ? 8 - pathNumber : pathNumber;

        Pose2d robotPose = DriverStation.getAlliance().equals(Alliance.Red) ? 
            RobotContainer.drivetrain.flipPose() : 
            RobotContainer.drivetrain.getPose();

        SmartDashboard.putString("RobotPose", robotPose.toString());

        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION), 
            new PathPoint(robotPose.getTranslation(), 
                Rotation2d.fromDegrees(180), 
                robotPose.getRotation()),
            scoringPathPoints.get(pathNumber));

        SmartDashboard.putString("Path Start", traj.getInitialState().toString());
        SmartDashboard.putString("Path End", traj.getEndState().toString());

        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            traj,
            RobotContainer.drivetrain::getPose, // Pose2d supplier
            SwerveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
            frontController,
            sideController,
            turnController,
            RobotContainer.drivetrain::setModuleStates,
            true,
            RobotContainer.drivetrain
        );

        PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
        PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(), Alliance.Red) :
        traj.getInitialState();
  
        Pose2d initialPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(initialPose)),
            new InstantCommand(() -> RobotContainer.drivetrain.autoScoringTrue()),
            command,
            new InstantCommand(() -> RobotContainer.drivetrain.autoScoringFalse()),
            new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
        );
    }
}
