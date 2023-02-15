// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PhotonCameraWrapper;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront = new SwerveModule(
    SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    SwerveConstants.LEFT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    SwerveConstants.LEFT_FRONT_OFFSET, 
    false);

  private SwerveModule rightFront = new SwerveModule(
    SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    SwerveConstants.RIGHT_FRONT_OFFSET, 
    false);

  private SwerveModule leftBack = new SwerveModule(
    SwerveConstants.LEFT_BACK_DRIVE_ID, 
    SwerveConstants.LEFT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_BACK_CANCODER_ID, 
    SwerveConstants.LEFT_BACK_OFFSET, 
    false);

  private SwerveModule rightBack = new SwerveModule(
    SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    SwerveConstants.RIGHT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    SwerveConstants.RIGHT_BACK_OFFSET, 
    false);

  private SlewRateLimiter frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private Pose2d pose = new Pose2d();
  private Pose2d posetest = new Pose2d(new Translation2d(12.65, 4.42), new Rotation2d(0));
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    SwerveConstants.DRIVE_KINEMATICS, 
    getHeadingRotation2d(),
    getModulePositions(), 
    new Pose2d());

  private PhotonCameraWrapper pcw;

  private boolean autoScoring = false;

  private static final Drivetrain drivetrain = new Drivetrain();

  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    try {
      pcw = new PhotonCameraWrapper();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    if(isDriving()){
      autoScoring = false;
    }
    SmartDashboard.putNumber("Robot Angle", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());

    SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((gyro.getRate() / 180)) + "pi rad/s");
  }
  
  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, boolean fieldOriented, boolean deadband){
    SmartDashboard.putNumber("X Speed", frontSpeed);
    SmartDashboard.putNumber("Y Speed", sideSpeed);
    SmartDashboard.putNumber("Turn Speed", turnSpeed);

    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = RobotContainer.controller.getLeftTriggerAxis() > 0.9 ? frontSpeed * 0.6 : frontSpeed;
    sideSpeed = RobotContainer.controller.getLeftTriggerAxis() > 0.9 ? sideSpeed * 0.6 : sideSpeed;
    turnSpeed = RobotContainer.controller.getLeftTriggerAxis() > 0.9 ? turnSpeed * 0.6 : turnSpeed;

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }
  
  public Pose2d getPose(){
    return pose;
  }

  public void resetOdometry(Pose2d pose){
    poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public void updateOdometry() {
    poseEstimator.update(getHeadingRotation2d(), getModulePositions());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

    if (result.isPresent() && !autoScoring) {
        EstimatedRobotPose camPose = result.get();
        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds); //tune
        resetOdometry(poseEstimator.getEstimatedPosition());
    }

    pose = poseEstimator.getEstimatedPosition();
  }

  public Pose2d flipPose(){
    Pose2d flippedPose = new Pose2d(
      new Translation2d(
        Constants.FIELD_LENGTH_METERS - posetest.getX(),
        posetest.getY()
      ),
      Rotation2d.fromDegrees(Math.IEEEremainder(posetest.getRotation().getDegrees() + 180, 360))
    );
    return flippedPose;
  }

  public void setAllMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  } 

  public void autoScoringTrue(){
    autoScoring = true;
  }

  public void autoScoringFalse(){
    autoScoring = false;
  }

  public boolean isDriving(){
    return Math.abs(RobotContainer.controller.getLeftY()) > 0.1 ||
      Math.abs(RobotContainer.controller.getLeftX()) > 0.1 ||
      Math.abs(RobotContainer.controller.getRightX()) > 0.1;
  }
}
