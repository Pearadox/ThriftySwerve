// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private TalonFX driveMotor;
  private PearadoxSparkMax turnMotor;

  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANCoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;

  private Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      absoluteEncoder = new CANCoder(absoluteEncoderId);

      driveMotor = new TalonFX(driveMotorId);
      turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      driveMotor.setInverted(driveMotorReversed);
      turnMotor.setInverted(turnMotorReversed);

      driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      turnEncoder = turnMotor.getEncoder();

      driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));

      turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setNeutralMode(NeutralMode.Brake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setNeutralMode(NeutralMode.Coast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getDriveMotorPosition(){
    return driveMotor.getSelectedSensorPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveMotor.getSelectedSensorVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition();
    angle -= absoluteEncoderOffset;
    angle *= (Math.PI / 180);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveMotor.setSelectedSensorPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }
  
  public void stop(){
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  private void setSpeed(SwerveModuleState desiredState){
        driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);
    // else {
    //     double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    //     mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    // }
  }

  private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    lastAngle = angle;
  }
}
