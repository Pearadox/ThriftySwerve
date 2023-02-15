// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.ScoringTrajectories;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  public static final XboxController controller = new XboxController(0);
  private final JoystickButton resetHeading_B = new JoystickButton(controller, XboxController.Button.kB.value);
  private final JoystickButton autoScore_RB = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    resetHeading_B.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    autoScore_RB.onTrue(ScoringTrajectories.generateScoreCommand(drivetrain.getPose(), 7).until(() -> drivetrain.isDriving())
      .andThen(new InstantCommand(() -> RobotContainer.drivetrain.autoScoringFalse())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */

  public Command getAutonomousCommand(){
    // An ExampleCommand will run in autonomous
    PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
    PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
    PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath Copy", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectory, 
      drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController, 
      turnController, 
      drivetrain::setModuleStates,
      true,
      drivetrain);

    PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
      PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), Alliance.Red) :
      trajectory.getInitialState();

    Pose2d initialPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetOdometry(initialPose)),
      command,
      new InstantCommand(() -> drivetrain.stopModules())
    );
  }
}
