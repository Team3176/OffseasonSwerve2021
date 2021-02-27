package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.teleop.SwerveDefense;
import frc.robot.commands.teleop.SwerveDrive;
import frc.robot.commands.teleop.SwerveReZeroGyro;
import frc.robot.commands.teleop.SwerveVision;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
public class RobotContainer {

  private Joystick xboxController = new Joystick(ControllerConstants.XBOX_CONTROLLER_ID);

  private Controller controller;
  private Drivetrain drivetrain;
  private Robot robot;
  private Trajectory trajectory;

  public RobotContainer() {
    controller = Controller.getInstance();
    drivetrain = Drivetrain.getInstance();
    trajectory = robot.getTrajectory();

    // TODO: Is there a way to not have 6 inputs and check driveModes another way?
    drivetrain.setDefaultCommand(new SwerveDrive(
      () -> controller.getForward(), 
      () -> controller.getStrafe(),
      () -> controller.getSpin(),
      () -> controller.isFieldCentricButtonPressed(),
      () -> controller.isRobotCentricButtonPressed(),
      () -> controller.isBackRobotCentricButtonPressed()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Drivetrain buttons
    controller.getDefenseButton().whenHeld(new SwerveDefense());
    controller.getVisionButton().whenHeld(new SwerveVision(
      () -> controller.getForward(), 
      () -> controller.getStrafe()));
    controller.getReZeroGyroButton().whenHeld(new SwerveReZeroGyro());
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                DrivetrainConstants.MAX_WHEEL_SPEED_INCHES_PER_SECOND,
                DrivetrainConstants.MAX_ACCEL_INCHES_PER_SECOND)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

            var thetaController =
        new ProfiledPIDController(
            DrivetrainConstants.P_THETA_CONTROLLER, 0, 0, DrivetrainConstants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
        trajectory,
        drivetrain::getPose, // Functional interface to feed supplier
        DrivetrainConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(DrivetrainConstants.P_X_Controller, 0,0),
        new PIDController(DrivetrainConstants.P_Y_Controller, 0, 0),
        thetaController,
        drivetrain::setModuleStates,
        drivetrain);

// Reset odometry to the starting pose of the trajectory.
drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0));
}
}



