package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.auton.FollowSlalomPath;
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
  public Trajectory trajectory;

  private SendableChooser<String> m_autonChooser;
  private static final String slalom = "slalom";
  private static final String barrelRacing = "barrelRacong";
  private static final String bouncePath = "bouncePath";

  public ProfiledPIDController thetaController;


  public SwerveControllerCommand swerveControllerCommand;

  public RobotContainer() {
    controller = Controller.getInstance();
    drivetrain = Drivetrain.getInstance();

    // TODO: Is there a way to not have 6 inputs and check driveModes another way?
    drivetrain.setDefaultCommand(new SwerveDrive(
      () -> controller.getForward(), 
      () -> controller.getStrafe(),
      () -> controller.getSpin(),
      () -> controller.isFieldCentricButtonPressed(),
      () -> controller.isRobotCentricButtonPressed(),
      () -> controller.isBackRobotCentricButtonPressed()));

    configureButtonBindings();

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.addOption("Slalom", slalom);
    //m_autonChooser.addOption("Barrel Racing", barrelRacing);
  //  m_autonChooser.addOption("Bounce Path", bouncePath);
    SmartDashboard.putData("Auton Chooser", m_autonChooser);
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
    TrajectoryConfig config =
    new TrajectoryConfig(
            DrivetrainConstants.MAX_WHEEL_SPEED_INCHES_PER_SECOND,
            DrivetrainConstants.MAX_ACCEL_INCHES_PER_SECOND)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS); 

       thetaController =
    new ProfiledPIDController(
        DrivetrainConstants.P_THETA_CONTROLLER, 0, 0, DrivetrainConstants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);           


    if(m_autonChooser.getSelected().equals("slalom")) {
      createTrajectory("slalom");
      new FollowSlalomPath();
    }
    /* else if(m_autonChooser.getSelected().equals("barrelRacing")) {
      return new ThreeSecondDriveAndShoot();
    } else if(m_autonChooser.getSelected().equals("bouncePath")) {
      return new FarShootAndDrive();
    }*/
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0));
  }
  public void createTrajectory(String path){
    String trajectoryJSON = "paths/" + path + ".wpilib.json";
   
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }
}



