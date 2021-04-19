package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.auton.FollowGivenPath;
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
  public static Drivetrain drivetrain;

  public Trajectory trajectory;

  private SendableChooser<String> m_autonChooser;
  private static final String slalom = "slalom";
  private static final String easy = "easy";
  private static final String forward = "forward";
  private static final String forward_and_back = "forward_and_back";
  private static final String L_shape = "L_shape";
  

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
    m_autonChooser.addOption("slalom", slalom);
    m_autonChooser.addOption("easy", easy);
  //  m_autonChooser.addOption("Bounce Path", bouncePath);
    m_autonChooser.addOption("forward", forward);
    m_autonChooser.addOption("forward_and_back", forward_and_back);
    m_autonChooser.addOption("L_shape", L_shape);
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
    /*TrajectoryConfig config =
        new TrajectoryConfig(
               4.468,
                1.631)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

    */
          
       thetaController =
    new ProfiledPIDController(
        DrivetrainConstants.P_THETA_CONTROLLER, 0, 0, DrivetrainConstants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);           


    if(m_autonChooser.getSelected().equals("slalom")) {
      createTrajectory("slalom");
      new FollowGivenPath(trajectory);
    }

    else if(m_autonChooser.getSelected().equals("easy")) {
      createTrajectory("easy");
      return new FollowGivenPath(trajectory);
    } 
    /*
    else if(m_autonChooser.getSelected().equals("bouncePath")) {
      return new FarShootAndDrive();
    }
    */
    else if(m_autonChooser.getSelected().equals("forward")) {
      //createTrajectory("L_shape");
      //return new FollowGivenPath(trajectory);
      /*String trajectoryJSON = "paths/forward.wpilib.json";
   trajectory = null;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }*/
    TrajectoryConfig config =
        new TrajectoryConfig(
                4.48,
                1.631)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(3, 0), new Translation2d(6,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d()),
            config);



drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
      swerveControllerCommand =
    new SwerveControllerCommand(
        exampleTrajectory,
        drivetrain::getCurrentPose, 
        DrivetrainConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(DrivetrainConstants.P_X_Controller, 0, 0),
        new PIDController(DrivetrainConstants.P_Y_Controller, 0, 0),
        thetaController,
        drivetrain::setModuleStates, //Not sure about setModuleStates
        drivetrain);
        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
        
        
        try {
        drivetrain.bw.close();
        drivetrain.fw.close();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
        
// Reset odometry to the starting pose of the trajectory.
//drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    }
    
    else if(m_autonChooser.getSelected().equals("forward_and_back")) {
      createTrajectory("forward_and_back");
      return new FollowGivenPath(trajectory);
    }

    else if(m_autonChooser.getSelected().equals("L_shape")) {
      createTrajectory("L_shape");
      return new FollowGivenPath(trajectory);
    }

    if(swerveControllerCommand == null) { System.out.println("long thing is null"); }
    if(drivetrain == null) { System.out.println("drivetrain is null"); }
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0));
  }
  
  public void createTrajectory(String path){
    String trajectoryJSON = "paths/" + path + ".wpilib.json";
   trajectory = null;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    
  }
}




