package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ArcadeDrive;
import frc.robot.constants.DrivetrainConstants;

public class RobotContainer {

  private Controller m_Controller;
  private Drivetrain m_Drivetrain;

  public RobotContainer() {
    m_Controller = Controller.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
      () -> m_Controller.getRotStickX(), 
      () -> m_Controller.getTransStickMag(),
      () -> m_Controller.getTransStickAngleRads()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  /*
  public Command getAutonomousCommand() {
    
  }
  */
}
