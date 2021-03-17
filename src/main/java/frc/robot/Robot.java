package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  private Controller controller = Controller.getInstance();

 

  @Override
  public void robotInit() {
    
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(robotContainer.drivetrain == null){
      System.out.println("YEs it is null");
    }
  }

  @Override
  public void disabledInit() { }

  @Override
  public void disabledPeriodic() { }

  @Override
  public void autonomousInit() { }

  @Override
  public void autonomousPeriodic() { }

  @Override
  public void teleopInit() { }

  @Override
  public void teleopPeriodic() {
    controller.outputToSmartDashboard();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() { }


  
  
}
