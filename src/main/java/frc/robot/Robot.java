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

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  private Controller controller = Controller.getInstance();

  public Trajectory slalomTrajectory;

  @Override
  public void robotInit() {
    String slalomJSON = "paths/Slalom.wpilib.json";
    Trajectory slalomTrajectory = new Trajectory();
    try {
      Path slalomPath = Filesystem.getDeployDirectory().toPath().resolve(slalomJSON);
      slalomTrajectory = TrajectoryUtil.fromPathweaverJson(slalomPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + slalomJSON, ex.getStackTrace());
    }
    robotContainer = new RobotContainer();
  }
public Trajectory getTrajectory(){
  return slalomTrajectory;
}
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
