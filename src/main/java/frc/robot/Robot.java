package frc.robot;

import frc.robot.commands.auton.Slalom;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private Slalom slalom;

  private Controller controller = Controller.getInstance();

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    SmartDashboard.putNumber("runTime", 0.5);
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
  public void autonomousInit() { 
    //slalom = new Slalom();
  }

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
