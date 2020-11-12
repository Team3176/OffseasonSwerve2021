/*
NEXT
Get the motor rotating on the translation stick

TO DO
- The swerve pod has an offset to it that needs fixed
- Tune the pod's PID loop
- The pod will turn 360 when the tics get too high because it goes from 4096 to 0

HISTORY
11/10: Got the code SEVERLY SNIPPED and transStickMag outputting to drive motor correctly
11/11: After working on rotation, realized I need to get rotation workong without fancy edits yet

*/

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  private Controller m_Controller = Controller.getInstance();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
