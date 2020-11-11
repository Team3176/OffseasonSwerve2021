package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.SwerveDrive;

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

  private void configureButtonBindings() {}
}
