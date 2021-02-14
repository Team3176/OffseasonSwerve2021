package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.SwerveDrive;
import frc.robot.constants.ControllerConstants;

import frc.robot.subsystems.Drivetrain;
public class RobotContainer {

  private Controller controller;
  private Drivetrain drivetrain;

  public RobotContainer() {
    controller = Controller.getInstance();
    drivetrain = Drivetrain.getInstance();

    drivetrain.setDefaultCommand(new SwerveDrive(
      () -> controller.getForward(), 
      () -> controller.getStrafe(),
      () -> controller.getSpin()));

    configureButtonBindings();
  }

  private void configureButtonBindings() { }
}
