package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.SwerveDrive;
import frc.robot.commands.SwerveNeutral;

import frc.robot.constants.ControllerConstants;

import frc.robot.subsystems.Drivetrain;
public class RobotContainer {

  private Controller controller;
  private Drivetrain drivetrain;

  private final Joystick transStick;
  private final Joystick rotStick;

  public RobotContainer() {
    controller = Controller.getInstance();
    drivetrain = Drivetrain.getInstance();

    transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
    rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);

    drivetrain.setDefaultCommand(new SwerveDrive(
      () -> controller.getTransStickY(), 
      () -> controller.getTransStickX(),
      () -> controller.getRotStickX()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    controller.getSwerveNeutralButton().whenHeld(new SwerveNeutral());
  }
}
