package frc.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class SwerveResetGyro extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private boolean isReset;

  public SwerveResetGyro() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.resetGyro();
    isReset = true;
  }

  @Override
  public void execute() { }

  @Override
  public boolean isFinished() { return isReset; }

  @Override
  public void end(boolean interrupted) {  }
}
