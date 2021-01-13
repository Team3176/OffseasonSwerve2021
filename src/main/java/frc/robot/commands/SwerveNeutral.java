package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class SwerveNeutral extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public SwerveNeutral() {
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}
