package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.driveMode;

public class SwerveOrbit extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  private DoubleSupplier spinCommand;

  public SwerveOrbit(DoubleSupplier spinCommand) {
    this.spinCommand = spinCommand;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.ORBIT);
  }

  @Override
  public void execute() {
      drivetrain.drive(spinCommand.getAsDouble() * 2 /* temporary constant */, 0.0, spinCommand.getAsDouble());
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}
