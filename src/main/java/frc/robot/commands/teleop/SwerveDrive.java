package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.state;

public class SwerveDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;

  public SwerveDrive(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
    this.forwardCommand = forwardCommand.getAsDouble();
    this.strafeCommand = strafeCommand.getAsDouble();
    this.spinCommand = spinCommand.getAsDouble();
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.setWantedState(state.DRIVE);
    drivetrain.drive(forwardCommand, strafeCommand, spinCommand);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}
