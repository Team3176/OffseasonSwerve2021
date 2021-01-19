package frc.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import frc.robot.subsystems.Drivetrain.driveMode;

public class SwerveDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;

  private boolean isFieldCentric;
  private boolean isRobotCentric;
  private boolean isBackRobotCentric;

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand,
                      BooleanSupplier isFieldCentric, BooleanSupplier isRobotCentric, BooleanSupplier isBackRobotCentric) {
    this.forwardCommand = forwardCommand.getAsDouble();
    this.strafeCommand = strafeCommand.getAsDouble();
    this.spinCommand = spinCommand.getAsDouble();
    this.isFieldCentric = isFieldCentric.getAsBoolean();
    this.isRobotCentric = isRobotCentric.getAsBoolean();
    this.isBackRobotCentric = isBackRobotCentric.getAsBoolean();
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {
    if(isFieldCentric) {
      drivetrain.setCoordType(coordType.FIELD_CENTRIC);
    }
    if(isRobotCentric) {
      drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    }
    if(isBackRobotCentric) {
      drivetrain.setCoordType(coordType.BACK_ROBOT_CENTRIC);
    }
    drivetrain.drive(forwardCommand, strafeCommand, spinCommand);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}
