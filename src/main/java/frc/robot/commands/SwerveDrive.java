package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private DoubleSupplier rotX;
  private DoubleSupplier transMag;
  private DoubleSupplier transAngle;

  public SwerveDrive(DoubleSupplier rotX, DoubleSupplier transMag, DoubleSupplier transAngle) {
    this.rotX = rotX;
    this.transMag = transMag;
    this.transAngle = transAngle;
    addRequirements(drivetrain);
    SmartDashboard.putNumber("transAngle Constr", this.transAngle.getAsDouble());
  }

  @Override
  public void execute() {
    drivetrain.drive(this.rotX.getAsDouble(), this.transMag.getAsDouble(), this.transAngle.getAsDouble());
    SmartDashboard.putNumber("transAngle Exe", this.transAngle.getAsDouble());
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}
