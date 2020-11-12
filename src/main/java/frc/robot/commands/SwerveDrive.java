package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private DoubleSupplier rotX;
  private DoubleSupplier transMag;
  private DoubleSupplier transAngle;


  public SwerveDrive(DoubleSupplier rotX, DoubleSupplier transMag, DoubleSupplier transAngle) {
    this.rotX = rotX;
    this.transMag = transMag;
    this.transAngle = transAngle;
    addRequirements(m_Drivetrain);
  }

  @Override
  public void execute() {
    m_Drivetrain.drive(this.rotX.getAsDouble(), this.transMag.getAsDouble(), this.transAngle.getAsDouble());
  }
}
