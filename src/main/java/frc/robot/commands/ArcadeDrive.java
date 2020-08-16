package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //temporary

public class ArcadeDrive extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private DoubleSupplier xSpeed;
  private DoubleSupplier rotSpeed;

  public ArcadeDrive(DoubleSupplier xSpeed, DoubleSupplier rotSpeed) {
    this.xSpeed = xSpeed;
    this.rotSpeed = rotSpeed;
    addRequirements(m_Drivetrain);
  }

  @Override
  public void execute() {
    // Calls the universal drive function of Drivetrain and passes it a ChassisSpeeds object that has been created with the desired linear and rotational speeds
    //m_Drivetrain.drive(new ChassisSpeeds(xSpeed.getAsDouble(), 0.0, rotSpeed.getAsDouble()));
    m_Drivetrain.drive();
    SmartDashboard.putNumber("test", xSpeed.getAsDouble());
    SmartDashboard.putNumber("Spin Stick y", rotSpeed.getAsDouble());
  }
}
