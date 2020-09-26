package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //temporary

public class ArcadeDrive extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotSpeed;

  public ArcadeDrive(DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    addRequirements(m_Drivetrain);
  }

  @Override
  public void execute() {
    //Provides joystick controll with a speed and rotation
    m_Drivetrain.drive(this.ySpeed.getAsDouble(), this.rotSpeed.getAsDouble());
    
    //SmartDashboard.putNumber("Drive Stick y", ySpeed.getAsDouble());
    //SmartDashboard.putNumber("Spin Stick y", rotSpeed.getAsDouble());
  }
}
