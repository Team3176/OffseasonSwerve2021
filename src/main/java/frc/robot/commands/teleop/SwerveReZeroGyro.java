package frc.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.net.ssl.TrustManagerFactorySpi;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class SwerveReZeroGyro extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private boolean isreset;

  public SwerveReZeroGyro() {
    addRequirements(drivetrain);
    isreset = false;
  }

  @Override
  public void initialize() {  
  }

  @Override
  public void execute() {
    drivetrain.gyroUpdateOffset();
    isreset = true;
  }

  @Override
  public boolean isFinished() { 
    if (isreset) {
      return true;
    } else {
      return false;
    } 
  }

  @Override
  public void end(boolean interrupted) { 
  }

}
