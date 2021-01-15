package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.state;

public class SwerveDefense extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public SwerveDefense() {
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.setWantedState(state.DEFENSE);
    drivetrain.enterDefense();
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}
