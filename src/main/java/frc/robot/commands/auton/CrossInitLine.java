package frc.robot.commands.auton;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CrossInitLine extends CommandBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();

    public CrossInitLine() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false; //Would normally be a timer checking things
    }

    @Override
    public void end(boolean interrupted) { 

    }
}
