package frc.robot.commands.auton;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.coordType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
/* ##################################################################################
*  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
* ################################################################################## */
import frc.robot.util.PIDLoop;
/* ##################################################################################
*  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
* ################################################################################## */

public class Slalom extends CommandBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private double startTime;
    double runTimeInput;
    double time1 = 1.15;
    double time2 = 1.43;
    double time3 = 3.5; // It's about that but still need to est

    /* ##################################################################################
    *  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
    * ################################################################################## */
    private PIDLoop pidRotate;
    double podSpin = 0.0;
    /* ##################################################################################
    *  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
    * ################################################################################## */


    public Slalom() {
        addRequirements(drivetrain);
        runTimeInput = SmartDashboard.getNumber("runTime", 0.5);
        /* ##################################################################################
        *  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
        * ################################################################################## */
        pidRotate = new PIDLoop(0.0015, 0, 0, 0.5);
        /* ##################################################################################
        *  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
        * ################################################################################## */

    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.setCoordType(coordType.FIELD_CENTRIC);
        drivetrain.resetGyro();  
    };

    @Override
    public void execute() {
        if((startTime + time1) > Timer.getFPGATimestamp()) {
            // BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            // END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
            drivetrain.drive(0.5, 0.0, podSpin);
        } else if((startTime + time1 + time2) > Timer.getFPGATimestamp()) {
            // BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            // END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
            drivetrain.drive(0.0, -0.5, podSpin);
        } else if((startTime + time1 + time2 + SmartDashboard.getNumber("runTime", runTimeInput)) > Timer.getFPGATimestamp()) {
            // BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            // END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
            drivetrain.drive(0.5, 0.0, podSpin);
        }
    }

    @Override
    public boolean isFinished() {
        return (startTime + time1 + time2 + SmartDashboard.getNumber("runTime", runTimeInput)) < Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) { 
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}
