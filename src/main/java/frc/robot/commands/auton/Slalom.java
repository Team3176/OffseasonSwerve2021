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
    double time3 = 4.25;
    double time4 = 1.4;
    double time5 = 1.6;

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
        drivetrain.resetGyro();
        drivetrain.setCoordType(coordType.FIELD_CENTRIC);          
    };

    @Override
    public void execute() {
        //Forward
        if((startTime + time1) > Timer.getFPGATimestamp()) {
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            drivetrain.drive(-0.5, 0.0, podSpin);
        // Left
        } else if((startTime + time1 + time2) > Timer.getFPGATimestamp()) {
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            drivetrain.drive(0.0, 0.5, podSpin);
        // Big forward
        } else if((startTime + time1 + time2 + time3) > Timer.getFPGATimestamp()) {
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            drivetrain.drive(-0.5, 0.0, podSpin);
        // Right
        } else if((startTime + time1 + time2 + time3 + time4) > Timer.getFPGATimestamp()) {
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            drivetrain.drive(0.0, -0.5, podSpin);
        // Forward
        } else if((startTime + time1 + time2 + time3 + time4 + time5) > Timer.getFPGATimestamp()) {
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            drivetrain.drive(-0.5, 0.0, podSpin);
        // Left
        } else if((startTime + time1 + time2 + time3 + time4 + time5 + SmartDashboard.getNumber("runTime", runTimeInput)) > Timer.getFPGATimestamp()) {
            podSpin = pidRotate.returnOutput(drivetrain.getAutonCrudeGyroAngleAvg(), 0);
            drivetrain.drive(0.0, 0.5, podSpin);
        } else {
            drivetrain.drive(0.0, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return (startTime + time1 + time2 + time3 + time4 + time5 + SmartDashboard.getNumber("runTime", runTimeInput)) < Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) { 
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}
