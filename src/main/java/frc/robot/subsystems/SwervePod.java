package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.SwervePodConstants;

public class SwervePod {

    private CANSparkMax driveMotor;
    private TalonSRX spinMotor;
    private CANEncoder driveEncoder;
    private CANPIDController driveController;

    private double fps2rpm = SwervePodConstants.FPS_2_RPM;
    
    public SwervePod() {
        driveMotor = new CANSparkMax(1, MotorType.kBrushless);
        spinMotor = new TalonSRX(2);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();

        spinMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
        SmartDashboard.putNumber("kP", 0);

        //Set up driveMotor PID
        driveController.setP(SwervePodConstants.DRIVE_PID_OFFSEASON[0]);
        driveController.setI(SwervePodConstants.DRIVE_PID_OFFSEASON[1]);
        driveController.setD(SwervePodConstants.DRIVE_PID_OFFSEASON[2]);
        driveController.setFF(SwervePodConstants.DRIVE_PID_OFFSEASON[3]);
        driveController.setIZone(SwervePodConstants.DRIVE_PID_OFFSEASON[4]);

        //Brownout Prevention
        driveMotor.setSmartCurrentLimit(SwervePodConstants.DRIVE_CURRENT_LIMIT);
        driveMotor.setClosedLoopRampRate(SwervePodConstants.DRIVE_RAMP_RATE);
    }

    public void outputToSmartdashboard() {
        SmartDashboard.putNumber("Drive Position", driveEncoder.getPosition());
        SmartDashboard.putNumber("Drive Velocity", driveEncoder.getVelocity());
        SmartDashboard.putNumber("Drive Current", driveMotor.getOutputCurrent());
        SmartDashboard.putNumber("Drive P", driveController.getP());
        SmartDashboard.putNumber("Drive I", driveController.getI());
        SmartDashboard.putNumber("Drive D", driveController.getD());
        SmartDashboard.putNumber("Steer Postion", encoderTics2Degrees(spinMotor.getSelectedSensorPosition()));
        SmartDashboard.putNumber("Steer Velocity", spinMotor.getSelectedSensorVelocity());
    }

    /******************
    FEED FORWARD METHODS
    *******************/

    public void driveNSpinPercentControl(double drivePercent, double spinPercent) {
        driveMotorPercentControl(-drivePercent);
        spinMotorPercentControl(spinPercent);
    }

    public void driveMotorPercentControl(double percent) {
        driveMotor.set(percent);
    }

    public void spinMotorPercentControl(double percent) {
        spinMotor.set(ControlMode.PercentOutput, percent);
    }

    /**********
    PID METHODS
    ***********/

    public void setPodDriveNSpin(double driveSpeed, double angle) {
        setPodDrive(driveSpeed);
        //setPodSpin(angle);
    }

    /**
     * @param driveSpeed The velocity value from 0 to 13 feet per second
     */
    public void setPodDrive(double driveSpeed) {
        double velocitySetPoint = driveSpeed * fps2rpm;
        driveController.setReference(velocitySetPoint, ControlType.kVelocity);
    }

    public void setPodSpin(double angle) {
        
    }

    public void steerMotorDegreeControl(double wantedDegrees) {
        double currentDegrees = encoderTics2Degrees(spinMotor.getSelectedSensorPosition());
        double error = wantedDegrees - currentDegrees;
        //kP = SmartDashboard.getNumber("kP", 0);
        //spinMotor.set(ControlMode.PercentOutput, kP*error);
    }

    public double encoderTics2Degrees(double tics) {
        return (tics % 4096) * (360.0 / 4096);
    }
}