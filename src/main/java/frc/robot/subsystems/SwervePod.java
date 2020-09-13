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

    //Class constants
    private double fps2rpm = SwervePodConstants.FPS_2_RPM;
    private double PI = Math.PI;
    private double kEncoderUnits = SwervePodConstants.ENCODER_UNITS;
    private double[] kAbsoluteOffsets = SwervePodConstants.OFFSETS;
    private double gearRatio = SwervePodConstants.DRIVE_GEAR_RATIO;

    //All measured in encoder units
    private double lastEncoderPosition; //Previous position
    private double encoderPosition; //Current position
    private double encoderError; //Error

    //All measured in radians
    private double radianPosition; //Current position
    private double radianError; //Error

    private double driveCommand;
    private double encoderSetpoint;
    private double velocitySetpoint;

    private int id;
    
    public SwervePod(int id/*,CANSparkMax driveMotor, TalonSRX spinMotor;*/) {
        this.id = id;
        //this.driveMotor = driveMotor
        //this.spinMotor = spinMotor
        
        driveMotor = new CANSparkMax(1, MotorType.kBrushless);
        spinMotor = new TalonSRX(2);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();

        spinMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
        SmartDashboard.putNumber("kP", 0);

        //Set up driveMotor PID
        driveController.setP(SwervePodConstants.DRIVE_PID_OFFSEASON_OFFSETS[0]);
        driveController.setI(SwervePodConstants.DRIVE_PID_OFFSEASON_OFFSETS[1]);
        driveController.setD(SwervePodConstants.DRIVE_PID_OFFSEASON_OFFSETS[2]);
        driveController.setFF(SwervePodConstants.DRIVE_PID_OFFSEASON_OFFSETS[3]);
        driveController.setIZone(SwervePodConstants.DRIVE_PID_OFFSEASON_OFFSETS[4]);

        //Set up steerMotor PID
        spinMotor.config_kP(0, SwervePodConstants.SPIN_PID_CONFIG[0][id], 0);
        spinMotor.config_kI(0, SwervePodConstants.SPIN_PID_CONFIG[1][id], 0);
        spinMotor.config_kD(0, SwervePodConstants.SPIN_PID_CONFIG[2][id], 0);
        spinMotor.config_kF(0, SwervePodConstants.SPIN_PID_CONFIG[3][id], 0);

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

    public void percentFFDriveNSpin(double drivePercent, double spinPercent) {
        percentFFDriveMotor(-drivePercent);
        percentFFSpinMotor(spinPercent);
    }

    public void percentFFDriveMotor(double percent) {
        driveMotor.set(percent);
    }

    public void percentFFSpinMotor(double percent) {
        spinMotor.set(ControlMode.PercentOutput, percent);
    }

    /**********
    PID METHODS
    ***********/

    public void velocityPIDDriveNSpin(double driveSpeed, double angle) {
        velocityPIDDrive(driveSpeed);
        //velocityPIDSpin(angle);
    }

    /**
     * @param driveSpeed The velocity value from 0 to 13 feet per second
     */
    public void velocityPIDDrive(double driveSpeed) {
        double velocitySetPoint = driveSpeed * fps2rpm;
        driveController.setReference(velocitySetPoint, ControlType.kVelocity);

        //(Max speed / max joystick) * input
    }

    public void velocityPIDSpin(double angle, double driveSpeed) {
        double velocitySetPoint = driveSpeed * fps2rpm;
        double encoderSetPoint = calcSpinPos(angle);
        
        if(driveSpeed != 0) {
            spinMotor.set(ControlMode.Position, encoderSetPoint);
            lastEncoderPosition = -encoderSetPoint;
        } else {
            spinMotor.set(ControlMode.Position, lastEncoderPosition);
        }
        driveController.setReference(velocitySetPoint, ControlType.kVelocity);
    }
    
    /**
     * @param angle Measured in radians
     * @return
     */
    private double calcSpinPos(double angle) {
        double encoderPos = spinMotor.getSelectedSensorPosition(0) - kAbsoluteOffsets[id];
        double radianPos = encoderUnits2Radians(encoderPos);
        radianError = angle - radianPos;

        if(Math.abs(radianError) > (5 * (PI / 2))) {
            System.out.println("Error: Overload");
        } else if(Math.abs(radianError) > (3 * (PI / 2))) {
            radianError -= Math.copySign(2 * PI, radianError);
        } else if(Math.abs(radianError) > (PI / 2)) {
            radianError -= Math.copySign(PI, radianError);
            velocitySetpoint = -velocitySetpoint;
        }

        encoderError = radian2EncoderUnits(radianError);
        driveCommand = encoderError + encoderPos + kAbsoluteOffsets[id];
        return driveCommand;
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

    private double radian2EncoderUnits(double radians) {
        double encoderUnits = (radians / (2 * PI)) * kEncoderUnits;
        return encoderUnits;
    }

    private double encoderUnits2Radians(double encoderUnits) {
        encoderUnits = encoderUnits % kEncoderUnits;
        if(encoderUnits < 0) {
            encoderUnits += kEncoderUnits;
        }
        encoderUnits -= (kEncoderUnits / 2);
        double angle = (encoderUnits / kEncoderUnits) * (2 * PI);
        return angle;
    }
}