package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

//TODO: Recognize the red dependecys because seeing red is annoying
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*; 
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.SwervePodConstants;

public class SwervePod {

    private TalonFX driveController;
    private TalonSRX spinController;

    private int id;
    private int encoderOffset; 
    private double kSpinEncoderUnitsPerRevolution;
    private double kDriveEncoderUnitsPerRevolution;
    private int off = 0;

    private double lastEncoderPos;
    private double radianError;
    private double radianPos;
    private double encoderError;
    private double encoderPos;

    private double driveCommand;
    private double velTicsPer100ms;

    public int kSlotIdx_spin, kPIDLoopIdx_spin, kTimeoutMs_spin;


    private double p = SwervePodConstants.SPIN_PID[0][id];
    private double i = SwervePodConstants.SPIN_PID[1][id];
    private double d = SwervePodConstants.SPIN_PID[2][id];
    private double f = SwervePodConstants.SPIN_PID[3][id];

    private double kP_drive = SwervePodConstants.DRIVE_PID[0][id];
    private double kI_drive = SwervePodConstants.DRIVE_PID[1][id];
    private double kD_drive = SwervePodConstants.DRIVE_PID[2][id];
    private double kF_drive = SwervePodConstants.DRIVE_PID[3][id];

    private double PI = Math.PI;
    private double maxFps = SwervePodConstants.DRIVE_SPEED_MAX_EMPIRICAL_FPS;

    public SwervePod(int id, TalonFX driveController, TalonSRX spinController) {
        this.id = id;
        this.driveController = driveController;
        this.spinController = spinController;

        this.driveController.configFactoryDefault();
        this.spinController.configFactoryDefault();

        this.driveController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        this.spinController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);

        this.driveController.config_kP(0, kP_drive, 0);
        this.driveController.config_kI(0, kI_drive, 0);
        this.driveController.config_kD(0, kD_drive, 0);
        this.driveController.config_kF(0, kF_drive, 0);

        SmartDashboard.putNumber("P", p);
        SmartDashboard.putNumber("I", i);
        SmartDashboard.putNumber("D", d);
        SmartDashboard.putNumber("F", f);

        this.spinController.config_kP(kPIDLoopIdx_spin, p, kTimeoutMs_spin);
        this.spinController.config_kI(kPIDLoopIdx_spin, i, kTimeoutMs_spin);
        this.spinController.config_kD(kPIDLoopIdx_spin, d, kTimeoutMs_spin);
        this.spinController.config_kF(kPIDLoopIdx_spin, f, kTimeoutMs_spin);

        encoderOffset = SwervePodConstants.OFFSETS[id];
        kSpinEncoderUnitsPerRevolution = SwervePodConstants.SPIN_ENCODER_UNITS_PER_REVOLUTION;
        kDriveEncoderUnitsPerRevolution = SwervePodConstants.DRIVE_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_spin = SwervePodConstants.TALON_PID_SLOT_ID;
        kPIDLoopIdx_spin = SwervePodConstants.TALON_PID_LOOP_ID;
        kTimeoutMs_spin = SwervePodConstants.TALON_TIMEOUT_MS;

    }

    /**
     * @param podDrive -1 to 1 (I think) but isn't ether giving 0-13 ft/s?
     * @param podSpin Angle from 0 to 2pi
     */
    public void set(double podDrive, double podSpin) {
        this.spinController.config_kP(kSlotIdx_spin, SmartDashboard.getNumber("P", p), kTimeoutMs_spin);
        this.spinController.config_kI(kSlotIdx_spin, SmartDashboard.getNumber("I", i), kTimeoutMs_spin);
        this.spinController.config_kD(kSlotIdx_spin, SmartDashboard.getNumber("D", d), kTimeoutMs_spin);
        this.spinController.config_kF(kSlotIdx_spin, SmartDashboard.getNumber("F", f), kTimeoutMs_spin);
            // TODO: need check ether output values. speed vs %-values
        velTicsPer100ms = podDrive * 2000.0 * kDriveEncoderUnitsPerRevolution / 600.0;
        double encoderSetPos = calcSpinPos(podSpin);
        if (podDrive != 0) {
            spinController.set(ControlMode.Position, encoderSetPos);
            lastEncoderPos = encoderSetPos;
        } else {
            spinController.set(ControlMode.Position, lastEncoderPos);
        }
        driveController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
     
        SmartDashboard.putNumber("tics P" + id, spinController.getSelectedSensorPosition());
    }

    /**
     * @param angle desired angle of swerve pod in units of radians, range from 0 to +2PI
     * @return
     */
    private double calcSpinPos(double angle) {
        encoderPos = spinController.getSelectedSensorPosition(0) - encoderOffset;
        radianPos = tics2Rads(encoderPos);
        radianError = angle - radianPos;

        //if (Math.abs(radianError) > (5 * (PI / 2))) {
        //    System.out.println("Error: Overload");
        //} else if (Math.abs(radianError) > (3 * (PI / 2))) {
        if (Math.abs(radianError) > (3 * (PI / 2))) {
            radianError -= Math.copySign(2 * PI, radianError);
        } else if (Math.abs(radianError) > (PI / 2)) {
            radianError -= Math.copySign(PI, radianError);
            velTicsPer100ms = -velTicsPer100ms;
            System.out.println("Flip thingy");
            SmartDashboard.putNumber("radian Error", radianError);
        }
        encoderError = rads2Tics(radianError);
        driveCommand = encoderError + encoderPos + encoderOffset;
        return (driveCommand);
    }

    public void goHome() {
        double homePos = 0 + encoderOffset;
        spinController.set(ControlMode.Position, homePos);
    }

    private int rads2Tics(double rads) {
        rads = rads * (2 * Math.PI);
        double tics = ((rads / (2.0*Math.PI)) * kSpinEncoderUnitsPerRevolution);
        return (int) tics;
    }

    private double tics2Rads(double tics) {
        tics = tics % kSpinEncoderUnitsPerRevolution;
        if(tics < 0) {
            tics += kSpinEncoderUnitsPerRevolution;
        }
        tics -= (kSpinEncoderUnitsPerRevolution / 2);
        return (tics / kSpinEncoderUnitsPerRevolution) * (2 * PI);
    }
}
