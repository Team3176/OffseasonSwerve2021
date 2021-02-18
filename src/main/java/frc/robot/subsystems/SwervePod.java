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
    private int kEncoderOffset; 
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

    public int kSlotIdx_spin, kPIDLoopIdx_spin, kTimeoutMs_spin,kSlotIdx_drive, kPIDLoopIdx_drive, kTimeoutMs_drive;


    private double kP_Spin;
    private double kI_Spin;
    private double kD_Spin;
    private double kF_Spin;

    private double kP_Drive;
    private double kI_Drive;
    private double kD_Drive;
    private double kF_Drive;

    private double PI = Math.PI;
    private double maxFps = SwervePodConstants.DRIVE_SPEED_MAX_EMPIRICAL_FEET_PER_SECOND;

    private int startTics;

    public SwervePod(int id, TalonFX driveController, TalonSRX spinController) {
        this.id = id;

        kEncoderOffset = SwervePodConstants.SPIN_OFFSET[id];
        kSpinEncoderUnitsPerRevolution = SwervePodConstants.SPIN_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_spin = SwervePodConstants.TALON_SPIN_PID_SLOT_ID;
        kPIDLoopIdx_spin = SwervePodConstants.TALON_SPIN_PID_LOOP_ID;
        kTimeoutMs_spin = SwervePodConstants.TALON_SPIN_PID_TIMEOUT_MS;

        kDriveEncoderUnitsPerRevolution = SwervePodConstants.DRIVE_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_drive = SwervePodConstants.TALON_DRIVE_PID_SLOT_ID;
        kPIDLoopIdx_drive = SwervePodConstants.TALON_DRIVE_PID_LOOP_ID;
        kTimeoutMs_drive = SwervePodConstants.TALON_DRIVE_PID_TIMEOUT_MS;


        kP_Spin = SwervePodConstants.SPIN_PID[0][id];
        kI_Spin = SwervePodConstants.SPIN_PID[1][id];
        kD_Spin = SwervePodConstants.SPIN_PID[2][id];
        kF_Spin = SwervePodConstants.SPIN_PID[3][id];

        kP_Drive = SwervePodConstants.DRIVE_PID[0][id];
        kI_Drive = SwervePodConstants.DRIVE_PID[1][id];
        kD_Drive = SwervePodConstants.DRIVE_PID[2][id];
        kF_Drive = SwervePodConstants.DRIVE_PID[3][id];

        this.driveController = driveController;
        this.spinController = spinController;

        this.driveController.configFactoryDefault();
        this.spinController.configFactoryDefault();

        this.driveController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        this.spinController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        this.driveController.config_kP(0, kP_Drive, 0);
        this.driveController.config_kI(0, kI_Drive, 0);
        this.driveController.config_kD(0, kD_Drive, 0);
        this.driveController.config_kF(0, kF_Drive, 0);

        // SmartDashboard.putNumber("P", kP_Spin);
        // SmartDashboard.putNumber("I", kI_Spin);
        // SmartDashboard.putNumber("D", kD_Spin);
        // SmartDashboard.putNumber("F", kF_Spin);

        this.spinController.config_kP(kPIDLoopIdx_spin, kP_Spin, kTimeoutMs_spin);
        this.spinController.config_kI(kPIDLoopIdx_spin, kI_Spin, kTimeoutMs_spin);
        this.spinController.config_kD(kPIDLoopIdx_spin, kD_Spin, kTimeoutMs_spin);
        this.spinController.config_kF(kPIDLoopIdx_spin, kF_Spin, kTimeoutMs_spin);

        

        startTics = spinController.getSelectedSensorPosition();
        SmartDashboard.putNumber("startTics", startTics);
    }

    /**
     * @param podDrive represents desired thrust of swervepod Range = -1 to 1 or
     *                 ft-per-sec?
     * @param podSpin  represents desired angle of swervepod. Range = -pi to pi.
     */
    public void set(double podDrive, double podSpin) {
        this.spinController.config_kP(kSlotIdx_spin, SmartDashboard.getNumber("P", kP_Spin), kTimeoutMs_spin);
        this.spinController.config_kI(kSlotIdx_spin, SmartDashboard.getNumber("I", kI_Spin), kTimeoutMs_spin);
        this.spinController.config_kD(kSlotIdx_spin, SmartDashboard.getNumber("D", kD_Spin), kTimeoutMs_spin);
        this.spinController.config_kF(kSlotIdx_spin, SmartDashboard.getNumber("F", kF_Spin), kTimeoutMs_spin);
        SmartDashboard.putNumber("P" + (id + 1) + " podDrive", podDrive);
        SmartDashboard.putNumber("P" + (id + 1) + " podSpin", podSpin);
            // TODO: need check ether output values. speed vs %-values
        velTicsPer100ms = podDrive * 2000.0 * kDriveEncoderUnitsPerRevolution / 600.0;
        double encoderSetPos = calcSpinPos(podSpin);
        double tics = rads2Tics(podSpin);
        SmartDashboard.putNumber("P" + (id + 1) + " tics", tics);
        SmartDashboard.putNumber("P" + (id + 1) + " absTics", spinController.getSelectedSensorPosition());
        if (podDrive != 0) {
            spinController.set(ControlMode.Position, tics);
            lastEncoderPos = encoderSetPos;
        } else {
            spinController.set(ControlMode.Position, lastEncoderPos);
        }
        driveController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
        SmartDashboard.putNumber("P" + (id + 1) + " velTicsPer100ms", velTicsPer100ms);
        SmartDashboard.putNumber("P" + (id + 1) + " encoderSetPos", encoderSetPos);
    }

    /**
     * @param angle desired angle of swerve pod in units of radians, range from 0 to +2PI
     * @param angle desired angle of swerve pod in units of radians, range from -PI to +PI
     * @return
     */
    private double calcSpinPos(double angle) {
        encoderPos = spinController.getSelectedSensorPosition(0) - kEncoderOffset;
        radianPos = tics2Rads(encoderPos);
        radianError = angle - radianPos;

        if (Math.abs(radianError) > (5 * (PI / 2))) {
            System.out.println("Error: Overload");
        } else if (Math.abs(radianError) > (3 * (PI / 2))) {
        //if (Math.abs(radianError) > (3 * (PI / 2))) {
            radianError -= Math.copySign(2 * PI, radianError);
        } else if (Math.abs(radianError) > (PI / 2)) {
            radianError -= Math.copySign(PI, radianError);
           // velTicsPer100ms = -velTicsPer100ms;
            SmartDashboard.putNumber("radian Error", radianError);
        }
        encoderError = rads2Tics(radianError);
        driveCommand = encoderError + encoderPos + kEncoderOffset;
        return (driveCommand);
    }

    public void goHome() {
        double homePos = 0 + kEncoderOffset;
        spinController.set(ControlMode.Position, homePos);
    }

    private int rads2Tics(double rads) {
        //rads = rads * (2 * Math.PI);
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
