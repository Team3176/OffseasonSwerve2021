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
    private double kEncoderUnitsPerRevolution;
    private int off = 0;

    private double lastEncoderPos;
    private double radianError;
    private double encoderError;
    private double driveCommand;
    private boolean flipDrive;

    private double p = SwervePodConstants.SPIN_PID[0][id];
    private double i = SwervePodConstants.SPIN_PID[1][id];
    private double d = SwervePodConstants.SPIN_PID[2][id];
    private double f = SwervePodConstants.SPIN_PID[3][id];

    private double PI = Math.PI;
    private double maxFps = SwervePodConstants.DRIVE_SPEED_MAX_EMPIRICAL_FPS;

    public SwervePod(int id, TalonFX driveController, TalonSRX spinController) {
        this.id = id;
        this.driveController = driveController;
        this.spinController = spinController;

        // TODO: May or may not need this. Figure that out then add or delete
        this.driveController.config_kP(0, SwervePodConstants.DRIVE_PID[0][2], 0);
        this.driveController.config_kI(0, SwervePodConstants.DRIVE_PID[1][2], 0);
        this.driveController.config_kD(0, SwervePodConstants.DRIVE_PID[2][2], 0);
        this.driveController.config_kF(0, SwervePodConstants.DRIVE_PID[3][2], 0);

        SmartDashboard.putNumber("P", p);
        SmartDashboard.putNumber("I", i);
        SmartDashboard.putNumber("D", d);
        SmartDashboard.putNumber("F", f);

        this.spinController.config_kP(0, p, 0);
        this.spinController.config_kI(0, i, 0);
        this.spinController.config_kD(0, d, 0);
        this.spinController.config_kF(0, f, 0);

        encoderOffset = SwervePodConstants.OFFSETS[id];
        kEncoderUnitsPerRevolution = SwervePodConstants.ENCODER_UNITS;
    }

    /**
     * @param podDrive Something
     * @param podSpin Angle from 0 to 2pi
     */
    public void set(double podDrive, double podSpin) {
        this.spinController.config_kP(0, SmartDashboard.getNumber("P", p), 0);
        this.spinController.config_kI(0, SmartDashboard.getNumber("I", i), 0);
        this.spinController.config_kD(0, SmartDashboard.getNumber("D", d), 0);
        this.spinController.config_kF(0, SmartDashboard.getNumber("F", f), 0);
        double velTicsPer100ms = podDrive * 2000.0 * 2048.0 / 600.0;
        double encoderSetPos = calcSpinPos(podSpin);
        if (podDrive != 0) {
            spinController.set(ControlMode.Position, encoderSetPos);
            lastEncoderPos = encoderSetPos;
        } else {
            spinController.set(ControlMode.Position, lastEncoderPos);
        }

        if(flipDrive) { 
            velTicsPer100ms *= -1; 
            flipDrive = !flipDrive; 
        }
        driveController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
        SmartDashboard.putNumber("tics", spinController.getSelectedSensorPosition());
        SmartDashboard.putNumber("SPTics", encoderSetPos);
        SmartDashboard.putNumber("Error", spinController.getSelectedSensorPosition() - encoderSetPos);
    }

    /**
     * @param angle desired angle of swerve pod in units of radians, range from 0 to +2PI
     * @return
     */
    private double calcSpinPos(double angle) {
        int encoderPos = spinController.getSelectedSensorPosition(0) - encoderOffset;
        double radianPos = tics2Rads(encoderPos);
        radianError = angle - radianPos;

        if (Math.abs(radianError) > (5 * (PI / 2))) {
            System.out.println("Error: Overload");
        } else if (Math.abs(radianError) > (3 * (PI / 2))) {
            radianError -= Math.copySign(2 * PI, radianError);
        } else if (Math.abs(radianError) > (PI / 2)) {
            radianError -= Math.copySign(PI, radianError);
            flipDrive = !flipDrive;
        }
        encoderError = rads2Tics(radianError);
        driveCommand = encoderError + encoderPos + encoderOffset;
        return driveCommand;
    }

    private int rads2Tics(double rads) {
        //return (int) ((kEndoderUnitsPerRevolution / (PI * 2)) * rads);
        double tics = ((rads / (2.0*Math.PI)) * kEncoderUnitsPerRevolution);
        return (int) tics;
    }

    private double tics2Rads(double tics) {
        tics = tics % kEncoderUnitsPerRevolution;
        if(tics < 0) {
            tics += kEncoderUnitsPerRevolution;
        }
        tics -= (kEncoderUnitsPerRevolution / 2);
        return (tics / kEncoderUnitsPerRevolution) * (2 * PI);
    }
}
