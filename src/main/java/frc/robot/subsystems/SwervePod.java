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

    private int off = 0;

    private double lastEncoderPos;
    private double radianError;
    private double encoderError;
    private double driveCommand;
    private double velocitySetPoint;

    private double PI = Math.PI;
    private double maxFps = SwervePodConstants.DRIVE_SPEED_MAX_EMPIRICAL_FPS;

    public SwervePod(int id, TalonFX driveController, TalonSRX spinController) {
        this.id = id;
        this.driveController = driveController;
        this.spinController = spinController;

        this.spinController.config_kP(0, SwervePodConstants.SPIN_PID_CONFIG[0][id], 0);
        this.spinController.config_kI(0, SwervePodConstants.SPIN_PID_CONFIG[1][id], 0);
        this.spinController.config_kD(0, SwervePodConstants.SPIN_PID_CONFIG[2][id], 0);
        this.spinController.config_kF(0, SwervePodConstants.SPIN_PID_CONFIG[3][id], 0);

        encoderOffset = SwervePodConstants.OFFSETS[id];

        SmartDashboard.putNumber("offset P" + id, off);
    }

    /**
     * @param podDrive Something
     * @param podSpin Angle from 0 to -2pi
     */
    public void set(double podDrive, double podSpin) {
        double ticsPer100ms = podDrive * 2000.0 * 2048.0 / 600.0;
        SmartDashboard.putNumber("McQueen", ticsPer100ms);
        velocitySetPoint = podDrive * SwervePodConstants.DRIVE_SPEED_MAX_EMPIRICAL_FPS;
        double encoderSetPos = calcSpinPos(podSpin);
        if (podDrive != 0) {
            spinController.set(ControlMode.Position, encoderSetPos);
            lastEncoderPos = encoderSetPos;
        } else {
            spinController.set(ControlMode.Position, lastEncoderPos);
        }
        
        driveController.set(TalonFXControlMode.Velocity, ticsPer100ms);
        if(id == 3) {
            SmartDashboard.putNumber("Pod3 Tics", spinController.getSelectedSensorPosition()); 
            SmartDashboard.putNumber("ESP", encoderSetPos);
        }
        

//        driveController.set(ControlMode., velocitySetPoint / maxFps);
    }

    /**
     * @param angle desired angle of swerve pod in units of radians, range from -2PI to +2PI
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
            velocitySetPoint = -velocitySetPoint;
        }
        encoderError = rads2Tics(radianError);
        driveCommand = encoderError + encoderPos + encoderOffset;
        return driveCommand;
    }

    private int rads2Tics(double rads) {
        return (int) ((4096 / (PI * 2)) * rads);
    }

    private double tics2Rads(int tics) {
        return ((PI * 2) / 4096) * tics;
    }
}
