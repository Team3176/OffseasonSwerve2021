package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.SwervePodConstants;

public class SwervePod {

    private CANSparkMax driveMotor;
    private TalonSRX spinMotor;
    private CANEncoder driveEncoder;
    private double kP; //0.3 is close
    
    public SwervePod() {
        driveMotor = new CANSparkMax(1, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        spinMotor = new TalonSRX(2);
        spinMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
        SmartDashboard.putNumber("kP", 0);
    }

    public void outputToSmartdashboard() {
        SmartDashboard.putNumber("Drive Position", driveEncoder.getPosition());
        SmartDashboard.putNumber("Drive Velocity", driveEncoder.getVelocity());
        SmartDashboard.putNumber("Steer Postion", encoderTics2Degrees(spinMotor.getSelectedSensorPosition()));
        SmartDashboard.putNumber("Steer Velocity", spinMotor.getSelectedSensorVelocity());
    }

    public void driveNSpinPercentControl(double drivePercent, double spinPercent) {
        driveMotorPercentControl(-drivePercent / SwervePodConstants.TEMP_JOYSTICK_MIN_MAX);
        spinMotorPercentControl(spinPercent / SwervePodConstants.TEMP_JOYSTICK_MIN_MAX);
    }

    public void driveMotorPercentControl(double percent) {
        driveMotor.set(percent);
    }

    public void spinMotorPercentControl(double percent) {
        spinMotor.set(ControlMode.PercentOutput, percent);
    }

    public void steerMotorDegreeControl(double wantedDegrees) {
        double currentDegrees = encoderTics2Degrees(spinMotor.getSelectedSensorPosition());
        double error = wantedDegrees - currentDegrees;
        kP = SmartDashboard.getNumber("kP", 0);
        spinMotor.set(ControlMode.PercentOutput, kP*error);
    }

    public double encoderTics2Degrees(double tics) {
        return (tics % 4096) * (360.0 / 4096);
    }
}