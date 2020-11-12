package frc.robot.subsystems;

import javax.xml.namespace.QName;

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

    private int id;
    private int encoderOffset = SwervePodConstants.OFFSETS[id];

    private int lastEncoderPos = 0;

    private double PI = Math.PI;
    
    public SwervePod(int id) {
        this.id = id;
        
        driveMotor = new CANSparkMax(1, MotorType.kBrushless);
        spinMotor = new TalonSRX(2);

        spinMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);

        this.spinMotor.config_kP(0, SwervePodConstants.SPIN_PID_CONFIG[0][id], 0);
		this.spinMotor.config_kI(0, SwervePodConstants.SPIN_PID_CONFIG[1][id], 0);
		this.spinMotor.config_kD(0, SwervePodConstants.SPIN_PID_CONFIG[2][id], 0);
		this.spinMotor.config_kF(0, SwervePodConstants.SPIN_PID_CONFIG[3][id], 0);
    }

    public void thrust(double transMag) {
        driveMotor.set(transMag);
        SmartDashboard.putNumber("TM", transMag);
    }

    public void spin(double transMag, double transAngle) {
        int encoderSetPos = rads2Tics(transAngle) + encoderOffset;
        if(transMag != 0) {
            spinMotor.set(ControlMode.Position, encoderSetPos);
            lastEncoderPos = encoderSetPos;
        } else {
            spinMotor.set(ControlMode.Position, lastEncoderPos);
        }
        SmartDashboard.putNumber("Encoder Set Pos", encoderSetPos);
    }

    private int rads2Tics(double rads) {
        return (int)((4096 / (PI * 2)) * rads);
    }
}
