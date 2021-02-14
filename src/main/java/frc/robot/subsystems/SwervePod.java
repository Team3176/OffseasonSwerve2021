package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.SwervePodConstants;

public class SwervePod {

    private TalonFX driveController;
    private TalonSRX spinController;

    private int id;
    private int encoderOffset; 
    private double spinTics2Rev;
    private double driveTics2Rev;

    public int spinSlotIdx, spinPIDLoopIdx, spinTimeoutMs;

    private double spinP = SwervePodConstants.SPIN_PID[0][id];
    private double spinI = SwervePodConstants.SPIN_PID[1][id];
    private double spinD = SwervePodConstants.SPIN_PID[2][id];
    private double spinF = SwervePodConstants.SPIN_PID[3][id];

    private double driveP = SwervePodConstants.DRIVE_PID[0][id];
    private double driveI = SwervePodConstants.DRIVE_PID[1][id];
    private double driveD = SwervePodConstants.DRIVE_PID[2][id];
    private double driveF = SwervePodConstants.DRIVE_PID[3][id];

    public SwervePod(int id, TalonFX driveController, TalonSRX spinController) {
        this.id = id;
        this.driveController = driveController;
        this.spinController = spinController;

        this.driveController.configFactoryDefault();
        this.spinController.configFactoryDefault();

        this.driveController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        this.spinController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        this.driveController.config_kP(0, driveP, 0);
        this.driveController.config_kI(0, driveI, 0);
        this.driveController.config_kD(0, driveD, 0);
        this.driveController.config_kF(0, driveF, 0);

        this.spinController.config_kP(spinPIDLoopIdx, spinP, spinTimeoutMs);
        this.spinController.config_kI(spinPIDLoopIdx, spinI, spinTimeoutMs);
        this.spinController.config_kD(spinPIDLoopIdx, spinD, spinTimeoutMs);
        this.spinController.config_kF(spinPIDLoopIdx, spinF, spinTimeoutMs);

        encoderOffset = SwervePodConstants.OFFSETS[id];
        spinTics2Rev = SwervePodConstants.SPIN_ENCODER_UNITS_PER_REVOLUTION;
        driveTics2Rev = SwervePodConstants.DRIVE_ENCODER_UNITS_PER_REVOLUTION;

        spinSlotIdx = SwervePodConstants.TALON_PID_SLOT_ID;
        spinPIDLoopIdx = SwervePodConstants.TALON_PID_LOOP_ID;
        spinTimeoutMs = SwervePodConstants.TALON_TIMEOUT_MS;
    }

    /**
     * @param podDrive -1 to 1 (I think) but isn't ether giving 0-13 ft/s?
     * @param podSpin Angle from 0 to 2pi
     */
    public void set(double podDrive, double podSpin) {
        double velTicsPer100ms = podDrive * 2000.0 * driveTics2Rev / 600.0;
        double tics = rads2Tics(podSpin + encoderOffset);
        spinController.set(ControlMode.Position, tics);
        driveController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
    }

    private int rads2Tics(double rads) {
        rads = rads * (2 * Math.PI);
        double tics = ((rads / (2.0*Math.PI)) * spinTics2Rev);
        return (int) tics;
    }

    private double spinTics2Rads(double tics) {
        tics = tics % spinTics2Rev;
        if(tics < 0) {
            tics += spinTics2Rev;
        }
        tics -= (spinTics2Rev / 2);
        return (tics / spinTics2Rev) * (2 * Math.PI);
    }
}
