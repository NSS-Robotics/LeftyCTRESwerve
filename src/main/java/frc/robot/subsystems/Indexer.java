package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final SparkMax motor = new SparkMax(
        Constants.CanIDs.indexerMotorID,
        MotorType.kBrushless
    );
    private final SparkMaxConfig motorConfig;
    private final LaserCan lasercan;

    public Indexer() {
        motorConfig = new SparkMaxConfig();
        lasercan = new LaserCan(Constants.CanIDs.indexerLaserCANID);

        motorConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
        motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
            lasercan.setRegionOfInterest(
                new LaserCan.RegionOfInterest(8, 8, 16, 16)
            );
            lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void setIndexer(double speed) {
        motor.set(speed);
    }

    public void resetEncoders() {
        motor.getEncoder().setPosition(0);
    }

    public boolean gamepieceDetected() {
        LaserCanInterface.Measurement reading = lasercan.getMeasurement();
        if (reading == null || reading.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS) {
            return false;
        }
        return reading.distance_mm < 20;
    }

    public void stopIndexer() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Indexer Velocity",
            motor.getEncoder().getVelocity()
        );
        SmartDashboard.putBoolean(
            "Indexer Game Piece Detected",
            gamepieceDetected()
        );
        SmartDashboard.putNumber(
            "LaserCAN dist Indexer",
            lasercan.getMeasurement().distance_mm
        );
        SmartDashboard.putNumber("Indexer Current", motor.getOutputCurrent());
    }
}
