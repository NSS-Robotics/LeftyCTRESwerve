package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Claw extends SubsystemBase {

    private final TalonFX motor = new TalonFX(Constants.ClawConstants.motorID); // The motor in the claw.
    private VelocityVoltage velocityVoltage;
    private VoltageOut voltageOut;
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final CurrentLimitsConfigs currentLimitsConfigs =
        new CurrentLimitsConfigs();
    private final LaserCan lasercan = new LaserCan(Constants.ClawConstants.laserCANID);
    private final RobotContainer rob;

    public Claw(RobotContainer rob) {
        slot0Configs.kP = Constants.ClawConstants.kP;
        slot0Configs.kI = Constants.ClawConstants.kI;
        slot0Configs.kD = Constants.ClawConstants.kD;
        slot0Configs.kS = Constants.ClawConstants.kS;
        slot0Configs.kA = Constants.ClawConstants.kA;
        slot0Configs.kV = Constants.ClawConstants.kV;
        this.rob = rob;

        currentLimitsConfigs.SupplyCurrentLimit = Constants.ClawConstants.currentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(currentLimitsConfigs);
        motor.setNeutralMode(NeutralModeValue.Coast);

        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
            lasercan.setRegionOfInterest(
                new LaserCan.RegionOfInterest(8, 8, 10, 10)
            );
            lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Sends the correct amount of voltage to the motor to move it at the given
     * velocity.
     *
     * @param velocity The velocity at which to spin the motor.
     */
    public void setClaw(double velocity) {
        velocityVoltage = new VelocityVoltage(velocity / 60);

        motor.setControl(velocityVoltage);
    }

    public void makeMotorSpin(double velocity) {
        motor.set(velocity);
    }

    public void stopClaw() {
        voltageOut = new VoltageOut(0);

        motor.setControl(voltageOut);
    }

    /**
     * Resets the motor encoder positions to 0.
     */
    public void resetEncoders() {
        motor.setPosition(0);
    }

    public boolean gamePieceDetected() {
        Measurement m = lasercan.getMeasurement();
        if (m == null) {
            return false;
        }
        SmartDashboard.putNumber("LaserCAN dist", m.distance_mm);
        return m.distance_mm < (rob.isCoral ? 20 : 20);
    }

    @Override
    public void periodic() {
        // if (gamePieceDetected()) {
        //     stopClaw();
        // }

        SmartDashboard.putNumber(
            "Claw Velocity",
            motor.getVelocity().getValueAsDouble()
        );
        SmartDashboard.putBoolean(
            "Claw Game Piece Detected",
            gamePieceDetected()
        );

        SmartDashboard.putNumber(
            "Claw Current",
            motor.getStatorCurrent().getValueAsDouble()
        );
    }

    public void setBrakeMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}