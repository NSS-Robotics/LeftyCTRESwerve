package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    private static TalonFX intakeMotor = new TalonFX(
        Constants.IntakeConstants.intakeMotorID
    );

    CurrentLimitsConfigs intakeCurrentLimitsConfigs =
        new CurrentLimitsConfigs();

    private static Slot0Configs intakeSlot0Configs = new Slot0Configs();
    private static VelocityVoltage intakeVelocityVoltage;
    private RobotContainer rob;

    public Intake(RobotContainer rob) {
        this.rob = rob;

        intakeSlot0Configs.kP = Constants.IntakeConstants.intakeKP;
        intakeSlot0Configs.kI = Constants.IntakeConstants.intakeKI;
        intakeSlot0Configs.kD = Constants.IntakeConstants.intakeKD;
        intakeSlot0Configs.kS = Constants.IntakeConstants.intakeKS;
        intakeSlot0Configs.kV = Constants.IntakeConstants.intakeKV;
        intakeSlot0Configs.kA = Constants.IntakeConstants.intakeKA;
        intakeCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
        intakeCurrentLimitsConfigs.StatorCurrentLimit = 55;

        intakeMotor.getConfigurator().apply(intakeSlot0Configs);
        intakeMotor.getConfigurator().apply(intakeCurrentLimitsConfigs);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Resets the encoder position.
     */
    public void resetEncoders() {
        intakeMotor.setPosition(0);
    }

    public double getIntakeCurrent() {
        return intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setIntake(double velocity, boolean l1) {
        intakeVelocityVoltage = new VelocityVoltage(velocity / 60);
        intakeCurrentLimitsConfigs.StatorCurrentLimit = rob.isCoral ? 80 : 50;
        intakeMotor.getConfigurator().apply(intakeCurrentLimitsConfigs);

        intakeMotor.setControl(intakeVelocityVoltage);
    }

    public void stopIntake() {
        VoltageOut intakeVoltageOut = new VoltageOut(0);
        intakeMotor.setControl(intakeVoltageOut);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", getIntakeCurrent());
    }
}