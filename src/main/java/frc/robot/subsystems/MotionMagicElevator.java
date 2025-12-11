package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage; // Motion magic!
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

//Motion magic imports (google AI)
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotionMagicElevator extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.ElevatorConstants.motorID);
    private final TalonFX followerMotor = new TalonFX(Constants.ElevatorConstants.followerMotorID);
    private final CANcoder encoder = new CANcoder(Constants.ElevatorConstants.encoder);

    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
    private double slot0TuneP = 0;
    private double slot0TuneI = 0;
    private double slot0TuneD = 0;
    // Motor configuration
    private TalonFXConfiguration motorConfig;
    private Slot0Configs slot0Configs;
    private Slot1Configs slot1Configs;
    private CANcoderConfiguration canCoderConfig;
    private CurrentLimitsConfigs currentLimitsConfigs;
    private MotionMagicConfigs motionMagicConfig; // Motion Magic

    private final ArmPivot m_armPivot;

    public MotionMagicElevator(RobotContainer robert) {
        m_armPivot = new ArmPivot(robert);

        canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder.getConfigurator().apply(canCoderConfig);
        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

        slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.ElevatorConstants.upKP;
        slot0Configs.kI = Constants.ElevatorConstants.upKI;
        slot0Configs.kD = Constants.ElevatorConstants.upKD;
        slot0Configs.kS = Constants.ElevatorConstants.upKS;
        slot0Configs.kV = Constants.ElevatorConstants.upKV;
        slot0Configs.kA = Constants.ElevatorConstants.upKA;

        slot1Configs = new Slot1Configs();
        slot1Configs.kP = Constants.ElevatorConstants.downKP;
        slot1Configs.kI = Constants.ElevatorConstants.downKI;
        slot1Configs.kD = Constants.ElevatorConstants.downKD;
        slot1Configs.kS = Constants.ElevatorConstants.downKS;
        slot1Configs.kV = Constants.ElevatorConstants.downKV;
        slot1Configs.kA = Constants.ElevatorConstants.downKA;

        motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimit = 40;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        // Motion Magic arbitrary test values
        motionMagicConfig = motorConfig.MotionMagic;
        motionMagicConfig.MotionMagicCruiseVelocity = 0; // Cruise velocity (0 means infinite)
        motionMagicConfig.MotionMagicExpo_kV = 0.12; // Voltage needed to increase voltage by 1 rps
        motionMagicConfig.MotionMagicExpo_kA = 0.1; // Voltage needed to accelerate by 1 rps/s
        motionMagicConfig.MotionMagicAcceleration = 0; // maximum acceleration (0 means infinite)
        motionMagicConfig.MotionMagicJerk = 0; // max jerk (derivative of accel) (0 means infinite)

        motor.getConfigurator().apply(motorConfig);
        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(slot1Configs);
        motor.getConfigurator().apply(currentLimitsConfigs);
        motor.getConfigurator().apply(motionMagicConfig);
        motor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.getConfigurator().apply(motorConfig);
        followerMotor.getConfigurator().apply(slot0Configs);
        followerMotor.getConfigurator().apply(slot1Configs);
        followerMotor.getConfigurator().apply(currentLimitsConfigs);
        followerMotor.getConfigurator().apply(motionMagicConfig);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        followerMotor.setControl(new Follower(Constants.ElevatorConstants.motorID, true));
    }

    public void setPosition(double targetPos) {
        int slot = 0;
        if (getEncoder() > targetPos) {
            slot = 1;
        }

        final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(targetPos).withSlot(slot);
        motor.setControl(m_request);
    }

    public double getEncoder() {
        return encoder.getPosition().getValueAsDouble();
    }

    public BooleanSupplier canArmMoveHome() {
        if (encoder.getPosition().getValueAsDouble() > 1.4 && encoder.getPosition().getValueAsDouble() < 1.6) {
            return () -> true;
        }
        return () -> false;
    }

    public BooleanSupplier canArmMoveUp() {
        if (encoder.getPosition().getValueAsDouble() > 1.5) {
            return () -> true;
        }
        return () -> false;
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        motor.setControl(new VoltageOut(0));
    }

    private boolean canMove() {
        // If arm is up, then check if elevator can move
        if (m_armPivot.getEncoder() < Constants.ArmPivotConstants.horizontal) {
            if (this.getEncoder() < Constants.ElevatorConstants.minRotationsWithArmScoring) {
                return false;
            }
        }

        // If arm is inside, check if elev can move
        if (this.getEncoder() < Constants.ElevatorConstants.minRotationsWithArmIn) {
            return false;
        }

        return true;
    }

    @Override
    public void periodic() {
        // if (!canMove()) {
        // stop();
        //
        slot0TuneP = SmartDashboard.getNumber("Slot 0 P", 0);
        SmartDashboard.putNumber("Slot 0 P", slot0TuneP);
        slot0TuneI = SmartDashboard.getNumber("Slot 0 I", 0);
        SmartDashboard.putNumber("Slot 0 I", slot0TuneI);
        slot0TuneD = SmartDashboard.getNumber("Slot 0 D", 0);
        SmartDashboard.putNumber("Slot 0 D", slot0TuneD);

        boolean isChanged = false;

        if (slot0TuneP != slot0Configs.kP) {
            slot0Configs.kP = slot0TuneP;
            isChanged = true;
        }

        if (slot0TuneI != slot0Configs.kI) {
            slot0Configs.kI = slot0TuneI;
            isChanged = true;
        }

        if (slot0TuneD != slot0Configs.kD) {
            slot0Configs.kD = slot0TuneD;
            isChanged = true;
        }

        // if(isChanged){
        // motor.getConfigurator().apply(slot0Configs);
        // }

        SmartDashboard.putNumber("Elevator Rotations", getEncoder());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator Current", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator can move", canMove());
    }
}