package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.CanIDs.elevatorMotorID);
  private final TalonFX followerMotor = new TalonFX(Constants.CanIDs.elevatorFollowerMotorID);
  private final CANcoder encoder = new CANcoder(Constants.CanIDs.elevatorEncoderID);

  private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
  private VelocityVoltage vv;

  // Motor configuration
  private TalonFXConfiguration motorConfig;
  private Slot0Configs slot0Configs;
  private Slot1Configs slot1Configs;
  private CANcoderConfiguration canCoderConfig;
  private CurrentLimitsConfigs currentLimitsConfigs;

  private final ArmPivot m_armPivot;
  
  public Elevator(RobotContainer robert) {
    m_armPivot = robert.armPivot;

    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(canCoderConfig);

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Elevator.upKP;
    slot0Configs.kI = Constants.Elevator.upKI;
    slot0Configs.kD = Constants.Elevator.upKD;
    slot0Configs.kS = Constants.Elevator.upKS;
    slot0Configs.kV = Constants.Elevator.upKV;
    slot0Configs.kA = Constants.Elevator.upKA;

    slot1Configs = new Slot1Configs();
    slot1Configs.kP = Constants.Elevator.downKP;
    slot1Configs.kI = Constants.Elevator.downKI;
    slot1Configs.kD = Constants.Elevator.downKD;
    slot1Configs.kS = Constants.Elevator.downKS;
    slot1Configs.kV = Constants.Elevator.downKV;
    slot1Configs.kA = Constants.Elevator.downKA;

    motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 40;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(slot0Configs);
    motor.getConfigurator().apply(slot1Configs);
    motor.getConfigurator().apply(currentLimitsConfigs);
    motor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.getConfigurator().apply(motorConfig);
    followerMotor.getConfigurator().apply(slot0Configs);
    followerMotor.getConfigurator().apply(slot1Configs);
    followerMotor.getConfigurator().apply(currentLimitsConfigs);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);

    followerMotor.setControl(new Follower(Constants.CanIDs.elevatorMotorID, true));
  }

  public void setPosition(double position) {
    int slot = 0; // Assume elevator is going up.

    if (getEncoder() > position) {
      // If it's going down, use down PID values.
      slot = 1;
    }

    // Use the PID values for the direction the elevator is going.
    pv = new PositionVoltage(position).withSlot(slot);
    motor.setControl(pv);
  }

  public double getEncoder() {
    return encoder.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    motor.setControl(new VoltageOut(0));
  }

  private boolean canMove() {
    // If arm is up, then check if elevator can move
    if (m_armPivot.getEncoder() < Constants.ArmPivot.horizontal) {
        if (this.getEncoder() < Constants.Elevator.minRotationsWithArmScoring) {
            return false;
        }
    }

    // If arm is inside, check if elev can move
    if (this.getEncoder() < Constants.Elevator.minRotationsWithArmIn) {
        return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    // if (!canMove()) {
    //     stop();
    // }

    SmartDashboard.putNumber("Elevator Rotations", getEncoder());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
    SmartDashboard.putNumber("Elevator Current", motor.getStatorCurrent().getValueAsDouble());
   SmartDashboard.putBoolean("Elevator can move", canMove());
  }
}