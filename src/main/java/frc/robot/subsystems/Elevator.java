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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.ElevatorConstants.motorID);
  private final TalonFX followerMotor = new TalonFX(Constants.ElevatorConstants.followerMotorID);
  private final CANcoder encoder = new CANcoder(Constants.ElevatorConstants.encoder);

  private PositionVoltage pv = new PositionVoltage(0).withSlot(0);

  // Motor configuration
  private TalonFXConfiguration motorConfig;
  private Slot0Configs slot0Configs;
  private Slot1Configs slot1Configs;
  private CANcoderConfiguration canCoderConfig;
  private CurrentLimitsConfigs currentLimitsConfigs;

  private final ArmPivot m_armPivot;
  
  public Elevator(RobotContainer robert) {
    m_armPivot = new ArmPivot(robert);

    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(canCoderConfig);

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.ElevatorConstants.upKP;
    slot0Configs.kI = Constants.ElevatorConstants.upKI;
    slot0Configs.kD = Constants.ElevatorConstants.upKD;
    slot0Configs.kS = Constants.ElevatorConstants.upKS;
    slot0Configs.kV = Constants.ElevatorConstants.upKV;
    slot0Configs.kA = Constants.ElevatorConstants.upKA;

    // slot1Configs = new Slot1Configs();
    // slot1Configs.kP = Constants.ElevatorConstants.downKP;
    // slot1Configs.kI = Constants.ElevatorConstants.downKI;
    // slot1Configs.kD = Constants.ElevatorConstants.downKD;
    // slot1Configs.kS = Constants.ElevatorConstants.downKS;
    // slot1Configs.kV = Constants.ElevatorConstants.downKV;
    // slot1Configs.kA = Constants.ElevatorConstants.downKA;

    motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 40;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(slot0Configs);
    // motor.getConfigurator().apply(slot1Configs);
    motor.getConfigurator().apply(currentLimitsConfigs);
    motor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.getConfigurator().apply(motorConfig);
    followerMotor.getConfigurator().apply(slot0Configs);
    // followerMotor.getConfigurator().apply(slot1Configs);
    followerMotor.getConfigurator().apply(currentLimitsConfigs);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);

    followerMotor.setControl(new Follower(Constants.ElevatorConstants.motorID, true));
  }

  public void setPosition(double targetPos) {
    double rps = 20;
    if (getEncoder() > targetPos) {
      rps = -rps;
    }

    pv = new PositionVoltage(targetPos).withVelocity(rps);
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
    //     stop();
    // }

    SmartDashboard.putNumber("Elevator Rotations", getEncoder());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
    SmartDashboard.putNumber("Elevator Current", motor.getStatorCurrent().getValueAsDouble());
   SmartDashboard.putBoolean("Elevator can move", canMove());
  }
}