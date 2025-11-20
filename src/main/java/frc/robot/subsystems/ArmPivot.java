// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class ArmPivot extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.ArmPivotConstants.motorID);
  private final CANcoder encoder = new CANcoder(Constants.ArmPivotConstants.encoderID);

  private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
  private VelocityVoltage vv;
  private final RobotContainer rob;

  // Motor configuration
  private TalonFXConfiguration motorConfig;
  private Slot0Configs slot0Configs;
  private Slot1Configs slot1Configs;
  private CANcoderConfiguration canCoderConfig;
  private CurrentLimitsConfigs currentLimitsConfigs;
  
  public ArmPivot(RobotContainer robert) {
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(canCoderConfig);

    this.rob = robert;

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.ArmPivotConstants.upKP;
    slot0Configs.kI = Constants.ArmPivotConstants.upKI;
    slot0Configs.kD = Constants.ArmPivotConstants.upKD;
    slot0Configs.kS = Constants.ArmPivotConstants.kS;
    slot0Configs.kV = Constants.ArmPivotConstants.kV;
    slot0Configs.kA = Constants.ArmPivotConstants.kA;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kG = Constants.ArmPivotConstants.kG;

    slot1Configs = new Slot1Configs();
    slot1Configs.kP = Constants.ArmPivotConstants.downKP;
    slot1Configs.kI = Constants.ArmPivotConstants.downKI;
    slot1Configs.kD = Constants.ArmPivotConstants.downKD;
    slot1Configs.kS = Constants.ArmPivotConstants.kS;
    slot1Configs.kV = Constants.ArmPivotConstants.kV;
    slot1Configs.kA = Constants.ArmPivotConstants.kA;
    slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot1Configs.kG = Constants.ArmPivotConstants.kG;

    motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 50;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(slot0Configs);
    // motor.getConfigurator().apply(slot1Configs);
    motor.getConfigurator().apply(currentLimitsConfigs);
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setPosition(double position) {
    // int slot = 1; // Default to assuming arm is going down.

    // if (getEncoder() < position) {
    //   // If arm is going up, use up PID.
    //   slot = 0;
    // }

    pv = new PositionVoltage(position);//.withSlot(slot);
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

  @Override
  public void periodic() {
    boolean isInAllowedROM = (
      getEncoder() < Constants.ArmPivotConstants.maxRotations &&
      getEncoder() > Constants.ArmPivotConstants.minRotations
    );

    if (!isInAllowedROM) {
      // stop();
    }

    SmartDashboard.putNumber("Arm Pivot Position", getEncoder());
    SmartDashboard.putNumber("Arm Pivot Velocity", getVelocity());
    SmartDashboard.putNumber("Arm Pivot Current", motor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putBoolean(
      "Arm Pivot Using Max Current",
      motor.getStatorCurrent().getValueAsDouble() >= currentLimitsConfigs.StatorCurrentLimit
    );
  }
}