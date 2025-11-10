// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
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

public class IntakePivot extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.CanIDs.intakePivotMotorID);
  private final CANcoder encoder = new CANcoder(Constants.CanIDs.intakePivotEncoderID);

  private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
  private VelocityVoltage vv;

  // Motor configuration
  private TalonFXConfiguration motorConfig;
  private Slot0Configs slot0Configs;
  private CANcoderConfiguration canCoderConfig;
  
  public IntakePivot(RobotContainer rob) {
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(canCoderConfig);

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Intake.downKP;
    slot0Configs.kI = Constants.Intake.downKI;
    slot0Configs.kD = Constants.Intake.downKD;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kG = Constants.Intake.kG;

    motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(slot0Configs);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setPosition(double position) {
    pv = new PositionVoltage(position);
    motor.setControl(pv);
  }

  public double getEncoder() {
    return encoder.getPosition().getValueAsDouble();
  }

  public void setVelocity(double velocity) {
    vv = new VelocityVoltage(velocity);
    motor.setControl(vv);
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    motor.setControl(new VoltageOut(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Position", getEncoder());
    SmartDashboard.putNumber("Intake Pivot Velocity", getVelocity());
    SmartDashboard.putNumber("Intake Pivot Current", motor.getStatorCurrent().getValueAsDouble());
  }
}
