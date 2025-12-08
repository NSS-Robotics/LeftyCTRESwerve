// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
  private final RobotContainer rob;

  // Motor configuration
  private TalonFXConfiguration motorConfig;
  private Slot0Configs upPIDConfigs;
  private Slot1Configs downPIDConfigs;
  private Slot2Configs scorePIDConfigs;
  private CANcoderConfiguration canCoderConfig;
  private CurrentLimitsConfigs currentLimitsConfigs;
  private double slot0TuneP = 0;
  private double slot0TuneI = 0;
  private double slot0TuneD = 0;
  
  public ArmPivot(RobotContainer robert) {
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(canCoderConfig);

    this.rob = robert;

    upPIDConfigs = new Slot0Configs();
    upPIDConfigs.kP = Constants.ArmPivotConstants.upKP;
    upPIDConfigs.kI = Constants.ArmPivotConstants.upKI;
    upPIDConfigs.kD = Constants.ArmPivotConstants.upKD;
    upPIDConfigs.kS = Constants.ArmPivotConstants.kS;
    upPIDConfigs.kV = Constants.ArmPivotConstants.kV;
    upPIDConfigs.kA = Constants.ArmPivotConstants.kA;
    upPIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    upPIDConfigs.kG = Constants.ArmPivotConstants.kG;

    downPIDConfigs = new Slot1Configs();
    downPIDConfigs.kP = Constants.ArmPivotConstants.downKP;
    downPIDConfigs.kI = Constants.ArmPivotConstants.downKI;
    downPIDConfigs.kD = Constants.ArmPivotConstants.downKD;
    downPIDConfigs.kS = Constants.ArmPivotConstants.kS;
    downPIDConfigs.kV = Constants.ArmPivotConstants.kV;
    downPIDConfigs.kA = Constants.ArmPivotConstants.kA;
    downPIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    downPIDConfigs.kG = Constants.ArmPivotConstants.kG;

    scorePIDConfigs = new Slot2Configs();
    scorePIDConfigs.kP = Constants.ArmPivotConstants.scoreKP;
    scorePIDConfigs.kI = Constants.ArmPivotConstants.downKI;
    scorePIDConfigs.kD = Constants.ArmPivotConstants.downKD;
    scorePIDConfigs.kS = Constants.ArmPivotConstants.kS;
    scorePIDConfigs.kV = Constants.ArmPivotConstants.kV;
    scorePIDConfigs.kA = Constants.ArmPivotConstants.kA;
    scorePIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;

    motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 50;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(upPIDConfigs);
    motor.getConfigurator().apply(downPIDConfigs);
    motor.getConfigurator().apply(scorePIDConfigs);
    motor.getConfigurator().apply(currentLimitsConfigs);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setPosition(double targetPos, boolean isScoring) {
    int slot = 1; // Default to assuming arm is going down.

    if (getEncoder() > targetPos) {
      // If arm is going up, use up PID.
      slot = 0;
    }

    if (isScoring) {
      slot = 2;
    }

    pv = new PositionVoltage(targetPos).withSlot(slot);
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

    slot0TuneP = SmartDashboard.getNumber("Slot 0 P Arm", 0);
    SmartDashboard.putNumber("Slot 0 P Arm", slot0TuneP);
    slot0TuneI = SmartDashboard.getNumber("Slot 0 I Arm", 0);
    SmartDashboard.putNumber("Slot 0 I Arm", slot0TuneI);
    slot0TuneD = SmartDashboard.getNumber("Slot 0 D Arm", 0);
    SmartDashboard.putNumber("Slot 0 D Arm", slot0TuneD);
    
    boolean isChanged = false;

    if(slot0TuneP != upPIDConfigs.kP){
      upPIDConfigs.kP = slot0TuneP;
    isChanged = true; 
    }

    if(slot0TuneI != upPIDConfigs.kI){
      upPIDConfigs.kI = slot0TuneI;
    isChanged = true;
    }

    if(slot0TuneD != upPIDConfigs.kD){
      upPIDConfigs.kD = slot0TuneD;
    isChanged = true;
    }

    if(isChanged){
      motor.getConfigurator().apply(upPIDConfigs);
    }

    
    slot0TuneP = SmartDashboard.getNumber("Slot 0 P Arm Down", 0);
    SmartDashboard.putNumber("Slot 0 P Arm Down", slot0TuneP);
    slot0TuneI = SmartDashboard.getNumber("Slot 0 I Arm Down", 0);
    SmartDashboard.putNumber("Slot 0 I Arm Down", slot0TuneI);
    slot0TuneD = SmartDashboard.getNumber("Slot 0 D Arm Down", 0);
    SmartDashboard.putNumber("Slot 0 D Arm Down", slot0TuneD);
    
    boolean isChangedDown = false;

    if(slot0TuneP != downPIDConfigs.kP){
      downPIDConfigs.kP = slot0TuneP;
    isChangedDown = true; 
    }

    if(slot0TuneI != downPIDConfigs.kI){
      downPIDConfigs.kI = slot0TuneI;
    isChangedDown = true;
    }

    if(slot0TuneD != downPIDConfigs.kD){
      downPIDConfigs.kD = slot0TuneD;
    isChangedDown = true;
    }

    if(isChangedDown){
      motor.getConfigurator().apply(downPIDConfigs);
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