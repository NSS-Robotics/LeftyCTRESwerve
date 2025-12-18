package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbPivot extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.ClimberConstants.climbPivotID);
    private final CANcoder encoder = new CANcoder(Constants.ClimberConstants.encoderID);

    private PositionVoltage pv = new PositionVoltage(0);

    // Motor configuration
    private TalonFXConfiguration motorConfig;

    public ClimbPivot() {
        
        motor.setNeutralMode(NeutralModeValue.Brake);
        motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        motor.getConfigurator().apply(motorConfig);
    }

    public void makeMotorSpin(double velocity) {
        motor.set(velocity);
    }
    public void stop() {
        motor.stopMotor();
    }

    public void setPosition(double position) {
        pv = new PositionVoltage(position);
        motor.setControl(pv);
    }
    
    public double getPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Pivot Position", 0);
    }
}