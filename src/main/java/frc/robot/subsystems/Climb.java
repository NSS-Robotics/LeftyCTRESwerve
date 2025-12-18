package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.ClimberConstants.motorID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.ClimberConstants.limitSwitchID);

    public Climb() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void makeMotorSpin(double velocity) {
        motor.set(velocity);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb Limit Switch", getLimitSwitch());
    }
}
