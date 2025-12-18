package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;

public class SetUpClimb extends Command{
    private final Elevator elevator;
    private final ArmPivot armPivot;
    private final Climb climb;

    public SetUpClimb(Elevator elevator, ArmPivot armPivot, Climb climb){
        this.armPivot = armPivot;
        this.elevator = elevator;
        this.climb = climb;
        addRequirements(elevator, armPivot, climb);
    }

    @Override
    public void execute(){
        elevator.setPosition(0.655);
        Timer.delay(1);
        armPivot.setPosition(-0.298427734375, false);
        climb.makeMotorSpin(0.3);
   }

    @Override
    public void end(boolean interrupted) {
        
    }
}
