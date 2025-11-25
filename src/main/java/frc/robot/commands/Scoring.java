package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Elevator;

public class Scoring extends Command{
    private final Elevator elevator;
    private final ArmPivot armPivot;
    //private final RobotState state;

    public Scoring(RobotContainer rob) {
        this.elevator = rob.elevator;
        this.armPivot = rob.armPivot;
        //this.state = state;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setPosition(4.147998046875);
        Timer.delay(1);
        armPivot.setPosition(-1.95751953125);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        armPivot.stop();

    }
}
