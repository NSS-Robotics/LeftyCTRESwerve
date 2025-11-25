package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw;

public class ClawOuttake extends Command {
    private final RobotContainer rob;
    private final Claw claw;

    public ClawOuttake(RobotContainer rob, Claw claw) {
        this.rob = rob;
        this.claw = rob.claw;

        addRequirements(rob.claw, rob.armPivot);
    }

    @Override
    public void execute() {
        // UNCOMMENT
        // if (rob.scoringLevel == RobotState.algaeReefLow ||
        //     rob.scoringLevel == RobotState.algaeReefHigh ||
        //     rob.scoringLevel == RobotState.pickup ||
        //     rob.scoringLevel == RobotState.algaeGround) {
        //         return;
        // }

        claw.makeMotorSpin(Constants.ClawConstants.teleopOuttakeVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
    }
}