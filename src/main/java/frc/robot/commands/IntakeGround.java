package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.*;

public class IntakeGround extends Command {
    private final Intake m_intake;
    private final IntakePivot m_intakePivot;
    private final Indexer m_indexer;
    private final ArmPivot m_armPivot;
    private final Elevator m_elevator;
    private final Claw m_claw;

    public IntakeGround(RobotContainer rob) {
        this.m_intake = rob.intake;
        this.m_intakePivot = rob.intakePivot;
        this.m_indexer = rob.indexer;
        this.m_armPivot = rob.armPivot;
        this.m_elevator = rob.elevator;
        this.m_claw = rob.claw;

        addRequirements(rob.intake);
    }

    @Override
    public void execute() {
        m_elevator.setPosition(1.3);
        m_intakePivot.setPosition(Constants.IntakeConstants.intakeStartPos);
        m_intake.setIntake(Constants.IntakeConstants.velocity, false);
        m_indexer.setIndexer(Constants.IndexerConstants.velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakePivot.setPosition(Constants.IntakeConstants.upPosition);
        m_indexer.stopIndexer();
        m_intake.stopIntake();

        // shotgun
        m_armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.start.ordinal()], false);
        m_elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.coralIntake.ordinal()]);
        m_claw.makeMotorSpin(-0.4);
        Timer.delay(1);
        m_claw.makeMotorSpin(-0.067);
    }

    // @Override
    // public boolean isFinished() {
    //     return m_indexer.troughGamepieceDetected();
    // }
}