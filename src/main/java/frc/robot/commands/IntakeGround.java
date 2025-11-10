package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Indexer;

public class IntakeGround extends Command {
    private final Intake m_intake;
    private final IntakePivot m_intakePivot;
    private final Indexer m_indexer;

    public IntakeGround(RobotContainer rob){
        this.m_intake = rob.intake;
        this.m_intakePivot = rob.intakePivot;
        this.m_indexer = rob.indexer;

        addRequirements(rob.intake, rob.intakePivot, rob.indexer);
    }

    @Override
    public void execute() {
        m_intakePivot.setPosition(Constants.Intake.intakeStartPos);
        m_intake.setIntake(Constants.Intake.velocity, false);
        m_indexer.setIndexer(Constants.Indexer.velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakePivot.setPosition(Constants.Intake.upPosition);
        m_indexer.stopIndexer();
        m_intake.stopIntake();
    }
}
