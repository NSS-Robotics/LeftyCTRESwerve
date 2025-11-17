package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Align extends Command {
    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric();

    private Pose2d startPos;
    private Pose2d targetPos;

    public Align(RobotContainer rob) {
        this.swerve = rob.drivetrain;
    }

    @Override
    public void initialize() {
        startPos = swerve.getPoseMeters();
        targetPos = new Pose2d(
            swerve.getPoseMeters().getX() + 0.3,
            swerve.getPoseMeters().getY() - 0.3,
            Rotation2d.fromDegrees(swerve.getPigeon2().getYaw().getValueAsDouble())
        );
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Target X", targetPos.getX());
        SmartDashboard.putNumber("Target Y", targetPos.getY());

        double xVelocity = 0;
        double yVelocity = 0;

        if (targetPos.getX() > startPos.getX()) {
            xVelocity = -0.25;
        } else if (targetPos.getX() < startPos.getX()) {
            xVelocity = 0.25;
        }

        if (targetPos.getY() > startPos.getY()) {
            yVelocity = -0.25;
        } else if (targetPos.getY() < startPos.getY()) {
            yVelocity = 0.25;
        }

        swerve.setControl(
            alignRequest.withVelocityX(xVelocity).withVelocityY(yVelocity)
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPos = swerve.getPoseMeters();

        if (Math.abs(currentPos.getX() - targetPos.getX()) < 0.1) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(alignRequest.withVelocityX(0).withVelocityY(0));
    }
}
