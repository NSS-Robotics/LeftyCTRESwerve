package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class Align extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Limelight limelight;

    private final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric();

    private Pose2d currentPos;
    private Pose2d targetPos;

    public Align(RobotContainer rob) {
        this.swerve = rob.drivetrain;
        this.limelight = rob.limelight;
    }

    @Override
    public void initialize() {
        currentPos = new Pose2d(
            limelight.botPose.getX(),
            limelight.botPose.getY(),
            Rotation2d.fromDegrees(limelight.getRotation())
        );
        targetPos = new Pose2d(
            5.25,
            1.35,
            Rotation2d.fromDegrees(limelight.getRotation())
        );
    }

    public double mapDistanceToVelocity(double distance) {
        final double maxVel = 1;
        final double minVel = 0.2;

        distance = Math.abs(distance);
        if (distance > 1) distance = 1;

        return minVel + distance * (maxVel - minVel);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Target X", targetPos.getX());
        SmartDashboard.putNumber("Target Y", targetPos.getY());

        double xVelocity = 0;
        double yVelocity = 0;
        double deadZone =  0.13;

        currentPos = new Pose2d(
            limelight.botPose.getX(),
            limelight.botPose.getY(),
            Rotation2d.fromDegrees(limelight.getRotation())
        );

        if ((targetPos.getX() + deadZone) > currentPos.getX()) {
            xVelocity = 0.25;
        } else if ((targetPos.getX() - deadZone) < currentPos.getX()) {
            xVelocity = -0.25;
        }

        if ((targetPos.getY() + deadZone) > currentPos.getY()) {
            yVelocity = 0.25;
        } else if ((targetPos.getY() - deadZone) < currentPos.getY()) {
            yVelocity = -0.25;
        }

        SmartDashboard.putNumber("X Velocity", xVelocity);
        SmartDashboard.putNumber("Y Velocity", yVelocity);

        swerve.setControl(
            alignRequest.withVelocityX(xVelocity).withVelocityY(yVelocity)
        );

        System.out.println(mapDistanceToVelocity(currentPos.getY() - targetPos.getY()));
    }

    @Override
    public boolean isFinished() {
        double distanceToTarget = Math.hypot(
            limelight.botPose.getX() - targetPos.getX(),
            limelight.botPose.getY() - targetPos.getY()
        );

        return distanceToTarget < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(alignRequest.withVelocityX(0).withVelocityY(0));
    }
}
