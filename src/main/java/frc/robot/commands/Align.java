package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class Align extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Limelight limelight;

    private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

    private Pose2d currentPos;
    private Pose2d targetPos;
    //headingcontroller.enableContinuousInput();
    public Align(RobotContainer rob) {
        this.swerve = rob.drivetrain;
        this.limelight = rob.limelight;
    }

    @Override
    public void initialize() {

        Pose2d fieldTarget = new Pose2d(13.88, 5.211, Rotation2d.fromDegrees(65));

        currentPos = swerve.getPoseMeters();
        // This is for when the limelight does NOT update the swerve odometry
        // targetPos = new Pose2d(
        //     swerve.getPoseMeters().getX() + (limelight.botPose.getX() - fieldTarget.getX()),
        //     swerve.getPoseMeters().getY() + (limelight.botPose.getY() - fieldTarget.getY()),
        //     Rotation2d.fromDegrees(swerve.getPigeon2().getYaw().getValueAsDouble())
        // );
        targetPos = fieldTarget;
    }

    public double mapDistanceToVelocity(double distance) {
        final double maxVel = 1;
        final double minVel = 0.05;

        distance = Math.abs(distance);
        if (distance > 1) distance = 1;

        return minVel + distance * (maxVel - minVel);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Target X", targetPos.getX());
        SmartDashboard.putNumber("Target Y", targetPos.getY());
        SmartDashboard.putNumber("Target R", targetPos.getRotation().getDegrees());

        double xVelocity = 0;
        double yVelocity = 0;
      //  double radianVelocity = headingController.calculate(currentPos.getRotation().getRadians(), targetPos.getRotation().getRadians(),Timer.getFPGATimestamp());
        double deadZone = 0.1;
        double autoDeadZone= 0.25;
        double angleDeadZone = 2;

        currentPos = swerve.getPoseMeters();

        // Get X velocity
        if ((targetPos.getX() + deadZone) > currentPos.getX()) {
            xVelocity = -mapDistanceToVelocity(currentPos.getX() - targetPos.getX());
        } else if ((targetPos.getX() - deadZone) < currentPos.getX()) {
            xVelocity = mapDistanceToVelocity(currentPos.getX() - targetPos.getX());
        }

        // Get Y velocity
        if ((targetPos.getY() + deadZone) > currentPos.getY()) {
            yVelocity = -mapDistanceToVelocity(currentPos.getY() - targetPos.getY());
        } else if ((targetPos.getY() - deadZone) < currentPos.getY()) {
            yVelocity = mapDistanceToVelocity(currentPos.getY() - targetPos.getY());
        }

        // // Get rotation rate
        // if ((targetPos.getRotation().getDegrees() + angleDeadZone) > currentPos.getRotation().getDegrees()) {
        //     radianVelocity = 0.2;
        // } else if ((targetPos.getRotation().getDegrees() - angleDeadZone) < currentPos.getRotation().getDegrees()) {
        //     radianVelocity = -0.2;
        // }

        SmartDashboard.putNumber("X Velocity", xVelocity);
        SmartDashboard.putNumber("Y Velocity", yVelocity);
      //  SmartDashboard.putNumber("Radian Velocity", radianVelocity);
    
        swerve.setControl(
            alignRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withDeadband(autoDeadZone)
                .withHeadingPID(1,0,0.1)
                .withTargetDirection(targetPos.getRotation())
                // .withMaxAbsRotationalRate(0.2)
              //  .withTargetRateFeedforward(Units.degreesToRadians(36))
        );

        System.out.println(mapDistanceToVelocity(currentPos.getY() - targetPos.getY()));
    }

    @Override
    public boolean isFinished() {
        double distanceToTarget = Math.hypot(
            limelight.botPose.getX() - targetPos.getX(),
            limelight.botPose.getY() - targetPos.getY()
        );

        return distanceToTarget < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(alignRequest.withVelocityX(0).withVelocityY(0));
    }
}
