package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* The Goal for today is to work on this function such that when the user calls this command the robot moves towards and set location and stops
 * 
 * Goal !: Move To a Target on the field
 * Goal 2: Move TO the Closest Target out of list of 6 set points
 * Goal 3: Trigger the Scoring Command when within 0.5 meters of an target 
 * 
 */

public class Align extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Limelight limelight;

    private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

    private Pose2d currentPos;
    private Pose2d targetPos;
    boolean wasTriggured;
    RobotContainer robot;
    //headingcontroller.enableContinuousInput();
    public Align(RobotContainer rob) {
        this.swerve = rob.drivetrain;
        this.limelight = rob.limelight;
        robot = rob;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Scoring", "False");

        //Pose2d fieldTarget = new Pose2d(13.88, 5.211, Rotation2d.fromDegrees(65));
        Pose2d fieldTarget_1 = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
        Pose2d fieldTarget_2 = new Pose2d(4, 3, Rotation2d.fromDegrees(60));
        Pose2d fieldTarget_3 = new Pose2d(5, 3, Rotation2d.fromDegrees(300));
        Pose2d fieldTarget_4 = new Pose2d(6, 4, Rotation2d.fromDegrees(180));
        Pose2d fieldTarget_5 = new Pose2d(5, 5, Rotation2d.fromDegrees(240));
        Pose2d fieldTarget_6 = new Pose2d(4, 5, Rotation2d.fromDegrees(120));

        boolean wasTriggured = false;


        currentPos = swerve.getPoseMeters();
        // This is for when the limelight does NOT update the swerve odometry
        // targetPos = new Pose2d(
        //     swerve.getPoseMeters().getX() + (limelight.botPose.getX() - fieldTarget.getX()),
        //     swerve.getPoseMeters().getY() + (limelight.botPose.getY() - fieldTarget.getY()),
        //     Rotation2d.fromDegrees(swerve.getPigeon2().getYaw().getValueAsDouble())
        // );

        Pose2d[] allTargets = {fieldTarget_1, fieldTarget_2, fieldTarget_3, fieldTarget_4, fieldTarget_5, fieldTarget_6};
        double min_distance =  Math.hypot(currentPos.getX() - allTargets[0].getX(), currentPos.getY() - allTargets[0].getY());
        Pose2d fieldTarget = allTargets[0];

        for (Pose2d target : allTargets) {
            double newDistance = Math.hypot(currentPos.getX() - target.getX(), currentPos.getY() - target.getY());
            if (newDistance < min_distance){
                min_distance = newDistance;
                fieldTarget = target;
            }

        }


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

        double distanceToTarget = Math.hypot(
            currentPos.getX() - targetPos.getX(),
            currentPos.getY() - targetPos.getY()
        );
        if (distanceToTarget < 0.5 && !wasTriggured){
            CommandScheduler.getInstance().schedule(new Scoring(robot));
            wasTriggured = true;

        }

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
            currentPos.getX() - targetPos.getX(),
            currentPos.getY() - targetPos.getY()
        );

        return distanceToTarget < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(alignRequest.withVelocityX(0).withVelocityY(0));
    }
}
