package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* The Goal for today is to work on this function such that when the user calls this command the robot moves towards and set location and stops
 * 
 * Goal !: Move To a Target on the field
 * Pose2d fieldTarget_1 = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
 * Goal 2: Move TO the Closest Target out of list of 6 set points
 * Pose2d fieldTarget_1 = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
 * Pose2d fieldTarget_2 = new Pose2d(4, 3, Rotation2d.fromDegrees(60));
 * Pose2d fieldTarget_3 = new Pose2d(5, 3, Rotation2d.fromDegrees(300));
 * Pose2d fieldTarget_4 = new Pose2d(6, 4, Rotation2d.fromDegrees(180));
 * Pose2d fieldTarget_5 = new Pose2d(5, 5, Rotation2d.fromDegrees(240));
 * Pose2d fieldTarget_6 = new Pose2d(4, 5, Rotation2d.fromDegrees(120));
 * Goal 3: Trigger the Scoring Command when within 0.5 meters of an target 
 * 
 * 
 * Useful functions:
 * Get distance: double distance = Math.hypot(currentPos.getX() - target.getX(), currentPos.getY() - target.getY())
 * Move the robot:swerve.setControl(alignRequest);
 * Get the current robot location: currentPos = swerve.getPoseMeters()
 */

public class Align extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

    private Pose2d currentPos;
    private Pose2d targetPos;
    boolean wasTriggured;
    RobotContainer robot;

    public Align(RobotContainer rob) {
        this.swerve = rob.drivetrain;
        robot = rob;
    }

    @Override
    public void initialize() {
        // In this block you should focus on setting the correct target for the robot as this will only run once
        //targetPos = fieldTarget;
    }


    @Override
    public void execute() {
        // In this block you want to have the robot get its location and calculate how to move to the target
        // look at these docs to learn how to use this command: https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/swerve/SwerveRequest.FieldCentricFacingAngle.html
        swerve.setControl(alignRequest);
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
