// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeGround;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Start CTRE Swerve stuff

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.maxSpeed * 0.1).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry ctreSwerveLogger = new Telemetry(Constants.Swerve.maxSpeed);
    // End of CTRE Swerve stuff

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Indexer indexer = new Indexer();
    public final Intake intake = new Intake(this);
    public final IntakePivot intakePivot = new IntakePivot(this);
    public final ArmPivot armPivot = new ArmPivot(this);
    public final Elevator elevator = new Elevator(this);

    // Variables
    public boolean isCoral = true;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * Constants.Swerve.maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * Constants.Swerve.maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * Constants.Swerve.maxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        drivetrain.registerTelemetry(ctreSwerveLogger::telemeterize);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // TODO: What the hell is this, is it rotational align?
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // zero gyro
        driverController.y().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        driverController
            .leftTrigger()
            .whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setPosition(0.9)),
                new IntakeGround(this)
            ));
        
        driverController
            .povLeft()
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> elevator.setPosition(Constants.Elevator.pos[RobotState.algaeReefLow.ordinal()])),
                    new WaitCommand(0.1),
                    new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivot.pos[RobotState.algaeReefLow.ordinal()]))//,
                    // new InstantCommand(() -> m_claw.makeMotorSpin(-0.7))
                )
            );
}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
