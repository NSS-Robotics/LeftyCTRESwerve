// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Align;
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

    private final Telemetry ctreSwerveLogger = new Telemetry(Constants.Swerve.maxSpeed);
    // End of CTRE Swerve stuff

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelight = new Limelight("front", drivetrain);
    public final Indexer indexer = new Indexer();
    public final Intake intake = new Intake(this);
    public final IntakePivot intakePivot = new IntakePivot(this);
    public final Claw claw = new Claw(this);
    public final ArmPivot armPivot = new ArmPivot(this);
    public final Elevator elevator = new Elevator(this);

    // Variables
    public boolean isCoral = true;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // zero gyro
        driverController.y().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        driverController
            .rightTrigger()
            .onTrue(
                new SequentialCommandGroup(
                    // reset all to home position
                    new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.start.ordinal()], false)),
                    new WaitCommand(1),
                    new InstantCommand(() -> elevator.setPosition(0.8))
                )
            );

        driverController
            .leftTrigger()
            .onTrue(
                new SequentialCommandGroup(
                    // score algae in barge
                    new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.barge.ordinal()])),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.barge.ordinal()], false))
                )
            );

        driverController.rightBumper().whileTrue(new IntakeGround(this));
        driverController.povUp().onTrue(new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.l4.ordinal()])));
        driverController.povDown().onTrue(new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.start.ordinal()])));

        driverController.b().onTrue(
            new SequentialCommandGroup(
                    // score l4
                    new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.coralIntake.ordinal()])),
                    new InstantCommand(() -> claw.makeMotorSpin(-0.4)),
                    new WaitCommand(1), // TODO: make WaitUntilCommand
                    new InstantCommand(() -> claw.makeMotorSpin(-0.07)),
                    new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.l4.ordinal()])),
                    new WaitCommand(1),
                    new InstantCommand(() -> armPivot.setPosition(-1.95751953125, false))
            )
        );

        driverController.x().onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> claw.setBrakeMode(false)),
                new InstantCommand(() -> armPivot.setPosition(-0.98, true)),
                new WaitCommand(1),
                new InstantCommand(() -> claw.setBrakeMode(true))
            )
        );

        driverController.leftBumper().whileTrue(
            new Align(
                    this,
                    new Pose2d(
                        Constants.AlignPositions.tag10R.getX(),
                        Constants.AlignPositions.tag10R.getY(),
                        Constants.AlignPositions.tag10R.getRotation()
                    ),
                    0.02
                )
        );

        driverController.a().whileTrue(new Align(this, new Pose2d(11.4, 4.1, Rotation2d.fromDegrees(180)), 0.1));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
