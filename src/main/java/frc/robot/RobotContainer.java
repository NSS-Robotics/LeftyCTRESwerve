
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Align;
import frc.robot.commands.ClawOuttake;
import frc.robot.commands.IntakeGround;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

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
        addCommandsToPathplanner();
        CameraServer.startAutomaticCapture();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        driverController.y().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric())); // TODO: button so l4 and zero gyro arent the same thing

        //align 
        driverController.a().whileTrue(new Align(this, new Pose2d(11.4, 4.1, Rotation2d.fromDegrees(180)), 0.1));

        // move up to l4/l3
        operatorController.y().onTrue(
            new SequentialCommandGroup(
                //l4
                new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.l4.ordinal()])),
                new WaitUntilCommand(()-> elevator.getEncoder() > 1.5),
                new InstantCommand(() -> armPivot.setPosition(-1.95751953125, false))
            )
        );

        operatorController.x().onTrue(
            new SequentialCommandGroup(
                // score l3
                new InstantCommand(() -> elevator.setPosition(1.3)),
                new WaitCommand(0.5), // BAD
                new InstantCommand(() -> armPivot.setPosition(
                    Constants.ArmPivotConstants.pos[RobotState.l3.ordinal()], false
                )),
                new WaitCommand(0.5), // BAD
                new InstantCommand(() -> elevator.setPosition(
                    Constants.ElevatorConstants.pos[RobotState.l3.ordinal()]
                ))
            )
        );

        // align to scoring postition (left and right)
        operatorController.rightBumper().whileTrue(
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

        operatorController.leftBumper().whileTrue(
            new Align(
                    this,
                    new Pose2d(
                        Constants.AlignPositions.tag10L.getX(),
                        Constants.AlignPositions.tag10L.getY(),
                        Constants.AlignPositions.tag10L.getRotation()
                    ),
                    0.02
                )
        );

        driverController.leftTrigger().onTrue(
            new SequentialCommandGroup(
                // intake and shotgun
                new InstantCommand(() -> elevator.setPosition(1.5)),
                new IntakeGround(this), // CHANGE - to detect the coral?? 
                new InstantCommand(() -> armPivot.setPosition((Constants.ArmPivotConstants.pos[RobotState.start.ordinal()]), true)),
                new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.coralIntake.ordinal()])),
                new InstantCommand(() -> claw.makeMotorSpin(-0.4)),
                new WaitCommand(1), // TODO: make WaitUntilCommand
                new InstantCommand(() -> claw.makeMotorSpin(-0.07))
            )
        );

        driverController.rightTrigger().onTrue(
            new SequentialCommandGroup(
                // score
                new InstantCommand(() -> claw.setBrakeMode(false)),
                new InstantCommand(() -> armPivot.setPosition(-0.98, true)),
                new WaitCommand(1),
                new InstantCommand(() -> claw.setBrakeMode(true)),
               
                // reset all to home position
                new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.start.ordinal()], false)),
                new InstantCommand(() -> elevator.setPosition(1.5)),
                //new WaitUntilCommand(()-> elevator.getEncoder() > 1.4 && elevator.getEncoder() < 1.6),
                //new InstantCommand(() -> armPivot.setPosition(0.4, false)),
                new WaitCommand(1),
                new InstantCommand(() -> elevator.setPosition(0.8))
            )
        );
        
        driverController.b().onTrue(
            new SequentialCommandGroup(
                // reset all to home position
                new InstantCommand(() -> elevator.setPosition(1.5)),
                new WaitCommand(1),
                //new WaitUntilCommand(()-> elevator.getEncoder() > 1.4 && elevator.getEncoder() < 1.6),
                new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.start.ordinal()], false)),
             //  new InstantCommand(() -> armPivot.setPosition(0.4, false)),
                new WaitCommand(1),
                new InstantCommand(() -> elevator.setPosition(0.8))
            )
        );

        //algae 
        driverController
            .povDown()
            .onTrue(
                new SequentialCommandGroup(
                    // descore algae from reef
                    new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.algaeReefLow.ordinal()])),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.algaeReefLow.ordinal()], false)),
                    new InstantCommand(() -> claw.makeMotorSpin(-0.4))
                )
            );
        
        driverController
            .leftBumper()
            .onTrue(new InstantCommand(() -> new IntakeGround(this).end(false)));
        
        // emotional support comments
        
        // testing pid values elevator goes up and down
        // driverController
        //     .povRight()
        //     .onTrue(
        //         new InstantCommand(() -> elevator.setPosition(3.7))
        // );    
        // driverController
        //     .povLeft()
        //     .onTrue(
        //         new InstantCommand(() -> elevator.setPosition(1))
        // );

        //driverController.povDown().whileTrue(new ClawOuttake(this,claw));
        //driverController.leftBumper          Left Align
    }


    //pathplanner commands
    private void addCommandsToPathplanner() {
        NamedCommands.registerCommand("shotgun", 
            new SequentialCommandGroup(
                new InstantCommand(() -> armPivot.setPosition((Constants.ArmPivotConstants.pos[RobotState.start.ordinal()]), true)),
                new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.coralIntake.ordinal()])),
                new InstantCommand(() -> claw.makeMotorSpin(-0.4)),
                new WaitCommand(1), // TODO: make WaitUntilCommand
                new InstantCommand(() -> claw.makeMotorSpin(-0.07)),
                new InstantCommand(() -> elevator.setPosition(Constants.ElevatorConstants.pos[RobotState.l4.ordinal()])),
                new WaitUntilCommand(()-> elevator.getEncoder() > 1.5),
                new InstantCommand(() -> armPivot.setPosition(-1.95751953125, false))   
            )
        );

        NamedCommands.registerCommand("ArmDown",
            new SequentialCommandGroup(
                new InstantCommand(() -> claw.setBrakeMode(false)),
                new InstantCommand(() -> armPivot.setPosition(-0.98, true)),
                new WaitCommand(1),
                new InstantCommand(() -> claw.setBrakeMode(true))
            )
        );

        NamedCommands.registerCommand("IntakeStop",
            new InstantCommand(() -> {
                intakePivot.setPosition(Constants.IntakeConstants.upPosition);
                indexer.stopIndexer();
                intake.stopIntake();   
            })
        );

        NamedCommands.registerCommand("IntakeStart", 
            new InstantCommand(() -> {
                intakePivot.setPosition(Constants.IntakeConstants.intakeStartPos);
                intake.setIntake(Constants.IntakeConstants.velocity, false);
                indexer.setIndexer(Constants.IndexerConstants.velocity);            
            })
        );

        NamedCommands.registerCommand("Home",
          new SequentialCommandGroup(
                // reset all to home position
                new InstantCommand(() -> elevator.setPosition(1.5)),
                new WaitCommand(1),
                //new WaitUntilCommand(()-> elevator.getEncoder() > 1.4 && elevator.getEncoder() < 1.6),
                new InstantCommand(() -> armPivot.setPosition(Constants.ArmPivotConstants.pos[RobotState.start.ordinal()], false)),
             //  new InstantCommand(() -> armPivot.setPosition(0.4, false)),
                new WaitCommand(1),
                new InstantCommand(() -> elevator.setPosition(0.8))
            )
        );


        NamedCommands.registerCommand("Align 10 Left",
            new Align(this, Constants.AlignPositions.tag10L, 0.02)
        );
        NamedCommands.registerCommand("Align 10 Right",
            new Align(this, Constants.AlignPositions.tag10R, 0.02)
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
