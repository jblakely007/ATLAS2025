// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// Ensure this import is present
import frc.robot.commands.DriveForwardCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AlgaeArm;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AlgaeArm algaeArm = new AlgaeArm();

    public RobotContainer() {
        configureBindings();
    }

    /**
     * Configures all joystick button bindings and default commands for the robot.
     *
     * Setup for match start:
     * 1. Robot should start in a disabled state where idle mode is applied to drive motors
     * 2. Upon entering teleop mode, the intake will automatically activate
     * 3. Press left bumper once at the beginning to reset the field-centric heading
     *
     * Drive controls:
     * - Left joystick Y-axis: Forward/backward movement
     * - Left joystick X-axis: Left/right movement
     * - Right joystick X-axis: Rotation (counterclockwise/clockwise)
     * - Button A: Brake mode (holds position)
     * - Button B: Point mode (aims modules based on left joystick)
     *
     * Game piece controls:
     * - Right trigger: Shoot algae
     * - Entering teleop: Automatically starts intake
     * - Exiting teleop: Automatically stops intake
     *
     * Calibration controls (not for match use):
     * - Back + Y: SysId dynamic forward
     * - Back + X: SysId dynamic reverse
     * - Start + Y: SysId quasistatic forward
     * - Start + X: SysId quasistatic reverse
     */
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Start intake when entering teleop mode
        RobotModeTriggers.teleop().onTrue(algaeArm.intakeCommand());

        // Stop intake when exiting telop mode
        RobotModeTriggers.teleop().onFalse(algaeArm.stopCommand());

        // Right trigger shoots algae
        joystick.rightTrigger().whileTrue(algaeArm.shootCommand());
    }

    public Command getAutonomousCommand() {
        final double kFeetToMeters = 0.3048;
        return Commands.sequence(
            // Reset heading at start of autonomous
            Commands.runOnce(() -> drivetrain.seedFieldCentric()),
            // Drive foward 1 foot in robot-centric mode
            new DriveForwardCommand(drivetrain, 1.0 * kFeetToMeters, false)
        );
    }
}
