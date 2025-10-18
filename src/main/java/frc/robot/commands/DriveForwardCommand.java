package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Autonomous command to drive the robot forward a set distance using PID control.
 * Can use either field-centric or robot-centric drive mode.
 */
public class DriveForwardCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final double targetDistanceMeters;
    private final double maxSpeedMetersPerSecond = 0.5;
    private final double tolerance = 0.02; // 2cm tolerance
    private final boolean useFieldCentric;

    private Pose2d startPose;

    // PID Controller for distance control
    private final PIDController pidController;
    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.05;

    // Timeout to prevent command from running forever
    // Calculate timeout based on distance: (distance / half_max_speed) + 1 second buffer
    private final double timeoutSeconds;
    private double startTime;

    /**
     * Creates a DriveForwardCommand with robot-centric drive and 1 foot distance (default)
     * @param swerve The swerve drivetrain subsystem
     */
    public DriveForwardCommand(CommandSwerveDrivetrain swerve) {
        this(swerve, 0.3048, false); // Default: 1 foot, robot-centric
    }

    /**
     * Creates a DriveForwardCommand with specified distance and robot-centric drive
     * @param swerve The swerve drivetrain subsystem
     * @param distanceMeters Distance to drive in meters
     */
    public DriveForwardCommand(CommandSwerveDrivetrain swerve, double distanceMeters) {
        this(swerve, distanceMeters, false); // Default to robot-centric
    }

    /**
     * Creates a DriveForwardCommand with specified drive mode and 1 foot distance
     * @param swerve The swerve drivetrain subsystem
     * @param useFieldCentric If true, uses field-centric drive; if false, uses robot-centric
     */
    public DriveForwardCommand(CommandSwerveDrivetrain swerve, boolean useFieldCentric) {
        this(swerve, 0.3048, useFieldCentric); // Default: 1 foot
    }

    /**
     * Creates a DriveForwardCommand with full configuration
     * @param swerve The swerve drivetrain subsystem
     * @param distanceMeters Distance to drive in meters
     * @param useFieldCentric If true, uses field-centric drive; if false, uses robot-centric
     */
    public DriveForwardCommand(CommandSwerveDrivetrain swerve, double distanceMeters, boolean useFieldCentric) {
        this.swerve = swerve;
        this.targetDistanceMeters = distanceMeters;
        this.useFieldCentric = useFieldCentric;

        // Calculate reasonable timeout: assume half max speed + 1 second safety buffer
        this.timeoutSeconds = (Math.abs(distanceMeters) / (maxSpeedMetersPerSecond / 2.0)) + 1.0;

        addRequirements(swerve);

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        pidController.setSetpoint(targetDistanceMeters);
    }

    @Override
    public void initialize() {
        startPose = swerve.getState().Pose;
        startTime = Timer.getFPGATimestamp();
        pidController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getState().Pose;

        // Calculate distance traveled in X direction
        double distanceTraveled = currentPose.getX() - startPose.getX();

        // Calculate PID output based on distance error
        double pidOutput = pidController.calculate(distanceTraveled);

        // Clamp output to max speed for safety
        double speed = Math.max(-maxSpeedMetersPerSecond,
                               Math.min(maxSpeedMetersPerSecond, pidOutput));

        // Apply appropriate drive mode
        if (useFieldCentric) {
            swerve.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(speed)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else {
            swerve.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(speed)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when at setpoint OR timeout exceeded
        return pidController.atSetpoint() ||
               (Timer.getFPGATimestamp() - startTime > timeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot using appropriate mode
        if (useFieldCentric) {
            swerve.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        } else {
            swerve.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }
}