package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveForwardCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final double targetDistanceMeters = 0.3048; // 1 foot
    private final double maxSpeedMetersPerSecond = 0.5;
    private final double tolerance = 0.02; // 2cm tolerance

    private Pose2d startPose;
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
    private double directionMultiplier = 1.0; // +1 = forward (red), -1 = backward (blue)
    
    // PID Controller for distance control
    private final PIDController pidController;
    private static final double kP = 2.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    public DriveForwardCommand(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        pidController.setSetpoint(targetDistanceMeters);
    }

    @Override
    public void initialize() {
        startPose = swerve.getState().Pose;

        // Blue alliance -> move toward its own wall (-X)
        // Red alliance -> move toward its own wall (+X)
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            directionMultiplier = -1.0;
        } else {
            directionMultiplier = 1.0;
        }
        
        pidController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getState().Pose;
        
        // Calculate distance traveled in X direction
        double distanceTraveled = Math.abs(currentPose.getX() - startPose.getX());
        
        // Calculate PID output based on distance error
        double pidOutput = pidController.calculate(distanceTraveled);
        
        // Clamp output to max speed
        double speed = Math.max(-maxSpeedMetersPerSecond, 
                               Math.min(maxSpeedMetersPerSecond, pidOutput));
        
        // Apply direction multiplier and drive
        swerve.setControl(driveRequest
            .withVelocityX(speed * directionMultiplier)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerve.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}