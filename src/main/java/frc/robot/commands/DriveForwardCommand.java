package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveForwardCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final double targetDistanceMeters = 0.3048; // 1 foot
    private final double speedMetersPerSecond = 0.5;    // 0.5 m/s (~slow and steady)

    private Pose2d startPose;
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
    private double directionMultiplier = 1.0; // +1 = forward, -1 = backward

    public DriveForwardCommand(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
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
    }

    @Override
    public void execute() {
        swerve.setControl(
            driveRequest.withVelocityX(speedMetersPerSecond * directionMultiplier)
                        .withVelocityY(0)
                        .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d current = swerve.getState().Pose;
        double deltaX = (current.getX() - startPose.getX()) * directionMultiplier;
        return deltaX >= targetDistanceMeters;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
            driveRequest.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
        );
    }
}