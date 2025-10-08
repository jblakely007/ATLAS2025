package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeArm extends SubsystemBase {
    
    private final TalonFX motor;
    
    // Constants - adjust these values as needed
    private static final int MOTOR_CAN_ID = 13;
    private static final double DEFAULT_INTAKE_SPEED = 0.5;
    private static final double DEFAULT_HOLD_SPEED = 0.15;
    private static final double DEFAULT_SHOOT_SPEED = -0.8;
    private static final double DEFAULT_CURRENT_THRESHOLD = 20.0;
    
    // Tunable values via Shuffleboard
    private final GenericEntry intakeSpeedEntry;
    private final GenericEntry holdSpeedEntry;
    private final GenericEntry shootSpeedEntry;
    private final GenericEntry currentThresholdEntry;
    
    // Current values
    private final GenericEntry currentDrawEntry;
    private final GenericEntry hasAlgaeEntry;
    
    private boolean isIntaking = false;
    private boolean hasAlgae = false;
    
    public AlgaeArm() {
        motor = new TalonFX(MOTOR_CAN_ID);
        configureMotor();
        
        // Create Shuffleboard tab for AlgaeArm
        ShuffleboardTab tab = Shuffleboard.getTab("AlgaeArm");
        
        // Add tunable speed values with static defaults
        intakeSpeedEntry = tab.add("Intake Speed", DEFAULT_INTAKE_SPEED)
            .withPosition(0, 0)
            .getEntry();
        holdSpeedEntry = tab.add("Hold Speed", DEFAULT_HOLD_SPEED)
            .withPosition(0, 1)
            .getEntry();
        shootSpeedEntry = tab.add("Shoot Speed", DEFAULT_SHOOT_SPEED)
            .withPosition(0, 2)
            .getEntry();
        currentThresholdEntry = tab.add("Current Threshold", DEFAULT_CURRENT_THRESHOLD)
            .withPosition(0, 3)
            .getEntry();
            
        // Add read-only status values
        currentDrawEntry = tab.add("Current Draw", 0.0)
            .withPosition(1, 0)
            .getEntry();
        hasAlgaeEntry = tab.add("Has Algae", false)
            .withPosition(1, 1)
            .getEntry();
    }
    
    private void configureMotor() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    /**
     * Start intaking algae at full intake speed
     */
    private void intake() {
        isIntaking = true;
        hasAlgae = false;
        motor.set(intakeSpeedEntry.getDouble(DEFAULT_INTAKE_SPEED));
    }
    
    /**
     * Hold algae with reduced speed
     */
    private void hold() {
        isIntaking = false;
        hasAlgae = true;
        motor.set(holdSpeedEntry.getDouble(DEFAULT_HOLD_SPEED));
    }
    
    /**
     * Shoot algae out
     */
    private void shoot() {
        isIntaking = false;
        hasAlgae = false;
        motor.set(shootSpeedEntry.getDouble(DEFAULT_SHOOT_SPEED));
    }
    
    /**
     * Stop the motor
     */
    private void stop() {
        isIntaking = false;
        motor.set(0);
    }
    
    /**
     * Check if we currently have algae
     */
    public boolean hasAlgae() {
        return hasAlgae;
    }
    
    /**
     * Get current draw from motor
     */
    private double getCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }
    
    @Override
    public void periodic() {
        // Update telemetry
        currentDrawEntry.setDouble(getCurrent());
        hasAlgaeEntry.setBoolean(hasAlgae);
        
        // Monitor current during intake and auto-switch to hold
        if (isIntaking && getCurrent() > currentThresholdEntry.getDouble(DEFAULT_CURRENT_THRESHOLD)) {
            hold();
        }
    }
    
    /**
     * Command to intake algae - automatically transitions to hold when detected
     */
    public Command intakeCommand() {
        return this.runOnce(() -> {
            intake();
        });
    }
    
    /**
     * Command to shoot algae - runs until button is released, then returns to intake
     */
    public Command shootCommand() {
        return this.run(() -> {
            shoot();
        }).finallyDo(() -> {
            intake(); // Always return to intake mode after shooting
        });
    }
    
    /**
     * Command to manually stop the motor
     */
    public Command stopCommand() {
        return this.runOnce(() -> {
            stop();
        });
    }
    
    /**
     * Command to hold algae at reduced speed
     */
    public Command holdCommand() {
        return this.runOnce(() -> {
            hold();
        });
    }
}