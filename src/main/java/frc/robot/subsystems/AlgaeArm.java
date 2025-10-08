package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeArm extends SubsystemBase {
    
    private final TalonFX motor;
    
    // Constants - adjust these values as needed
    private static final int MOTOR_CAN_ID = 10; // Change to your CAN ID
    private static final double INTAKE_SPEED = 0.5; // Speed when intaking
    private static final double HOLD_SPEED = 0.15; // Speed to hold algae
    private static final double SHOOT_SPEED = -0.8; // Negative for opposite direction
    private static final double CURRENT_THRESHOLD = 20.0; // Amps - triggers hold mode
    
    private boolean isIntaking = false;
    private boolean hasAlgae = false;
    
    public AlgaeArm() {
        motor = new TalonFX(MOTOR_CAN_ID);
        configureMotor();
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
        motor.set(INTAKE_SPEED);
    }
    
    /**
     * Hold algae with reduced speed
     */
    private void hold() {
        isIntaking = false;
        hasAlgae = true;
        motor.set(HOLD_SPEED);
    }
    
    /**
     * Shoot algae out
     */
    private void shoot() {
        isIntaking = false;
        hasAlgae = false;
        motor.set(SHOOT_SPEED);
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
        // Monitor current during intake and auto-switch to hold
        if (isIntaking && getCurrent() > CURRENT_THRESHOLD) {
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