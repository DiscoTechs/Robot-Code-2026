package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterDriveMotor;
    private final TalonFX shooterAngleMotor;
    
    /**
     * 
     */
    public ShooterSubsystem() {
        CANBus canbus = CANBus.roboRIO();
        shooterDriveMotor = new TalonFX(Constants.ShooterConstants.kShooterDriveId, canbus);
        shooterAngleMotor = new TalonFX(Constants.ShooterConstants.kShooterAngleId, canbus);
    }
}
