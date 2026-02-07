package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberDriveMotor;


    public ClimberSubsystem() {
        CANBus canbus = CANBus.roboRIO();
        climberDriveMotor = new TalonFX(Constants.ClimberConstants.kClimberDriveId, canbus);

    }
}
