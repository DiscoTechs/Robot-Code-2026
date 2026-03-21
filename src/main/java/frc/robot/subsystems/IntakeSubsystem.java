package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonSRX motor;

    public IntakeSubsystem() {
        this.motor = new TalonSRX(IntakeConstants.MOTOR_CAN_ID); // change to what it actucaly is
    }

    public Command forward() {
        return Commands.runOnce(() -> motor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED));
    }
    
    public Command reverse() {
        return Commands.runOnce(() -> motor.set(TalonSRXControlMode.PercentOutput, -IntakeConstants.INTAKE_SPEED));
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.set(TalonSRXControlMode.PercentOutput, 0));
    }
}
