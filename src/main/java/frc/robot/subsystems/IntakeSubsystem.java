package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX m_motor = new TalonSRX(IntakeConstants.INTAKE_CAN_ID); // change to what it actucaly is
    private final double SPEED = 0.8;

    public Command forward() {
        return Commands.runOnce(() -> {
            System.out.println("FORWARD");
            m_motor.set(TalonSRXControlMode.PercentOutput, SPEED);
        });
    }
    
    // we prob don't need this but just in case
    public Command reverse() {
        return Commands.runOnce(() -> {
            System.out.println("REVERSE");
            m_motor.set(TalonSRXControlMode.PercentOutput, -SPEED);
        });
    }

    public Command stop() {
        return Commands.runOnce(() -> m_motor.set(TalonSRXControlMode.PercentOutput, 0));
    }
}
