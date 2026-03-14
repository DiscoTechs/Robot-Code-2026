package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSlideSubsystem extends SubsystemBase {
    private final TalonFX motor;

    private final DigitalInput inside = new DigitalInput(0);
    private final DigitalInput outside = new DigitalInput(1);

    public IntakeSlideSubsystem() {
        motor = new TalonFX(3, new CANBus("CANivore 1"));
    }

    public Command extend() {
        return Commands.run(() -> motor.set(!outside.get() ? 0 : 0.25), this)
            .until(() -> !outside.get())
            .andThen(stop());
    }

    public Command retract() {
        return Commands.run(() -> motor.set(!inside.get() ? 0 : -0.25), this)
                .until(() -> !inside.get())
                .andThen(stop());
    }

    public Command stop() {
        return Commands.run(() -> motor.set(0), this);
    }

    @Override
    public void periodic() {
        // System.out.println("I: " + inside.get() + " O: " + outside.get());
    }
}
