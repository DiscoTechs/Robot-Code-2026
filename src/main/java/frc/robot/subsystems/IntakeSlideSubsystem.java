package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSlideSubsystem extends SubsystemBase {
    private final TalonFX motor;

    private final DigitalInput outside = new DigitalInput(Constants.IntakeSlideConstants.OUTSIDE_LIMIT_SWITCH_DIO);
    private final DigitalInput inside = new DigitalInput(Constants.IntakeSlideConstants.INSIDE_LIMIT_SWITCH_DIO);

    public IntakeSlideSubsystem() {
        motor = new TalonFX(Constants.IntakeSlideConstants.MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    }

    public Command extend() {
        return Commands.run(() -> motor.set(!outside.get() ? 0 : Constants.IntakeSlideConstants.SLIDE_SPEED), this)
            .until(() -> !outside.get())
            .andThen(stop());
    }

    public Command retract() {
        return Commands.run(() -> motor.set(!inside.get() ? 0 : -Constants.IntakeSlideConstants.SLIDE_SPEED), this)
                .until(() -> !inside.get())
                .andThen(stop());
    }

    public Command stop() {
        return Commands.run(() -> motor.set(0), this);
    }

    @Override
    public void periodic() {
        System.out.println("I: " + !inside.get() + " O: " + !outside.get());
    }
}
