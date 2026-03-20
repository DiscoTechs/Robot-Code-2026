package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
    private final TalonFX motor;

    public ConveyorSubsystem() {
        motor = new TalonFX(Constants.ConveyorConstants.MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    }

    public Command forward() {
        return Commands.run(() -> motor.set(Constants.ConveyorConstants.CONVEYOR_SPEED), this);
    }

    public Command reverse() {
        return Commands.run(() -> motor.set(-Constants.ConveyorConstants.CONVEYOR_SPEED), this);
    }

    public Command stop() {
        return Commands.run(() -> motor.set(0), this);
    }

}