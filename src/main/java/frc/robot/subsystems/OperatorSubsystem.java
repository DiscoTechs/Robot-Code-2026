package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorSubsystem extends SubsystemBase {
    private final ClimberSubsystem climber;
    private final IndexerSubsystem indexer;

    public OperatorSubsystem(ClimberSubsystem climb, IndexerSubsystem ind) {
        this.climber = climb;
        this.indexer = ind;
    }

    public Command indexerIntake() { return indexer.forward(); }
    public Command indexerOuttake() { return indexer.reverse(); }
    public Command indexerStop() { return indexer.stop(); }

    public Command climbUp() { return climber.climbUp(); }
    public Command climbDown() { return climber.climbDown(); }

    public Command intakeAll() {
        return Commands.parallel(
            indexer.forward().asProxy()
        );
    }

    public Command outtakeAll() {
        return Commands.parallel(
            indexer.reverse().asProxy()
        );
    }
}
