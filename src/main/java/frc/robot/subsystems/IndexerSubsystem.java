package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
        public IndexerSubsystem() {}
    
        public Command forward() {
            return Commands.runOnce(() -> {});
        }

        public Command reverse() {
            return Commands.runOnce(() -> {});
        }

        public Command setSpeed(double speed) {
            return Commands.runOnce(() -> {});
        }

        public Command stop() {
            return Commands.runOnce(() -> {});
        }
}
