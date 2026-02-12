// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class OperatorSubsystem extends SubsystemBase {
//     private final ClimberSubsystem climber;
//     private final IndexerSubsystem shooter;

//     public OperatorSubsystem(ClimberSubsystem climb, IndexerSubsystem ind) {
//         this.climber = climb;
//         this.shooter = ind;
//     }

//     public Command indexerIntake() { return shooter.forward(); }
//     public Command indexerOuttake() { return shooter.reverse(); }
//     public Command indexerStop() { return shooter.stop(); }

//     public Command climbUp() { return climber.climbUp(); }
//     public Command climbDown() { return climber.climbDown(); }

//     public Command intakeAll() {
//         return Commands.parallel(
//             shooter.forward().asProxy()
//         );
//     }

//     public Command outtakeAll() {
//         return Commands.parallel(
//             shooter.reverse().asProxy()
//         );
//     }
// }
