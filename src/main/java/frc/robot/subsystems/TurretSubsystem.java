package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    public TurretSubsystem() {}

    public void setAngle(double angle) {}

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public void setZero() {}
    public void stop() {}
}
