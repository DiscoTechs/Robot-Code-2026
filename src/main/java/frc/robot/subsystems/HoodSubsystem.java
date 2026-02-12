package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    public HoodSubsystem() {}

    public void setAngle(Rotation2d angle) {}

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public void setZero() {}
    public void stop() {}
}
