// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(50);
    public static final Matter CHASSIS_MATTER = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)),
            ROBOT_MASS_KG);
    public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(14.5); // Robot maximum speed (m/s). Used to
                                                                                // limit acceleration.
    public static final double WHEEL_LOCK_TIME_SEC = 10; // Hold time on motor brakes when disabled (seconds)
    public static final double CONTROL_LOOP_PERIOD_SEC = 0.13; // s, 20ms + 110ms spark max velocity lag
    public static final double ROBOT_SQUARE_DIMENSIONS = Units.inchesToMeters(21.75);

    public static class Limelight {
        public static final boolean ENABLED = false;
        public static final Pose3d ROBOT_TO_CAMERA_POSE = new Pose3d(
                Inches.of(0).in(Meters),
                Inches.of(0).in(Meters),
                Inches.of(0).in(Meters),
                new Rotation3d(0, Degrees.of(0).in(Radians), Degrees.of(0).in(Radians)));
    }

    public static class PathplannerConstants {
        public static final boolean enableFeedforward = true;
        public static PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(
                new PIDConstants(2.0, 0.0, 0.3), // Translation PID constants
                new PIDConstants(2.0, 0.0, 0.5) // Rotation PID constants
        );
        public static final RobotConfig config = new RobotConfig(
                ROBOT_MASS_KG,
                1/12 * ROBOT_MASS_KG * (Math.pow(ROBOT_SQUARE_DIMENSIONS, 2) + Math.pow(ROBOT_SQUARE_DIMENSIONS, 2)), // Robot MOI 1/12 (kg*m^2)
                new ModuleConfig(
                        0.0508,
                        4.473,
                        1.1,
                        DCMotor.getNEO(1).withReduction(6.75),
                        50.0,
                        1),
                // Swerve Drive Module Locations
                // should be in FL, FR, BL, BR order.
                new Translation2d(0.2175, 0.2175), // FL
                new Translation2d(0.2175, -0.2175), // FR
                new Translation2d(-0.2175, 0.2175), // BL
                new Translation2d(-0.2175, -0.2175) // BR
        );
    }

    public static class OperatorConstants {
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int DRIVER_CONTROLLER_PORT = 0;
        // Multiplier for all joystick values
        // Smaller value sets lower max speed
        public static final double TRANSLATION_SCALE = 0.8;
        // ignores values until they hit this threshold
        // makes robot less sensitive to joystick movement
        public static final double JOYSTICK_DEADBAND = 0.1; // Joystick Deadband
    }

    public static class ClimberConstants {
        public static final Distance STARTING_HEIGHT = Meters.of(0);
        public static final Mass MASS = Pounds.of(16);
        public static final int CLIMBER_MOTOR_CAN_ID = 1;
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTOR_CAN_ID = 2; // make 2 for actual robot
    }

    public static class TurretConstants {
        public static final int TURRET_MOTOR_CAN_ID = 3;
    }

    public static class IndexerConstants {
        public static final int INDEXER_MOTOR_CAN_ID = 6; 
    }
    public static class IntakeConstants{
        public static final int INTAKE_PIVOT_CAN_ID = 7;
        public static final int INTAKE_CAN_ID = 8; // change later mabye
    }
    public static class KickerConstants{
        public static final int KICKER_CAN_ID = 4;
        
    }
}
