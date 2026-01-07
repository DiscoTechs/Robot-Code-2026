// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
//  * constants. This class should not be used for any other purpose. All constants should be declared
//  * globally (i.e. public static). Do not put anything functional in this class.
//  *
//  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
//  * constants are needed, to reduce verbosity.
//  */
// public final class Constants {
//     // ------------ MODULE CONSTANTS ------------ //
//     public static final class ModuleConstants {
//         public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
//         public static final double kDriveMotorGearRatio = 1 / 6.75;
//         public static final double kTurningMotorGearRatio = 7 / 150.0;
//         public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
//         public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
//         public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
//         public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
//         public static final double kPTurning = 1.00;
//     }

//     // ------------ DRIVE CONSTANTS ------------ //
//     public static final class DriveConstants {
//         public static final double kTrackWidth = Units.inchesToMeters(21.75);
//         // Distance between right and left wheels
//         public static final double kWheelBase = Units.inchesToMeters(21.75);
//         // Distance between front and back wheels
//         public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//                 new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
//                 new Translation2d(kWheelBase / 2, kTrackWidth / 2),
//                 new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
//                 new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

//         // SPARK MAXES
//         public static final double driveBaseRadius = Units.inchesToMeters(21.75 * Math.sqrt(2) / 2);

//         // drive 40 amps
//         public static final int kFrontLeftDriveMotorPort = 2;
//         public static final int kBackLeftDriveMotorPort = 6;
//         public static final int kFrontRightDriveMotorPort = 4;
//         public static final int kBackRightDriveMotorPort = 15;

//         // turning 20 amps
//         public static final int kFrontLeftTurningMotorPort = 1;
//         public static final int kBackLeftTurningMotorPort = 5;
//         public static final int kFrontRightTurningMotorPort = 3;
//         public static final int kBackRightTurningMotorPort = 7;

//         public static final boolean kFrontLeftTurningEncoderReversed = true;
//         public static final boolean kBackLeftTurningEncoderReversed = true;
//         public static final boolean kFrontRightTurningEncoderReversed = true;
//         public static final boolean kBackRightTurningEncoderReversed = true;

//         public static final boolean kFrontLeftDriveEncoderReversed = true;
//         public static final boolean kBackLeftDriveEncoderReversed = true;
//         public static final boolean kFrontRightDriveEncoderReversed = true;
//         public static final boolean kBackRightDriveEncoderReversed = true;

//         public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
//         public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
//         public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
//         public static final int kBackRightDriveAbsoluteEncoderPort = 0;

//         public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
//         public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
//         public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
//         public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

//         public static final int kLeftFrontModule = 0;
//         public static final int kRightFrontModule = 1;
//         public static final int kLeftBackModule = 2;
//         public static final int kRightBackModule = 3;

//         public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = (-301+180) * Math.PI / 180;
//         public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (-52.6+6) * Math.PI / 180;
//         public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (314 - 90) * Math.PI / 180;
//         public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (33 + 180) * Math.PI / 180;

//         public static final double kPhysicalMaxSpeedMetersPerSecond = 4.60248;
//         public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

//         public static final double kTeleDriveMaxSpeedMetersPerSecond = 4.5; //4.5; //kPhysicalMaxSpeedMetersPerSecond / 4;
//         public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
//                // kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
//         public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
//         public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
//     }

//     // ------------ AUTO CONSTANTS ------------ //
//     public static final class AutoConstants {
//         public static final double kMaxSpeedMetersPerSecond = 1.0;
//         public static final double kMaxAngularSpeedRadiansPerSecond = 
//                             DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
//         public static final double kMaxAccelerationMetersPerSecondSquared = 3;
//         public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
//         // public static final double kPXController = 1.5;
//         // public static final double kPYController = 1.5;
//         // public static final double kPThetaController = 3;

//         public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
//                 new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
//     }

//     // ------------ OI CONSTANTS ------------ //
//     public static final class OIConstants {

//         public static double DISTANCE_FROM_CENTER = 7;
//         public static final int kDriverControllerPort = 0;
//         public static final int kOperatorControllerPort = 1;
//         public static final int kButtonBoxPort = 2;

//         public static final int kDriverYAxis = 1;
//         public static final int kDriverXAxis = 0;
//         public static final int kDriverRotAxis = 4;
//         public static final int kDriverFieldOrientedButtonIdx = 1;

//         public static final double kDeadband = 0.1;

//         public static int XBX_L_X = 0;
//         public static int XBX_L_Y = 1;
//         public static int XBX_L_TRIG = 2;
//         public static int XBX_R_TRIG = 3;
//         public static int XBX_R_X = 4;
//         public static int XBX_R_Y = 5;

//         public static int XBX_A = 1;
//         public static int XBX_B = 2;
//         public static int XBX_X = 3;
//         public static int XBX_Y = 4;

//         public static int LEFT_BUMPER = 5;
//         public static int RIGHT_BUMPER = 6;

//         public static int ADJUST_LEFT = 7;
//         public static int ADJUST_RIGHT = 8;

//         public static int POSITION_1_ANGLE = 60;
//         public static int POSITION_3_ANGLE = 240;
//         public static int POSITION_2_ANGLE = 120;
//         public static int POSITION_4_ANGLE = 300;

//         public static int FACE_FORWARD_ANGLE = 0;
//         public static int FACE_BACKWARDS_ANGLE = 180;

//         public static int FACE_LEFT_ANGLE = 270; 
//         public static int FACE_RIGHT_ANGLE = 90;

        
//         //Don't change unless needed --> Buttons
//         public static int UP_POV = 0;
//         public static int DOWN_POV = 180;
//         public static int LEFT_POV = 270; //adjust if needed
//         public static int RIGHT_POV = 90; //adjust if needed


//     }

//     // DOUBLE CECK ALL BUTTON IDS BELOW

//     // ------------ ALGAE CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
//     public static final class AlgaeConstants {
//         //ALGAE CAN IDS
//         public static final int kLeftAlgaeEffectorMotorPort = 25;
//         public static final int kRightAlgaeEffectorMotorPort = 26;
//         public static final int kAlgaeAngleMotorPort = 24;
//             public static final int ALGAE_ANGLE_UP = 8;
//             public static final int ALGAE_ANGLE_DOWN = 7;
//             public static final int ALGAE_ANGLE_ESCAPE = 2;

        
//         public static final double kScoringEncoderValue = -14.38 - 0.2;//1.978;
//         public static final double kFloorIntakeValue = -19.65;//1.978;
//         public static final double kMaxEncoderValue = -0.3;
//         public static final double kMinEncoderValue = -17.5*1.2 - 0.5;
//         public static final double kAbsoluteMinEncoderValue = -16.4;

//         public static final double kAlgaeDelta = 0.3;

//         public static final int GoBetweenL2AndL3 = 7;
//         public static final double EncoderBetweenL2AndL3 = 34.6 - 2.5;
//         public static final int GoBetweenL3AndL4 = 8;
//         public static final double EncoderBetweenL3AndL4 = 53 - 2.5;

//         public static final double l1 = 0;
//         public static final double l2 = 20;
//         public static final double l3 = 35;
//         public static final double l4 = 50;

//         public static final double Delta = 0.5;

//         public static final double kAlgaeAngleSpeed = 0.2;
//         public static final double kSlowAlgaeAngleSpeed = 0.07;

//         //BUTTONS (XBOX Joytick)
//         public static final int ALGAE_INTAKE = 6; //good
//         public static final int ALGAE_OUTTAKE = 5; //good

//         public static final int ALGAE_ANGLE_STAY_MAX = 4; //good
//         public static final int ALGAE_ANGLE_STAY_MIN = 1; //good
//         public static final int ALGAE_ANGLE_SCORING = 3; //good
//     }

//     // ------------ CORAL CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
//     public static final class CoralConstants {
//         //CORAL CAN IDS
//         public static final int kCoralEffectorMotorPort = 28;
//         public static final int kCoralPlateAngleMotorPort = 32; //final CAN ID
//             public static final int CORAL_PLATE_ANGLE_DEFAULT = 3; //17
//             public static final int CORAL_PLATE_STAY_MAX = 9;
//             public static final int CORAL_PLATE_STAY_MIN = 7;
        

//         //CORAL_PLATE_ANGLE
//         public static final double kAngleSpeed = 0.1; //default through-bore encoder value --> Angle Speed To Adjust the Plate
//         public static final double kDelta = 0.01; //small value that can be adjusted

//         public static final double kDefaultEncoder = 0.2; //default through-bore encoder value
//         public static final double MIN_ENCODER_VALUE = 0; //SET TO BE LARGER THAN ACTUAL MIN
//         public static final double MAX_ENCODER_VALUE = 1; //SET TO BE SMALLER THAN ACTUAL MAX
        
//         public static final double l1Speed = 0.3;
//         public static final double l2speed = 0.5;
//         public static final double l3speed = 0.5;
//         public static final double l4speed = 0.3;

//         public static final double defaultSpeed = 0.5;

//         //BUTTONS (BUTTON BOX)
//         public static final int CORAL_PLATE_ANGLE_UP = 12; //good
//         public static final int CORAL_PLATE_ANGLE_DOWN = 11; //good

//         public static final int CORAL_OUTTAKE = 2; //good
//         public static final int CORAL_INTAKE = 3; //good
        
//         public static final int INITIATE_HUMAN_PLAYER_SEQUENCE = 8; //good
//         public static final int ESCAPE = 10; //good
//     }

//     // ------------ ELEVATOR CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
//     public static final class ElavatorConstants {
//         //ELEVATOR CAN IDS
//         public static final int kRightElevatorMotorPort = 19;
//         public static final int kLeftElevatorMotorPort = 21;
        
//         public static final int MANUAL_CONTROL_AXIS = 1;
        
//         //SENSOR PORT --> I Have A SUSPICION THAT LEVEL TWO IS SWITCHED WITH EITHER LEVEL 1 OR LIMIT SENSOR
//         public static final int CORAL_SENSOR = 2;
//         public static final int ELAVATOR_SENSOR_1 = 5;
//         public static final int ELAVATOR_SENSOR_2 = 6;
//         public static final int ELAVATOR_SENSOR_3 = 7;
//         public static final int ELAVATOR_SENSOR_4 = 4;
//         public static final int ELAVATOR_LIMIT_SENSOR = 3;

//         //BUTTONS (BUTTON BOX)
//         public static final int LEVEL_1 = 5; //good
//         public static final int LEVEL_2 = 4; //good
//         public static final int LEVEL_3 = 1; //good
//         public static final int LEVEL_4 = 6; //good
//         public static final int USE_MANUAL_CONTROL_JOYSTICK = 7; //good
//     }

//     // ------------ CLIMBER CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
//     public static final class ClimberConstants {
//         //CLIMBER CAN IDS
//         public static final int kClimberMotorPort = 23;
//     }




//     // ------------ VISION CONSTANTS ------------ //
//     public static final class VisionConstants {
//         public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

//         static { DISTANCE_TO_ANGLE_MAP.put(30.0, .75); }
//         public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOT_MAP = new InterpolatingDoubleTreeMap();
//         static { DISTANCE_TO_SHOOT_MAP.put(30.0, 4000.0); }
//     }
// }