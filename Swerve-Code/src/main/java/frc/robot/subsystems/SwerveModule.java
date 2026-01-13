// // Jake McQuade //
// // Team 1099 //

// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.EncoderConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.ModuleConstants;

// public class SwerveModule {
//     private final SparkMax driveMotor;
//     private final SparkMax turnMotor;
//     private final RelativeEncoder driveEncoder;
//     private final RelativeEncoder turnEncoder;
//     private final PIDController turnPIDController;
//     private final AnalogInput absoluteEncoder;
//     private final boolean absoluteEncoderInverted;
//     private final double absoluteEncoderOffsetRad;

//     public SwerveModule(
//         int driveId,
//         int turningId,
//         boolean driveInverted,
//         boolean turningInverted,
//         int absoluteEncoderId,
//         double absoluteEncoderOffset,
//         boolean absoluteEncoderInverted
//     ) {
//         absoluteEncoder = new AnalogInput(absoluteEncoderId);
//         driveMotor = new SparkMax(driveId, MotorType.kBrushless);
//         turnMotor = new SparkMax(turningId, MotorType.kBrushless);
//         driveEncoder = driveMotor.getEncoder();
//         turnEncoder = turnMotor.getEncoder();

//         this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
//         this.absoluteEncoderInverted = absoluteEncoderInverted;

//         // Configure Motors
//         SparkMaxConfig driveConfig = new SparkMaxConfig();
//         driveConfig.smartCurrentLimit(40);
//         driveConfig.inverted(driveInverted);

//         SparkMaxConfig turnConfig = new SparkMaxConfig();
//         turnConfig.smartCurrentLimit(20);
//         turnConfig.inverted(turningInverted);

//         // Configure Encoders
//         EncoderConfig driveEncoderConfig = new EncoderConfig();
//         driveEncoderConfig.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
//         driveEncoderConfig.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
//         driveConfig.encoder.apply(driveEncoderConfig);

//         EncoderConfig turnEncoderConfig = new EncoderConfig();
//         turnEncoderConfig.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
//         turnEncoderConfig.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
//         turnConfig.encoder.apply(turnEncoderConfig);

//         // Force Configuration
//         driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//         turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//         // Setup Turn PID Controller
//         turnPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
//         turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

//         // Reset Encoders
//         driveEncoder.setPosition(0);
//         turnEncoder.setPosition(getAbsoluteEncoderRad());
//     }

//     public double getDrivePosition() {
//         return driveEncoder.getPosition();
//     }

//     public double getTurningPosition() {
//         return turnEncoder.getPosition();
//     }

//     public double getDriveVelocity() {
//         return driveEncoder.getVelocity();
//     }

//     public double getTurningVelocity() {
//         return driveEncoder.getVelocity();
//     }

//     public double getAbsoluteEncoderRad() {
//         double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
//         angle *= 2.0 * Math.PI;
//         angle -= absoluteEncoderOffsetRad;

//         return angle * (absoluteEncoderInverted ? -1.0 : 1.0);
//     }

//     public double getAbsoluteEncoderDeg() {
//         return getAbsoluteEncoderRad() * 180 / Math.PI;
//     }

//     public SwerveModuleState getState() {
//         return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
//     }

//     public void setDesiredState(SwerveModuleState state) {
//         if (Math.abs(state.speedMetersPerSecond) < 0.001) {
//             stop();
//             return;
//         }

//         state.optimize(getState().angle);
//         driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
//         turnMotor.set(turnPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
//         SmartDashboard.putString(String.format("Swerve[%s] state", absoluteEncoder.getChannel()), state.toString());
//     }

//     public void stop() {
//         driveMotor.set(0);
//         turnMotor.set(0);
//     }
// };