package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * define Hardware Ports in here
 */
public class RobotMap {

    

    // public static final class ModuleConstants {
    //     public static final double SPARK_FLEX_ENCODER_RESOLUTION = 7168 / 1; // 7168 counts per rev
    //     public static final double SPARK_MAX_ENCODER_RESOLUTION = 7168 / 1; //42 counts per rev

    //     // Not Updated
    //     // Note: these are for the drive and turning motors
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
        public static final double DRIVE_GEAR_RATIO = 1 / 4.71; // 14T pinion
        public static final double TURN_GEAR_RATIO = 1 / (1); // this should be 1 this is correct 
        public static final double WRIST_GEAR_RATIO = 44.44 / 1; //TODO TBD 44.44 / 1 

        public static final double FREE_MOTOR_SPEED_RPS = 6784 / 60; // RPM/60 //confmried
        public static final double DRIVE_FREE_MAX_SPEED_MPS = (FREE_MOTOR_SPEED_RPS * WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVE_GEAR_RATIO;
        public static final double DRIVE_ROTATIONS_TO_METERS = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ROTATIONS_TO_RAD = TURN_GEAR_RATIO * 2 * Math.PI;



    //     public static final double DRIVE_RPM_TO_METERS_PER_SEC = DRIVE_ROTATIONS_TO_METERS / 60;
    //     public static final double TURNING_RPM_TO_RAD_PER_SEC = TURNING_ROTATIONS_TO_RAD / 60;
    //     public static final double P_TURNING = 0.075; // PID constant
    //     public static final double I_TURNING = 0.000001; // PID Constant
    //     public static final double D_TURNING = 0.00025; // PID constant
    //     public static final double P_DRIVE = 0.25; // PID constant

    //     public static final double P_WRIST = 0.075; // PID constant
    //     public static final double I_WRIST = 0.000001; // PID Constant
    //     public static final double D_WRIST = 0.00025; // PID constant

    //     public static final int DRIVE_CURRENT_LIMIT = 50; // amps
    //     public static final int TURN_CURRENT_LIMIT = 20;

    //     public static final double FL_ANGLE_OFFSET = Math.PI / 2; //
    //     public static final double FR_ANGLE_OFFSET = Math.PI; //
    //     public static final double BL_ANGLE_OFFSET = 0;
    //     public static final double BR_ANGLE_OFFSET = -Math.PI / 2; //

    //     // public static final double FL_ANGLE_OFFSET = Math.PI / 2 - Math.PI/2;
    //     // public static final double FR_ANGLE_OFFSET = Math.PI - Math.PI/2; //
    //     // public static final double BL_ANGLE_OFFSET = 0 - Math.PI/2;
    //     // public static final double BR_ANGLE_OFFSET = -Math.PI / 2 - Math.PI/2; //

    //     public static final double CAMERA_ANGLE = 0;
    //     public static final double SPEAKER_TAG_HEIGHT = 0;
    // }
    // // Not Updated
    // // All the overall constants for the drivetrain
    // public static final class DriveConstants {

        // Distance between right and left wheels (meters)
        public static final double ROBOT_WIDTH = 0.498475;//19 5/8 in
        // Distance between front and back wheels (meters)
        public static final double ROBOT_LENGTH = 0.4953; //19.5 in

        // Note positive x is forward
        // Wheel order: FR, FL, BR, BL
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2));

    //     // FL is front left, BR is back right, etc.

    //     public static final int FL_DRIVE_PORT = 2;
    //     public static final int BL_DRIVE_PORT = 4;
    //     public static final int FR_DRIVE_PORT = 6;
    //     public static final int BR_DRIVE_PORT = 8;

    //     public static final int FL_TURN_PORT = 3;
    //     public static final int BL_TURN_PORT = 5;
    //     public static final int FR_TURN_PORT = 7;
    //     public static final int BR_TURN_PORT = 9;

    //     // Sometimes encoders are mounted backwards based on robot design, this fixes
    //     // that although it's not a thing on stargazer
    //     public static final boolean FL_TURN_REVERSED = false;
    //     public static final boolean BL_TURN_REVERSED = false;
    //     public static final boolean FR_TURN_REVERSED = false;
    //     public static final boolean BR_TURN_REVERSED = false;

    //     public static final boolean FL_DRIVE_REVERSED = true;
    //     public static final boolean BL_DRIVE_REVERSED = true;
    //     public static final boolean FR_DRIVE_REVERSED = true;
    //     public static final boolean BR_DRIVE_REVERSED = true;

    //     public static final boolean FL_ABS_REVERSED = true;
    //     public static final boolean BL_ABS_REVERSED = true;
    //     public static final boolean FR_ABS_REVERSED = true;
    //     public static final boolean BR_ABS_REVERSED = true;

    //     // The physical max if motors go full speed
    //     public static final double MAX_METERS_PER_SEC = 10; 
    //     public static final double MAX_RADIANS_PER_SEC = 12; 

    //     public static final double MAX_LINEAR_ACCEL = 20; // m/s/s  
    //     public static final double MAX_ANGULAR_ACCEL = 23; // rad/s/s 

    // }

    public static final class MotorPorts{
        public static final int SHOOT_LEFT = 11; 
        public static final int SHOOT_RIGHT = 10; 
        public static final int WRIST_MOTOR = 12; 
        public static final int INDEXER_MOTOR = 14; 
        public static final int INTAKE_MOTOR = 13;
        public static final int ELEVATOR_MOTOR = 16;

        public static final int WRIST_ENCODER = 8;

        public static final int PIGEON = 18;
    }



    public static final class LightConstants {
        public static final int LED_PORT = 7;
        public static final int COUNT = 60;

    }

    public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; //10
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 100000; // radians per second //1.2
    public static final double kMagnitudeSlewRate = 100000; // percent per second (1 = 100%) //1.8
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(19.625);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(19.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2; //FL
    public static final int kRearLeftDrivingCanId = 4; //BL
    public static final int kFrontRightDrivingCanId = 6; //FR
    public static final int kRearRightDrivingCanId = 8; //BR

    public static final int kFrontLeftTurningCanId = 3; //FL
    public static final int kRearLeftTurningCanId = 5; //BL
    public static final int kFrontRightTurningCanId = 7; //FR
    public static final int kRearRightTurningCanId = 9; //BR

    public static final boolean kPigeonReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    //     public static final double P_TURNING = 0.075; // PID constant
    //     public static final double I_TURNING = 0.000001; // PID Constant
    //     public static final double D_TURNING = 0.00025; // PID constant
    //     public static final double P_DRIVE = 0.25; // PID constant

    public static final double kTurningP = 1; //1
    public static final double kTurningI = 0; //0
    public static final double kTurningD = 0; //0
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

}