package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
/**
 * define Hardware Ports in here
 */
public class RobotMap {

    public static final class ModuleConstants {
        // Not Updated
        // Note: these are for the drive and turning motors
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
        public static final double DRIVE_GEAR_RATIO = 1 / 4.71; // 14T pinion
        public static final double TURN_GEAR_RATIO = 1 / (1); // this should be 1 this is correct 
        public static final double WRIST_GEAR_RATIO = 1 / 1; //TODO TBD 44.44 / 1 

        public static final double FREE_MOTOR_SPEED_RPS = 5676 / 60; // RPM/60
        public static final double DRIVE_FREE_MAX_SPEED_MPS = (FREE_MOTOR_SPEED_RPS * WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVE_GEAR_RATIO;
        public static final double DRIVE_ROTATIONS_TO_METERS = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ROTATIONS_TO_RAD = TURN_GEAR_RATIO * 2 * Math.PI;


        public static final double DRIVE_RPM_TO_METERS_PER_SEC = DRIVE_ROTATIONS_TO_METERS / 60;
        public static final double TURNING_RPM_TO_RAD_PER_SEC = TURNING_ROTATIONS_TO_RAD / 60;
        public static final double P_TURNING = 0.075; // PID constant
        public static final double I_TURNING = 0.000001; // PID Constant
        public static final double D_TURNING = 0.00025; // PID constant
        public static final double P_DRIVE = 0.25; // PID constant

        public static final int DRIVE_CURRENT_LIMIT = 50; // amps
        public static final int TURN_CURRENT_LIMIT = 20;

        public static final double FL_ANGLE_OFFSET = Math.PI / 2; //
        public static final double FR_ANGLE_OFFSET = Math.PI; //
        public static final double BL_ANGLE_OFFSET = 0;
        public static final double BR_ANGLE_OFFSET = -Math.PI / 2; //

        // public static final double FL_ANGLE_OFFSET = Math.PI / 2 - Math.PI/2;
        // public static final double FR_ANGLE_OFFSET = Math.PI - Math.PI/2; //
        // public static final double BL_ANGLE_OFFSET = 0 - Math.PI/2;
        // public static final double BR_ANGLE_OFFSET = -Math.PI / 2 - Math.PI/2; //

        public static final double CAMERA_ANGLE = 0;
        public static final double SPEAKER_TAG_HEIGHT = 0;
    }
    // Not Updated
    // All the overall constants for the drivetrain
    public static final class DriveConstants {

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

        // FL is front left, BR is back right, etc.

        public static final int FL_DRIVE_PORT = 2;
        public static final int BL_DRIVE_PORT = 4;
        public static final int FR_DRIVE_PORT = 6;
        public static final int BR_DRIVE_PORT = 8;

        public static final int FL_TURN_PORT = 3;
        public static final int BL_TURN_PORT = 5;
        public static final int FR_TURN_PORT = 7;
        public static final int BR_TURN_PORT = 9;

        // Sometimes encoders are mounted backwards based on robot design, this fixes
        // that although it's not a thing on stargazer
        public static final boolean FL_TURN_REVERSED = false;
        public static final boolean BL_TURN_REVERSED = false;
        public static final boolean FR_TURN_REVERSED = false;
        public static final boolean BR_TURN_REVERSED = false;

        public static final boolean FL_DRIVE_REVERSED = true;
        public static final boolean BL_DRIVE_REVERSED = true;
        public static final boolean FR_DRIVE_REVERSED = true;
        public static final boolean BR_DRIVE_REVERSED = true;

        public static final boolean FL_ABS_REVERSED = true;
        public static final boolean BL_ABS_REVERSED = true;
        public static final boolean FR_ABS_REVERSED = true;
        public static final boolean BR_ABS_REVERSED = true;

        // The physical max if motors go full speed
        public static final double MAX_METERS_PER_SEC = 15; // m/s earlier -> 10
        public static final double MAX_RADIANS_PER_SEC = 30; // rad/s earlier -> 12

        public static final double MAX_LINEAR_ACCEL = 20; // m/s/s  
        public static final double MAX_ANGULAR_ACCEL = 30; // rad/s/s earlier -> 23

    }

    public static final int SHOOT_LEFT = 11; 
    public static final int SHOOT_RIGHT = 10; 
    public static final int WRIST_MOTOR = 12; 
    public static final int INDEXER_MOTOR = 13; 
    public static final int INTAKE_MOTOR = 14;
    public static final int ELEVATOR_MOTOR = 16;

    public static final int PIGEON = 18;

    public static final class LightConstants {
        public static final int LED_PORT = 7;
        public static final int COUNT = 60;

    }

}