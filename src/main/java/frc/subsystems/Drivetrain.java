// package frc.subsystems;



// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.hardware.Pigeon2;
// //import com.ctre.phoenix6.hardware.WPI_PigeonIMU;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.Kinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;
// // import frc.robot.RobotMap.AutoConstants;
// import frc.robot.RobotMap.DriveConstants;
// import frc.robot.RobotMap.ModuleConstants;


// public class Drivetrain extends SubsystemBase {
//     private static final double MAX_DRIVE_SPEED = Units.feetToMeters(10);
//     public final SwerveModule frontLeft = new SwerveModule(
//         DriveConstants.FL_DRIVE_PORT,
//         DriveConstants.FL_TURN_PORT,
//         DriveConstants.FL_DRIVE_REVERSED,
//         DriveConstants.FL_TURN_REVERSED,
//         DriveConstants.FL_ABS_REVERSED,
//         ModuleConstants.FL_ANGLE_OFFSET);

//     public final SwerveModule frontRight = new SwerveModule(
//         DriveConstants.FR_DRIVE_PORT,
//         DriveConstants.FR_TURN_PORT,
//         DriveConstants.FR_DRIVE_REVERSED,
//         DriveConstants.FR_TURN_REVERSED,
//         DriveConstants.FR_ABS_REVERSED,
//         ModuleConstants.FR_ANGLE_OFFSET);

//     public final SwerveModule backLeft = new SwerveModule(
//         DriveConstants.BL_DRIVE_PORT,
//         DriveConstants.BL_TURN_PORT,
//         DriveConstants.BL_DRIVE_REVERSED,
//         DriveConstants.BL_TURN_REVERSED,
//         DriveConstants.BL_ABS_REVERSED,
//         ModuleConstants.BL_ANGLE_OFFSET);

//     public final SwerveModule backRight = new SwerveModule(
//         DriveConstants.BR_DRIVE_PORT,
//         DriveConstants.BR_TURN_PORT,
//         DriveConstants.BR_DRIVE_REVERSED,
//         DriveConstants.BR_TURN_REVERSED,
//         DriveConstants.BR_ABS_REVERSED,
//         ModuleConstants.BR_ANGLE_OFFSET);

    
 //     private final AHRS gyro = new AHRS(SPI.Port.kMXP);
//     private final Pigeon2 pigeon = new Pigeon2(RobotMap.MotorPorts.PIGEON, "rio");
//     //private final WPI_PigeonIMU pigeon = new Pigeon2(RobotMap.MotorPorts.PIGEON, "rio");

    // public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.driveKinematics,
    //         new Rotation2d(0), getModulePositions(), new Pose2d());
            
    // private static final double TRACK_WIDTH_X = Units.inchesToMeters(24.0);
    
    // private static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.0);
    
    // private final Field2d m_field = new Field2d();
    
    // private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    
//     private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR

//     public Drivetrain() {
//      //   PigeonConfig();
    //         AutoBuilder.configureHolonomic(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         () -> kinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                 4.5, // Max module speed, in m/s
    //                 0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //                 new ReplanningConfig() // Default path replanning config. See the API for the options here
    //         ),
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // ); 
// }

//     // Pigeon Stuff
//     public void zeroHeading() {
//         pigeon.reset();
//      //   pigeon.setAngleAdjustment(0);
//     }

//     public void PigeonConfig(){
//         Pigeon2Configuration configs = new Pigeon2Configuration();
//         // mount X-up
//         configs.MountPose.MountPoseYaw = 0;
//         configs.MountPose.MountPosePitch = 90;
//         configs.MountPose.MountPoseRoll = 0;
//         configs.Pigeon2Features.DisableNoMotionCalibration = false;
//         configs.Pigeon2Features.DisableTemperatureCompensation = false;
//         configs.Pigeon2Features.EnableCompass = false;
//         pigeon.setYaw(0);
//         pigeon.getConfigurator().apply(configs);
//     }

//     // public void setAngleAdjustment(double angle) {
//     //     pigeon.setAngleAdjustment(angle);   
//     // }



//     // Returns degrees from -180 to 180
//     public double getHeading() {
//         return Math.IEEEremainder(pigeon.getAngle(), 360);
//     }

//     // public double getPitch(){
//     //     return Math.IEEEremainder(gyro.getPitch(), 360);
//     // }

//     // public double getRoll(){
//     //     return Math.IEEEremainder(gyro.getRoll(), 360);
//     // }

//     public Rotation2d getRotation2d() {
//         return Rotation2d.fromDegrees(getHeading());
//     }

//     // Position of the robot
//     public Pose2d getPose() {
//         return poseEstimator.getEstimatedPosition();
//     }

    // public void resetPose(Pose2d kms) {
    //     poseEstimator.resetPosition(getRotation2d(), getModulePositions(), kms);
    // }

//     public void resetOdometry(Pose2d pose) {
//         poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
//     }

//     // To stop sliding
//     public void setX() {
//         frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
//         frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
//         backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
//         backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
//     }

//     // //Use with vision
//     // public void updateOdometryLatency(Pose2d measuredPose, double timestamp) {
//     //     Pose2d currentPose = getPose();
//     //     double xDiff = Math.abs(currentPose.getX() - measuredPose.getX());
//     //     double yiff = Math.abs(currentPose.getY() - measuredPose.getY());

//     //     //Check that measured pose isn't more than 1m away
//     //     if(Math.hypot(xDiff, yiff) <= 1) {
//     //         poseEstimator.addVisionMeasurement( measuredPose, timestamp);
//     //     } 
//     // }

//     public void updateFieldPose() {
//         m_field.setRobotPose(getPose());

//     }

//     @Override
//     public void periodic() {
//         poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getModulePositions());
//         updateFieldPose();

//         SmartDashboard.putNumber("Robot Heading", getHeading());
//         SmartDashboard.putNumber("Pigeon yaw", pigeon.getYaw().getValueAsDouble());
//         SmartDashboard.putNumber("Pigeon pitch", pigeon.getPitch().getValueAsDouble());
//         SmartDashboard.putNumber("Pigeon roll", pigeon.getRoll().getValueAsDouble());
//         SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
//     }

//     public void stopModules() {
//         frontLeft.stop();
//         frontRight.stop();
//         backLeft.stop();
//         backRight.stop();
//     }

//     // Wheel order: FR, FL, BR, BL
    // public SwerveModulePosition[] getModulePositions() {
    //     return new SwerveModulePosition[] {
    //         frontRight.getPosition(),
    //         frontLeft.getPosition(),
    //         backRight.getPosition(),
    //         backLeft.getPosition()
    //     };
    // }

    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     // Normalize to within robot max speed
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_METERS_PER_SEC);
        
    //     frontRight.setDesiredState(desiredStates[0], true);
    //     frontLeft.setDesiredState(desiredStates[1], true);
    //     backRight.setDesiredState(desiredStates[2], true);
    //     backLeft.setDesiredState(desiredStates[3], true);
        
    // }

    // public void driveRobotRelative(ChassisSpeeds speeds){
    //  ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    //  SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    //  SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_DRIVE_SPEED);


    // }
    
    // public static Translation2d[] getModuleTranslations() {
    //     return new Translation2d[] {
    //       new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
    //       new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
    //       new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
    //       new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    //     };
    //   }

    // private SwerveModuleState[] getModuleStates() {
    //     SwerveModuleState[] states = new SwerveModuleState[4];
    //     for (int i = 0; i < 4; i++) {
    //       states[i] = modules[i].getState();
    //     }
    //     return states;
    //   }  
// }