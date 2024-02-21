package frc.subsystems;



import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
// import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;


public class Drivetrain extends SubsystemBase {
    private static final double MAX_DRIVE_SPEED = Units.feetToMeters(10);
    public final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FL_DRIVE_PORT,
        DriveConstants.FL_TURN_PORT,
        DriveConstants.FL_DRIVE_REVERSED,
        DriveConstants.FL_TURN_REVERSED,
        DriveConstants.FL_ABS_REVERSED,
        ModuleConstants.FL_ANGLE_OFFSET);

    public final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FR_DRIVE_PORT,
        DriveConstants.FR_TURN_PORT,
        DriveConstants.FR_DRIVE_REVERSED,
        DriveConstants.FR_TURN_REVERSED,
        DriveConstants.FR_ABS_REVERSED,
        ModuleConstants.FR_ANGLE_OFFSET);

    public final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BL_DRIVE_PORT,
        DriveConstants.BL_TURN_PORT,
        DriveConstants.BL_DRIVE_REVERSED,
        DriveConstants.BL_TURN_REVERSED,
        DriveConstants.BL_ABS_REVERSED,
        ModuleConstants.BL_ANGLE_OFFSET);

    public final SwerveModule backRight = new SwerveModule(
        DriveConstants.BR_DRIVE_PORT,
        DriveConstants.BR_TURN_PORT,
        DriveConstants.BR_DRIVE_REVERSED,
        DriveConstants.BR_TURN_REVERSED,
        DriveConstants.BR_ABS_REVERSED,
        ModuleConstants.BR_ANGLE_OFFSET);

    
    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON, "rio");

    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.driveKinematics,
            new Rotation2d(0), getModulePositions(), new Pose2d());
    
    private final Field2d m_field = new Field2d();

    public Drivetrain() {
            //PigeonConfig();
            AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements

    );    }

    // Pigeon Stuff
    public void zeroHeading() {
        pigeon.reset();
     //   pigeon.setAngleAdjustment(0);
    }

    public void PigeonConfig(){
        Pigeon2Configuration configs = new Pigeon2Configuration();
        // mount X-up
        configs.MountPose.MountPoseYaw = 0;
        configs.MountPose.MountPosePitch = 90;
        configs.MountPose.MountPoseRoll = 0;
        configs.Pigeon2Features.DisableNoMotionCalibration = false;
        configs.Pigeon2Features.DisableTemperatureCompensation = false;
        configs.Pigeon2Features.EnableCompass = false;
        pigeon.setYaw(0);
        pigeon.getConfigurator().apply(configs);
    }

    // public void setAngleAdjustment(double angle) {
    //     pigeon.setAngleAdjustment(angle);   
    // }



    // Returns degrees from -180 to 180
    public double getHeading() {
        return Math.IEEEremainder(pigeon.getAngle(), 360);
    }

    // public double getPitch(){
    //     return Math.IEEEremainder(gyro.getPitch(), 360);
    // }

    // public double getRoll(){
    //     return Math.IEEEremainder(gyro.getRoll(), 360);
    // }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Position of the robot
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d kms) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), kms);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.driveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                               frontRight.getState(),
                                                               backLeft.getState(),
                                                               backRight.getState());
    }
    // public void resetOdometry(Pose2d pose) {
    //     poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    // }

    // To stop sliding
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
    }

    // //Use with vision
    // public void updateOdometryLatency(Pose2d measuredPose, double timestamp) {
    //     Pose2d currentPose = getPose();
    //     double xDiff = Math.abs(currentPose.getX() - measuredPose.getX());
    //     double yiff = Math.abs(currentPose.getY() - measuredPose.getY());

    //     //Check that measured pose isn't more than 1m away
    //     if(Math.hypot(xDiff, yiff) <= 1) {
    //         poseEstimator.addVisionMeasurement( measuredPose, timestamp);
    //     } 
    // }

    public void updateFieldPose() {
        m_field.setRobotPose(getPose());
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getModulePositions());
        updateFieldPose();

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Pigeon yaw", pigeon.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon pitch", pigeon.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon roll", pigeon.getRoll().getValueAsDouble());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Wheel order: FR, FL, BR, BL
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Normalize to within robot max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_METERS_PER_SEC);
        
        frontRight.setDesiredState(desiredStates[0], true);
        frontLeft.setDesiredState(desiredStates[1], true);
        backRight.setDesiredState(desiredStates[2], true);
        backLeft.setDesiredState(desiredStates[3], true);
        
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
     ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
     //toSwerveModuleStates doesn't work for some reason
  //   SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_DRIVE_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    }
    // Return a command to follow given pathplannertrajectory
    
    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //     // m_field.getObject("traj").setTrajectory(traj);
    //      // Define PID controllers for tracking trajectory
    //      PIDController xController = new PIDController(AutoConstants.P_AUTO_X, AutoConstants.I_AUTO_X, 0);
    //      PIDController yController = new PIDController(AutoConstants.P_AUTO_Y, AutoConstants.I_AUTO_Y, 0);
    //      PIDController thetaController = new PIDController(AutoConstants.P_AUTO_THETA, 0, 0);
    //      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //         // Reset odometry for the first path you run during auto
    //         if(isFirstPath){
    //             Pose2d currentPose = traj.getInitialHolonomicPose();

    //             // Mirror over y axis and reverse rotation if red (path mirroring done below)
    //             if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    //                 currentPose = new Pose2d(new Translation2d(
    //                     currentPose.getX(),
    //                     8-currentPose.getY()), // fieldwidth - y
    //                     new Rotation2d().minus(currentPose.getRotation())); // Flip rotation
    //             }
    //             this.resetOdometry(currentPose);
    //         }
    //         }),
    //         new PPSwerveControllerCommand(
    //             traj, 
    //             this::getPose, // Pose supplier
    //             DriveConstants.driveKinematics, 
    //             xController, // X controller
    //             yController, // Y controller
    //             thetaController, // Rotation controller
    //             this::setModuleStates, // Module states consumer
    //             true,
    //             this
    //         )
    //     );
    // }


}