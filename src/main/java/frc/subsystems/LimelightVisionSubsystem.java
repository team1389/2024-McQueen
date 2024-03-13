package frc.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.VisionConstants;
import frc.util.LimelightHelpers;

//try using robot oritented to find distance
public class LimelightVisionSubsystem extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx1 = table.getEntry("tx");
    NetworkTableEntry ty1 = table.getEntry("ty");
    NetworkTableEntry ta1 = table.getEntry("ta");

    Pose3d botPose = LimelightHelpers.getBotPose3d("");

    double botXPose = botPose.getX();
    double botYPose = botPose.getY();

   // double botPose = li
    
    double tx = LimelightHelpers.getTX("");
    double ty = LimelightHelpers.getTY("") * (Math.PI/180);
    double ta = LimelightHelpers.getTA(""); //area
    // double dist = (APRILTAGHEIGHT-LIMELIGHTHEIGHT)/ (Math.tan(ty) * Math.cos(tx));
    double dist = -10.27749229*(Math.log(0.03008423*ty));

    private double aprilTagHeight = RobotMap.ShooterConstants.AprilTagHeight; //h2
    private double limelightHeight = RobotMap.ShooterConstants.LimelightHeight; //h1
    private double tagToSpeakerHeight = RobotMap.ShooterConstants.TagToSpeakerHeight; //s
    private double limelightAngle = RobotMap.ShooterConstants.LimelightAngle; //a1
    private double SpeakerXDistfromCenter = RobotMap.ShooterConstants.SpeakerXDistfromCenter;
    private double SpeakerYDistfromCenter = RobotMap.ShooterConstants.SpeakerYDistfromCenter;
    private double XOffset = RobotMap.ShooterConstants.XOffset;
    private double YOffset = RobotMap.ShooterConstants.YOffset;

    double rpm = 0;
    double distance = 0;
    double angle = 0;



    public LimelightVisionSubsystem(){
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        LimelightHelpers.setCropWindow("",-1,1,-1,1);
        LimelightHelpers.getTX("");
        
    }
    public void updatePose(){
            // pid loop to return yaw (and then turn the robot) (from pigeon)
            // use x,y,area and pid loop to move robot from current point to set point
            // move angle and shoot
        }

    public double getXDistance(){
        distance = (aprilTagHeight - limelightHeight) / (Math.tan(ty + limelightAngle));
        return distance;
    }

    public double getAngleToShoot(){
        return angle;
    }

    public double toEncoderVal(){
        return .98-((1.39-getAngleToShoot())/1.39)*(.98-.803);
    }

    public double getDist(double ty){
        return -10.27749229*(Math.log(0.03008423*ty));
    }

    // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getModulePositions());
    // updateFieldPose();

    public double rpmTableForShoot(){
        double distance = getDistanceSpeaker();
        if(distance < 5){
            rpm = 3000;
        } else if(distance < 10){
            rpm = 4000;
        } else if(distance <= 16){
            rpm = 6000;
        }
        return rpm;

    }

    public double shooterEquation(){
        return 9.58*Math.pow(Math.E, -0.0672*ty);
    }

    public double getXPoseSpeaker(){
        return (SpeakerXDistfromCenter-Math.abs(botXPose) + XOffset) * 3.2808399;
    }

    public double getYPoseSpeaker(){
        return (SpeakerYDistfromCenter+YOffset-botYPose) * 3.2808399;
    }
    
    public double getDistanceSpeaker(){
        return Math.sqrt(Math.pow(getXPoseSpeaker(),2) + Math.pow(getYPoseSpeaker(), 2));
    }
    

    @Override
    public void periodic() {
        angle = Math.atan((tagToSpeakerHeight+aprilTagHeight-limelightHeight)/getDistanceSpeaker());
        botPose = LimelightHelpers.getBotPose3d("");
        botXPose = botPose.getX();
        botYPose = botPose.getY();
        //read values periodically
        double x = tx1.getDouble(0.0);
        double y = ty1.getDouble(0.0);
        double area = ta1.getDouble(0.0);

        SmartDashboard.putNumber("Bot Pose X", botPose.getX());
        SmartDashboard.putNumber("Bot Pose Y", botPose.getY());
        SmartDashboard.putNumber("Bot Pose Z", botPose.getZ());
    //    SmartDashboard.putNumber("Bot Pose Rotation", botPose.getRotation());

        SmartDashboard.putNumber("X Pose Speaker", getXPoseSpeaker());
        SmartDashboard.putNumber("Y Pose Speaker", getYPoseSpeaker());
        SmartDashboard.putNumber("Dist Pose Speaker", getDistanceSpeaker());


        ty = LimelightHelpers.getTY("") * (Math.PI/180);

        // //post to smart dashboard periodically
        SmartDashboard.putNumber("RPM for vision", rpm);
        SmartDashboard.putNumber("X Distance from AprilTag", distance);
        SmartDashboard.putNumber("Angle to shoot (rad)", angle);

        SmartDashboard.putNumber("Shooter Equation Angle", shooterEquation());
        SmartDashboard.putNumber("Encoder Val", toEncoderVal());

        SmartDashboard.putNumber("LimelightX1", x);
        SmartDashboard.putNumber("LimelightY2", y);
        SmartDashboard.putNumber("LimelightArea3", area);
        SmartDashboard.putNumber("Calculated Distance From April Tag (Formula)", getDist(y));
        
        SmartDashboard.putNumber("LimelightX", LimelightHelpers.getTX(""));
        SmartDashboard.putString("InsideAutoAlign", "success");
        SmartDashboard.putNumber("LimelightY", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("LimelightArea", LimelightHelpers.getTA(""));
    }

    
}

