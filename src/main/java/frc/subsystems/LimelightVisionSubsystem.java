package frc.subsystems;

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
    
    double tx = LimelightHelpers.getTX("");
    double ty = LimelightHelpers.getTY("") * (Math.PI/180);
    double ta = LimelightHelpers.getTA(""); //area
    // double dist = (APRILTAGHEIGHT-LIMELIGHTHEIGHT)/ (Math.tan(ty) * Math.cos(tx));
    double dist = -10.27749229*(Math.log(0.03008423*ty));

    private double aprilTagHeight = RobotMap.ShooterConstants.AprilTagHeight; //h2
    private double limelightHeight = RobotMap.ShooterConstants.LimelightHeight; //h1
    private double tagToSpeakerHeight = RobotMap.ShooterConstants.TagToSpeakerHeight; //s
    private double limelightAngle = RobotMap.ShooterConstants.LimelightAngle; //a1

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
        angle = Math.atan((tagToSpeakerHeight+aprilTagHeight-limelightHeight)/getXDistance());
        return angle;
    }

    public double getDist(double ty){
        return -10.27749229*(Math.log(0.03008423*ty));
    }

    // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getModulePositions());
    // updateFieldPose();

    public double rpmTableForShoot(){
        double distance = getXDistance();
        if(distance < 5){
            rpm = 3000;
        } else if(distance < 10){
            rpm = 4000;
        } else if(distance <= 16){
            rpm = 6000;
        }
        return rpm;

    }
    
    @Override
    public void periodic() {
        //read values periodically
        double x = tx1.getDouble(0.0);
        double y = ty1.getDouble(0.0);
        double area = ta1.getDouble(0.0);

        ty = LimelightHelpers.getTY("") * (Math.PI/180);

        // //post to smart dashboard periodically
        SmartDashboard.putNumber("RPM for vision", rpm);
        SmartDashboard.putNumber("X Distance from AprilTag", distance);
        SmartDashboard.putNumber("Angle to shoot (rad)", angle);

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

