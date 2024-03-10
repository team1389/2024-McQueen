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
    double ty = LimelightHelpers.getTY("");
    double ta = LimelightHelpers.getTA(""); //area
    // double dist = (APRILTAGHEIGHT-LIMELIGHTHEIGHT)/ (Math.tan(ty) * Math.cos(tx));
    double dist = -10.27749229*(Math.log(0.03008423*ty));


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

    public double getDist(double ty){
        return -10.27749229*(Math.log(0.03008423*ty));
    }

    // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getModulePositions());
    // updateFieldPose();
    
    @Override
    public void periodic() {
        //read values periodically
        double x = tx1.getDouble(0.0);
        double y = ty1.getDouble(0.0);
        double area = ta1.getDouble(0.0);

        // //post to smart dashboard periodically
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

