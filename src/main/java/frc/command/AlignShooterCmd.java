package frc.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;
import frc.util.LimelightHelpers;

public class AlignShooterCmd extends Command{
     private ShooterSubsystem shooter;
    public AlignShooterCmd(ShooterSubsystem shooter){
        this.shooter = shooter;
        SmartDashboard.putNumber("Robot_Space X", 0);
        SmartDashboard.putNumber("Field_Space X", 0);
        SmartDashboard.putNumber("Target_Space X", 0);
        SmartDashboard.putBoolean("Is Align", false);
        // addRequirements(shooter);
    }

    @Override
    public void execute(){
        var tx = LimelightHelpers.getTX("");
        //needs to be fixed. Plug into limelight try and find the best way to find dist through robotPose type
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        var rrResults = llresults.targetingResults.targets_Retro[0];
        tx = rrResults.tx;
        var ty = rrResults.ty;
        var robotPose = rrResults.getTargetPose_RobotSpace();
        var robotPose3d = rrResults.getRobotPose_FieldSpace();
        var robotPose3d3d = rrResults.getRobotPose_TargetSpace();
        SmartDashboard.putBoolean("Is Align", true);
        SmartDashboard.putNumber("Robot_Space X", robotPose.getX());
        SmartDashboard.putNumber("Field_Space X", robotPose3d.getX());
        SmartDashboard.putNumber("Target_Space X", robotPose3d3d.getX());
        var tz = robotPose.getZ();
        
        //get distance time 

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tyDistance = table.getEntry("ty");
        double targetOffsetAngle_Vertical = tyDistance.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 
    
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; 

        // distance from the target to the floor
        double goalHeightInches = 60.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        

        //USE TX VALUE FROM MEGATAG INSTEAD TO GET DISTANCE

       // double distanceFromLimelightToGoalInches = 40;//CHANGE - get tx value from MegaTag pose
        

        //now calculate shooting angle from distance and height
        double speakerHeight = 78; //change to speakerheight in inches
        double targetAngleInRadians = Math.atan(speakerHeight/distanceFromLimelightToGoalInches);
        double targetAngleInWeirdUnits = 0.97-(((1.4833-targetAngleInRadians)*0.17)/1.396);
        shooter.setWrist(targetAngleInWeirdUnits);
        // wrist.runWristDown();
        // addCommand(new WaitCommand(5)); 
        
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}