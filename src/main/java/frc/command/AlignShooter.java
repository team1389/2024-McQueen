package frc.command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;
import frc.util.LimelightHelpers;

public class AlignShooter extends Command{
     private Shooter shooter;
     private Shooter wrist;

    public AlignShooter(Shooter shooter, Shooter wrist){
        this.shooter = shooter;
        this.wrist = wrist;
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

        //now calculate shooting angle from distance and height
        double speakerHeight = 0; //change to speakerheight in inches
        double targetAngleInRadians = Math.atan(speakerHeight/distanceFromLimelightToGoalInches);
        
        shooter.setWrist(targetAngleInRadians);// CHANGE to encoder angle units
        shooter.runShoot();
        wrist.runWristDown();
        // addCommand(new WaitCommand(5)); 
        
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}
