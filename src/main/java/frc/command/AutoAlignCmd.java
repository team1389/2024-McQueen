package frc.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.util.LimelightHelpers;

public class AutoAlignCmd extends Command{

    private final DriveSubsystem drivetrainSubsystem;
    private final LimelightVisionSubsystem limelightVisionSubsystem;
    double alignTx;
        
     public AutoAlignCmd(DriveSubsystem drivetrainSubsystem, LimelightVisionSubsystem limeLightVisionSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.limelightVisionSubsystem = limeLightVisionSubsystem;
    }


    ///connect to limelight to proceed, thrid link;
//https://www.chiefdelphi.com/t/introducing-limelight-lib/425660/66
//https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-getting-in-range
//https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
    @Override
    public void execute() {        
        alignTx = LimelightHelpers.getTX("");
        // //needs to be fixed. Plug into limelight try and find the best way to find dist through robotPose type
        // LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        // var rrResults = llresults.targetingResults.targets_Retro[0];
        // var tx = rrResults.tx;
        // var ty = rrResults.ty;
        // var robotPose = rrResults.getTargetPose_RobotSpace();
        // var tz = robotPose.getZ();        
        
        double speed = 0.1;

        //math
        double rotAngle = alignTx;

       // rotAngle = tx;

        SmartDashboard.putNumber("Rotation Angle", Math.toDegrees(rotAngle));

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, -(speed * rotAngle));
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrainSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        drivetrainSubsystem.setModuleStates(moduleStates);
    }

    // @Override
    // public boolean isFinished(){
    //     return (Math.abs(LimelightHelpers.getTX("")) < 0.5);
    // } 


}
