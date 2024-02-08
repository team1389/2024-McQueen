package frc.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;
import frc.subsystems.LimelightVision;
import frc.util.LimelightHelpers;

public class AutoAlign extends Command{

    private final Drivetrain drivetrain;
    private final LimelightVision limeLightVision;
    double tx, ta;
    
     public AutoAlign(Drivetrain drivetrain, LimelightVision limeLightVision) {
        this.drivetrain = drivetrain;
        this.limeLightVision = limeLightVision;

        addRequirements(drivetrain, limeLightVision);
    }


    ///connect to limelight to proceed, thrid link;
//https://www.chiefdelphi.com/t/introducing-limelight-lib/425660/66
//https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-getting-in-range
//https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
    @Override
    public void execute() {        
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        var rrResults = llresults.targetingResults.targets_Retro[0];
        var tx = rrResults.tx;
        var ty = rrResults.ty;
        var robotPose = rrResults.getTargetPose_RobotSpace();
        var tz = robotPose.getZ();
        tx = LimelightHelpers.getTX("");

        
        double speed = 0.1;

        //math
        double targetAngle = 0;
        double rotAngle = tx;

       // rotAngle = tx;

        SmartDashboard.putNumber("Rotation Angle", Math.toDegrees(rotAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));

        if (Math.abs(rotAngle) < 0.5) {
            drivetrain.stopModules();
            //rotAngle*speed
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,
                    0, speed * rotAngle);
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    
}
