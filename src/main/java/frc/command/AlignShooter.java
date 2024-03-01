package frc.command;

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

        /*Angle / Distance = 32.9e^-0.0962x
         * Angle / Distance + x = 33.1e^-0.0973x
         * Angle / Distance + 2x = 32.8e^-0.0969x
         * Angle / Distance - x = 33.5e^-0.0975x
         * Angle / Distance - 2x = 33.9e^-0.0986x
         * Final EQ? (Average of Formulas) = 33.24e^-0.0973x
         * Final EQ (Solves for Distance When given ty) = x = -10.27749229ln(0.03008423y)
         */
        shooter.runShoot();
        wrist.runWristDown();
 //       addCommand(new WaitCommand(5));
        
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}
