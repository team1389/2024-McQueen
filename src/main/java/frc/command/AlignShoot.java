package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;
import frc.util.LimelightHelpers;

public class AlignShoot extends Command{
    public ShooterSubsystem shooter;
    public LimelightVisionSubsystem limelight;
   // private double ty;
    private double xDistance; // x distance from robot to speaker
    private double offset;
    // private double aprilTagHeight = RobotMap.ShooterConstants.AprilTagHeight; //h2
    // private double limelightHeight = RobotMap.ShooterConstants.LimelightHeight; //h1
    // private double tagToSpeakerHeight = RobotMap.ShooterConstants.TagToSpeakerHeight; //s
    // private double limelightAngle = RobotMap.ShooterConstants.LimelightAngle; //a1

    public AlignShoot(ShooterSubsystem shooter, LimelightVisionSubsystem limelight){
        this.shooter = shooter;
        this.limelight = limelight;
       // ty = LimelightHelpers.getTY("");
        xDistance = 5; //use distance to calcute rpm readings
    }

    @Override
    public void execute(){
        shooter.setWrist(limelight.getAngleToShoot());
        shooter.runShoot(limelight.rpmTableForShoot());
        // ty = LimelightHelpers.getTY("");
        // xDistance = (aprilTagHeight - limelightHeight) / (Math.tan(ty + limelightAngle));
    }

}
