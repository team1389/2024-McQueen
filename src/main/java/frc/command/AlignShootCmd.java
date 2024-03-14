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

public class AlignShootCmd extends SequentialCommandGroup{
//     public ShooterSubsystem shooter;
//     public LimelightVisionSubsystem limelightVisionSubsystem;
//    // private double ty;
//     private double xDistance; // x distance from robot to speaker
//     private double offset;
    // private double aprilTagHeight = RobotMap.ShooterConstants.AprilTagHeight; //h2
    // private double limelightHeight = RobotMap.ShooterConstants.LimelightHeight; //h1
    // private double tagToSpeakerHeight = RobotMap.ShooterConstants.TagToSpeakerHeight; //s
    // private double limelightAngle = RobotMap.ShooterConstants.LimelightAngle; //a1

    // public AlignShoot(ShooterSubsystem shooter, LimelightVisionSubsystem limelightVisionSubsystem){
    //     this.shooter = shooter;
    //     this.limelightVisionSubsystem = limelightVisionSubsystem;
    //    // ty = LimelightHelpers.getTY("");
    //     xDistance = 5; //use distance to calcute rpm readings
    // }

    // @Override
    // public void execute(){
    //     shooter.setWrist(limelightVisionSubsystem.getAngleToShoot());
    //     shooter.runShoot(limelightVisionSubsystem.rpmTableForShoot());
    //     // ty = LimelightHelpers.getTY("");
    //     // xDistance = (aprilTagHeight - limelightHeight) / (Math.tan(ty + limelightAngle));
    // }

    public AlignShootCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, LimelightVisionSubsystem limelightVisionSubsystem){
        addCommands(
            Commands.parallel(
              new SetWristCmd(shooterSubsystem, limelightVisionSubsystem.calculateShooterAngle(), limelightVisionSubsystem),
                    Commands.sequence(
                        new AutoShootPIDCmd(shooterSubsystem, limelightVisionSubsystem.rpmTableForShoot() ,limelightVisionSubsystem),//, //limelightVisionSubsystem.rpmTableForShoot()
                        new PreShootCmd(indexerSubsystem,intakeSubsystem, shooterSubsystem)
            )
            )
        );
    }

}
