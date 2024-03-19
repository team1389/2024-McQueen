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

public class ShootSeq extends SequentialCommandGroup{

    public ShootSeq(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, LimelightVisionSubsystem limelightVisionSubsystem){
        addCommands(
            Commands.parallel(
                    Commands.sequence(
                        new RunShoot(shooterSubsystem),
                        new PreShootCmd(indexerSubsystem,intakeSubsystem, shooterSubsystem)
            )
            )
        );
    }

}
