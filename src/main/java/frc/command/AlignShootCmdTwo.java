package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;
import frc.util.LimelightHelpers;

public class AlignShootCmdTwo extends SequentialCommandGroup{
    public AlignShootCmdTwo(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, LimelightVisionSubsystem limelightVisionSubsystem, double angle){
        addCommands(
            Commands.parallel(
              new AutoSetWristCmd(shooterSubsystem, angle, limelightVisionSubsystem),
                    Commands.sequence(
                        new AutoShootPIDCmd(shooterSubsystem, 3200, limelightVisionSubsystem),//, //limelightVisionSubsystem.rpmTableForShoot()
                        new PreShootCmd(indexerSubsystem, intakeSubsystem, shooterSubsystem)
            )
            )
        );
    }

}
