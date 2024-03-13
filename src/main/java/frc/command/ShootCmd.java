package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;


public class ShootCmd extends SequentialCommandGroup{

    public ShootCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, LimelightVisionSubsystem limelightVisionSubsystem){
        addCommands(
            Commands.parallel(
              new SetWristCmd(shooterSubsystem, .9, limelightVisionSubsystem),
                    Commands.sequence(
                        new AutoShootPIDCmd(shooterSubsystem, 3000),
                        new PreShootCmd(indexerSubsystem,intakeSubsystem, shooterSubsystem)
            )    
            )
        );
    }
}