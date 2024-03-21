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

public class TeleopAlignShootCmd extends SequentialCommandGroup{
    public TeleopAlignShootCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, LimelightVisionSubsystem limelightVisionSubsystem){
        addCommands(
            Commands.parallel(
              new AutoSetWristCmd(shooterSubsystem, limelightVisionSubsystem.toEncoderVal(), limelightVisionSubsystem),
                    Commands.sequence(
                        new TeleopAutoShootPIDCmd(shooterSubsystem, 3750, limelightVisionSubsystem),//, //limelightVisionSubsystem.rpmTableForShoot()
                        new OverridePreShootCmd(indexerSubsystem, intakeSubsystem)
            )
            )
        );
    }

}
