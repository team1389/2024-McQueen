package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;

public class IsRpmTarget extends Command{
    
    private ShooterSubsystem shooterSubsystem;
    private         LimelightVisionSubsystem limelightVisionSubsystem;


    public IsRpmTarget(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem limelightVisionSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.limelightVisionSubsystem = limelightVisionSubsystem;
    }

    @Override
    public boolean isFinished(){
        return ((Math.abs(shooterSubsystem.getTopSpeedRPM()-limelightVisionSubsystem.rpmTableForShoot())<500)&&(Math.abs(shooterSubsystem.getBottomSpeedRPM()-limelightVisionSubsystem.rpmTableForShoot())<500));
    }
}
