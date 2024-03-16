package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;

public class ManualSetWrist extends Command{
    ShooterSubsystem shooterSubsystem;
    double angle = 0.95;
    LimelightVisionSubsystem limelight;
    public ManualSetWrist(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        SmartDashboard.putNumber("Target Angle for SetWrist", angle);
        //addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        // angle = limelight.calculateShooterAngle();
        // angle = SmartDashboard.getNumber("Target Angle for SetWrist", angle);
        shooterSubsystem.setWrist(angle);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopWrist();
    }
}