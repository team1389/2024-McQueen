package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;

public class AutoSetWristCmd extends Command{
    ShooterSubsystem shooterSubsystem;
    double angle = 0.85;
    LimelightVisionSubsystem limelight;
    int timer = 0;
    public AutoSetWristCmd(ShooterSubsystem shooterSubsystem, double angle, LimelightVisionSubsystem limelight){
        this.angle = angle;
        this.shooterSubsystem = shooterSubsystem;
        this.limelight = limelight;
        SmartDashboard.putNumber("Target Angle for SetWrist", angle);
        //addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        timer++;
        // angle = limelight.calculateShooterAngle();
        // angle = SmartDashboard.getNumber("Target Angle for SetWrist", angle);
        shooterSubsystem.setWrist(angle);
    }

    @Override
    public void end(boolean interrupted){
        timer = 0;
        shooterSubsystem.stopWrist();
    }

    @Override
    public boolean isFinished(){
        return (timer>100);
        // return (timer>30 && Math.abs(angle-shooterSubsystem.getAbsWristPosition())<.0025);
    }
}