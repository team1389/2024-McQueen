package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class SetWristCmd extends Command{
    ShooterSubsystem shooter;
    double angle = 0.85;
    public SetWristCmd(ShooterSubsystem shooter, double angle){
        this.angle = angle;
        this.shooter = shooter;
        SmartDashboard.putNumber("Target Angle for SetWrist", angle);
        //addRequirements(shooter);
    }

    @Override
    public void execute(){
        // angle = SmartDashboard.getNumber("Target Angle for SetWrist", angle);
        shooter.setWrist(angle);
        SmartDashboard.putBoolean("IsFishied", Math.abs(angle - shooter.getWristPosition())<.0025);
        SmartDashboard.putNumber("Is Ria a Fish", Math.abs(angle - shooter.getWristPosition()));
    }

    @Override
    public void end(boolean interrupted){
        shooter.setWrist(angle);
    }

    @Override
    public boolean isFinished(){
        SmartDashboard.putBoolean("IsFishied", Math.abs(angle - shooter.getWristPosition())<.0025);
        SmartDashboard.putNumber("Is Ria a Fish", Math.abs(angle - shooter.getWristPosition()));
        return (Math.abs(angle - shooter.getAbsWristPosition())<.0025);
    }
}