package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class SetWristCmd extends Command{
    ShooterSubsystem shooterSubsystem;
    double angle = 0.85;
    int timer = 0;
    public SetWristCmd(ShooterSubsystem shooterSubsystem, double angle){
        this.angle = angle;
        this.shooterSubsystem = shooterSubsystem;
        SmartDashboard.putNumber("Target Angle for SetWrist", angle);
        //addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        timer++;
        // angle = SmartDashboard.getNumber("Target Angle for SetWrist", angle);
        shooterSubsystem.setWrist(angle);
    }

    @Override
    public void end(boolean interrupted){
        timer = 0;
        shooterSubsystem.holdPosition();
    }

    @Override
    public boolean isFinished(){
        return (timer>50);
        // return (timer>30 && Math.abs(angle-shooterSubsystem.getAbsWristPosition())<.0025);
    }
}