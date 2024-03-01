package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class SetWrist extends Command{
    Shooter shooter;
    double angle = 0.85;
    public SetWrist(Shooter shooter){
        this.shooter = shooter;
        SmartDashboard.putNumber("Target Angle for SetWrist", angle);
        //addRequirements(shooter);
    }

    @Override
    public void execute(){
        angle = SmartDashboard.getNumber("Target Angle for SetWrist", angle);
        shooter.setWrist(angle);
    }

    @Override
    public void end(boolean interrupted){
        shooter.holdPosition();
    }
}