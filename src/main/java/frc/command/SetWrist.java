package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class SetWrist extends Command{
    Shooter shooter;
    double angle;
    public SetWrist(Shooter shooter, double angle){
        this.shooter = shooter;
        this.angle = angle;
    }

    @Override
    public void execute(){
        shooter.setWrist(angle);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopWrist();
    }
}
