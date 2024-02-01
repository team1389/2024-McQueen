package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class Shoot extends Command{
     private Shooter shooter;

    public Shoot(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.runShoot();
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}
