package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Shooter;

public class AlignShooter extends Command{
     private Shooter shooter;
     private Shooter wrist;

    public AlignShooter(Shooter shooter, Shooter wrist){
        this.shooter = shooter;
        this.wrist = wrist;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.runShoot();
        wrist.runWristDown();
 //       addCommand(new WaitCommand(5));
        
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}
