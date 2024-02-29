package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class SetPower extends Command{
    Shooter shooter;
    double power;
    public SetPower(Shooter shooter,double power){
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void execute(){
        shooter.moveWrist(power);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopWrist();
    }
}
