package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class ManualWrist extends Command{
    Shooter shooter;
    private double power;

    public ManualWrist(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
        
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.controllerInterrupt = true;
        shooter.moveWrist(power);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.moveWrist(0);
    }
}
