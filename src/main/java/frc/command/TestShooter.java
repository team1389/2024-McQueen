package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class TestShooter extends Command{
    private Shooter shooter;
    public TestShooter(Shooter shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute(){
        // shooter.moveWrist(setWrist());
    }
}
