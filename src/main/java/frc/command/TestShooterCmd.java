package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class TestShooterCmd extends Command{
    private ShooterSubsystem shooter;
    public TestShooterCmd(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute(){
        // shooter.moveWrist(setWrist());
    }
}
