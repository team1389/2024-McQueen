package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class RunOuttakeCmd extends Command{
    private IntakeSubsystem intakeSubsystem;
    // private double timer = 0;
    public RunOuttakeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;

        // addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.runOuttake();
        // timer += 1;
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return (!intakeSubsystem.hitSensor());
    }
}
