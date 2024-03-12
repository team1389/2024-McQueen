package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class RunIntakeCmd extends Command{
    private IntakeSubsystem intake;
    public RunIntakeCmd(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void execute(){
        intake.runIntake();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return intake.hitSensor();
    }
}