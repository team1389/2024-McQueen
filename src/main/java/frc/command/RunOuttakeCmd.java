package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class RunOuttakeCmd extends Command{
    private IntakeSubsystem intake;
    public RunOuttakeCmd(IntakeSubsystem intake){
        this.intake = intake;

        // addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runOuttake();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return !intake.hitSensor();
    }
}
