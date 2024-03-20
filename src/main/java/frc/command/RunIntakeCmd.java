package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.util.LimelightHelpers;

public class RunIntakeCmd extends Command{
    private IntakeSubsystem intakeSubsystem;
    private LimelightVisionSubsystem limelight;
    public RunIntakeCmd(IntakeSubsystem intakeSubsystem, LimelightVisionSubsystem limelight){
        this.intakeSubsystem = intakeSubsystem;
        this.limelight = limelight;
    }

    @Override
    public void execute(){
        intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        limelight.on();
        return intakeSubsystem.hitSensor();
    }
}
