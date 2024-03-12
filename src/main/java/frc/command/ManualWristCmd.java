package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class ManualWristCmd extends Command{
    ShooterSubsystem shooter;
    private final Supplier<Double> power;

    public ManualWristCmd(ShooterSubsystem shooter, Supplier<Double> power) {
        this.shooter = shooter;
        this.power = power;
        shooter.controllerInterrupt = true;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.moveWrist(MathUtil.clamp(power.get(), -0.2, 0.2));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.holdPosition();
    }
}