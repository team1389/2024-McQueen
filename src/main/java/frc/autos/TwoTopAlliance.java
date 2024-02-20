package frc.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Shooter;
public class TwoTopAlliance extends SequentialCommandGroup{
    
    public TwoTopAlliance(Drivetrain drivetrain, Intake Intake, Shooter shooter)
    {
        PathPlannerPath path = PathPlannerPath.fromPathFile("2 Top Alliance Side");
        
        

    }
}
