package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.command.*;
import frc.subsystems.*;
import frc.util.DPadButton;
import frc.util.DPadButton.Direction;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {


    private GenericHID driveController;
    private Trigger driveRightBumper, driveLeftBumper;
    private Trigger driveAButton;
    private Trigger driveXButton;
    private Trigger driveBButton;
    private Trigger driveYButton;
    private Trigger driveRightTrigger;
    private Trigger driveLeftTrigger;



    private XboxController manipController;
    private Trigger manipEllipsisButton;
    private Trigger manipMenuButton;
    private Trigger manipFullscreen;
    private Trigger manipStadia;
    private Trigger manipGoogle;


    private Trigger manipRightBumper;
    private Trigger manipLeftBumper;
    private Trigger manipRightTrigger;
    private Trigger manipLeftTrigger;

    private Trigger manipAButton;
    private Trigger manipBButton;
    private Trigger manipXButton;
    private Trigger manipYButton;

    private Trigger manipUp;
    private Trigger manipLeft;
    private Trigger manipDown;
    private Trigger manipRight;
    
    public final Drivetrain drivetrain = new Drivetrain();
    
    public final Indexer indexer = new Indexer();

    public final Intake intake = new Intake();
    public final Lights light = new Lights();
    public final Shooter shooter = new Shooter();
    public final Elevator elevator = new Elevator();
    public final LimelightVision limeLightVision = new LimelightVision();

    public OI() {
        
        initControllers();
        manipAButton.whileTrue(new RunIntake(intake).andThen(new InstantCommand(() -> light.setColor(0, 128, 255))));
        manipBButton.whileTrue(new RunIndexerAmp(indexer, false));
        manipFullscreen.whileTrue(new Shoot(shooter));
       // manipYButton.whileTrue(new IndexAndShoot(indexer, intake));
         manipYButton.whileTrue(new ShootToSpeaker(shooter, indexer, intake));
        // manipEllipsisButton.whileTrue(new RunIndexer(indexer, true)); // indexer to amp
        manipXButton.whileTrue(new RunElevatorUp(elevator).andThen(new RunIntake(intake)).alongWith(new RunIndexer(indexer, false)));
        manipLeftTrigger.whileTrue(new RunIntake(intake));
        manipRightTrigger.whileTrue(new RunOuttake(intake));
        manipMenuButton.whileTrue(new RunOuttake(intake));
        manipLeftBumper.whileTrue(new Shoot(shooter));
        manipRightBumper.whileTrue(new RunIntake(intake).alongWith(new RunIndexer(indexer, true)));
        manipEllipsisButton.whileTrue(new AlignShooter(shooter, shooter));
        manipGoogle.onTrue(new InstantCommand(() -> shooter.setTargetAngle(shooter.getWristPos())).alongWith(new InstantCommand(() -> shooter.holdPosition())));

        // manipStadia.whileTrue(new AutoAlign(drivetrain, limeLightVision));

        // Cool new way to make a drive command by passing in Suppliers for the
        // joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
                drivetrain,
                () -> getDriveLeftY(),
                () -> getDriveLeftX(),
                () -> getDriveRightX(),
                () -> getDriveRightY(),
                () -> getDriveLeftBumper(), // By default be in field oriented
                () -> !getDriveRightBumper(), // Slow function
                () -> driveXButton.getAsBoolean(), // Hold x position
                () -> driveRightTrigger.getAsBoolean(),
                () -> driveRightTrigger.getAsBoolean(),//auto alignment
                () -> driveController.getRawAxis(5),
                limeLightVision) // flip
        );

        shooter.setDefaultCommand(new ManualWrist(shooter, () -> getManipLeftY()));
        elevator.setDefaultCommand(new ManualElevator(elevator, () -> getManipRightY()));


        // Press A button -> zero gyro heading
        driveAButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        driveYButton.onTrue(new InstantCommand(() -> {light.isRainbowing = true;}));

        NamedCommands.registerCommand("Shoot", new Shoot(shooter));
        NamedCommands.registerCommand("IndexerToShooter", new RunIndexer(indexer, true));
        NamedCommands.registerCommand("IndexerToAmp", new RunIndexer(indexer, false));
        NamedCommands.registerCommand("RunIntake", new RunIntake(intake));
        
        // getAutonomousCommand();

        // Create a path following command using AutoBuilder. This will also trigger event markers.
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);

        manipAButton = new JoystickButton(manipController,1);
        manipBButton = new JoystickButton(manipController, 2); 
        manipXButton = new JoystickButton(manipController, 3);
        manipYButton = new JoystickButton(manipController, 4);

        manipRightBumper = new JoystickButton(manipController, 6);
        manipRightTrigger = new JoystickButton(manipController, 12);
        manipLeftTrigger = new JoystickButton(manipController, 13);
        manipLeftBumper = new JoystickButton(manipController, 5);

        manipMenuButton = new JoystickButton(manipController, 10);
        manipFullscreen = new JoystickButton(manipController, 15);
        manipGoogle = new JoystickButton(manipController, 14);
        manipEllipsisButton = new JoystickButton(manipController, 9);

        driveRightBumper = new JoystickButton(driveController, 6);
        driveRightTrigger = new JoystickButton(driveController, 12);
        driveLeftTrigger = new JoystickButton(driveController, 13);
        driveLeftBumper = new JoystickButton(driveController, 5);


        driveAButton = new JoystickButton(driveController, 1);
        driveBButton = new JoystickButton(driveController, 2); 
        driveXButton = new JoystickButton(driveController, 3); 
        driveYButton = new JoystickButton(driveController, 4); 


    }

    private double getManipLeftY() {
        return -manipController.getRawAxis(1);
    }
    
    private double getManipRightY() {
        return manipController.getRawAxis(4);
    }

    private double getDriveLeftX() {
        return -driveController.getRawAxis(0);
    }
    
    private double getDriveLeftY() {
        return -driveController.getRawAxis(1);
    }
    
    private double getDriveRightX() {
        return -driveController.getRawAxis(3); 
    }
    
    private double getDriveRightY() {
        return driveController.getRawAxis(4); 
    }
    
    private boolean getDriveLeftBumper() {
        return !driveController.getRawButton(5);
    }
    
    private boolean getDriveRightBumper() {
        return !driveController.getRawButton(6);
    }

    public Command getAutonomousCommand() {
    return new PathPlannerAuto("Top Drive Shoot");
  }


}