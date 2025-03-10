// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    final CommandXboxController driver = new CommandXboxController(0);
    final CommandXboxController operator = new CommandXboxController(1);
    final CommandXboxController test = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

     // Subsystems
     final AlgaeSubsystem algaeSubsystem;
     final ClimbSubsystem climbSubsystem;
     final ElevatorSubsystem elevatorSubsystem;
     final IntakeSubsystem intakeSubsystem;
 
     // Command Factory
     final CommandFactory commandFactory;

    // Auto Selector
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

    
      // Subystems part 2 electric boogaloo
      algaeSubsystem = new AlgaeSubsystem();
      climbSubsystem = new ClimbSubsystem();
      elevatorSubsystem = new ElevatorSubsystem();
      intakeSubsystem = new IntakeSubsystem();
      
      commandFactory = new CommandFactory(algaeSubsystem, climbSubsystem, intakeSubsystem, elevatorSubsystem);


      /* Register Named Commands */

        NamedCommands.registerCommand("coralShoot", commandFactory.coralAutoBranch());
        NamedCommands.registerCommand("coralTrough", commandFactory.coralAutoTrough());
        NamedCommands.registerCommand("levelOne", commandFactory.levelOne());
        NamedCommands.registerCommand("levelTwo", commandFactory.levelTwo());
        NamedCommands.registerCommand("levelFour", commandFactory.levelFour().withTimeout(1.5));
        NamedCommands.registerCommand("lowerAlgae", commandFactory.lowerAlgae().withTimeout(4));
        NamedCommands.registerCommand("upperAlgae", commandFactory.upperAlgae());
        NamedCommands.registerCommand("humanStation", commandFactory.humanStation());
        NamedCommands.registerCommand("bargeSetpoint", commandFactory.bargeSetpoint());
        NamedCommands.registerCommand("coralIntake", commandFactory.coralIntake());
        NamedCommands.registerCommand("constantIntake", new InstantCommand (() -> intakeSubsystem.intakeMotorSpeed(0.5, -0.5)).withTimeout(5));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(0.25));
        NamedCommands.registerCommand("algaeShoot", new InstantCommand (() -> intakeSubsystem.intakeMotorSpeed(-1, 1)).withTimeout(0.5));
  

      // Create the autoChooser
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
            Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);

        configureBindings();
        configureAutoSelector();
    }

    private void configureAutoSelector() {
        SmartDashboard.putData("Auto Selector", autoChooser);
    }
    

    private void configureBindings() {

        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    
        drivetrain.setDefaultCommand(
    
        // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        
        );

        driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.2) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.2) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    .withDeadband(MaxSpeed * 0).withRotationalDeadband(MaxAngularRate * 0) // Add a 10% deadband
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on A Button press
        driver.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

/*......................DRIVER CONTROLLER.............................. */

//Coral Outtake/Shoot

    driver.rightTrigger().whileTrue(
            new StartEndCommand(
                // When the button is held down, check if the elevator is at the L1 position
                () -> {
                    if (elevatorSubsystem.isAtPositionSetpoint(elevatorSubsystem.getL1ElevatorPosition())) {
                        intakeSubsystem.intakeMotorSpeed(-6, 0.35);
                    } else {
                        intakeSubsystem.intakeMotorSpeed(-0.8, 0.8);
                    }
                },
                // When the button is released, stop the intake motor and return elevator to HumanStation
                () -> intakeSubsystem.intakeMotorSpeed(0, 0)  
            )
        );

    driver.rightTrigger().onFalse(commandFactory.humanStation());

//Coral Intake

    // Coral Intake – Does NOT use the distance sensor yet
    driver.leftTrigger().whileTrue(new StartEndCommand(() -> intakeSubsystem.intakeMotorSpeed(0.7,-0.7),
                                                       () -> intakeSubsystem.intakeMotorSpeed(0,0)));

// ELEVATOR



// CLIMB SERVO

        driver.povDown().onTrue(new InstantCommand (() -> {climbSubsystem.ServoBrake();}));

        driver.povUp().onTrue(new InstantCommand (() -> {climbSubsystem.ServoLoose();}));


/*......................OPERATOR CONTROLLER.............................. */

//Algae Outtake/Shoot

operator.rightTrigger().whileTrue(
    new StartEndCommand(
        // When the button is held down, check if the elevator is at the L1 position
        () -> intakeSubsystem.intakeMotorSpeed(-1, 1),
        // When the button is released, stop the intake motor and return elevator to HumanStation
        () -> intakeSubsystem.intakeMotorSpeed(0, 0)  
    )
);

operator.rightTrigger().onFalse(commandFactory.humanStation());

//Algae Intake

    operator.leftTrigger().whileTrue(new SequentialCommandGroup(
        new InstantCommand(()->{
            intakeSubsystem.intakeMotorSpeed(0.5, -0.5);}),
        new WaitCommand(1),
        new InstantCommand(() -> {
            intakeSubsystem.HumanStation_IntakePosition();
        })))
        .onFalse(new SequentialCommandGroup(
        new InstantCommand(()-> {
            intakeSubsystem.intakeMotorSpeed(0.45, -0.45);
        })));

// ELEVATOR

    operator.b().onTrue(commandFactory.levelFour());

    operator.y().onTrue(commandFactory.levelThree());

    operator.x().onTrue(commandFactory.levelTwo());

    operator.a().onTrue(commandFactory.levelOne());

    operator.povRight().onTrue(commandFactory.humanStation());

    operator.leftBumper().onTrue(commandFactory.lowerAlgae());

    operator.rightBumper().onTrue(commandFactory.upperAlgae());

    //operator.leftBumper().whileTrue(new StartEndCommand(() -> elevatorSubsystem.elevatorSpeed(0.3), 
    //                                                     () -> elevatorSubsystem.elevatorSpeed(0.1)));

    //operator.rightBumper().whileTrue(new StartEndCommand(() -> elevatorSubsystem.elevatorSpeed(-0.5),
    //                                                    () -> elevatorSubsystem.elevatorSpeed(0.1)));

    operator.leftStick().onTrue(commandFactory.bargeSetpoint());

// ALGAE PIVOT

    /*algaeSubsystem.setDefaultCommand(new RunCommand(
        () -> {
            double speed = operator.getLeftY(); // Invert for correct control
            if (Math.abs(speed) < 0.1) { // Apply deadband to ignore small movements
                speed = 0;
            }
            algaeSubsystem.algaePivotSpeed(speed*0.25);
        },
        algaeSubsystem
    ));*/

    //operator.leftStick().onTrue(commandFactory.algaeSetpoint());

// CORAL PIVOT

    operator.povDown().whileTrue(new StartEndCommand(()-> intakeSubsystem.intakePivotSpeed(0.1),
                                                ()->intakeSubsystem.intakePivotSpeed(0))); 

    operator.povUp().whileTrue(new StartEndCommand(()-> intakeSubsystem.intakePivotSpeed(-0.1),
                                                ()->intakeSubsystem.intakePivotSpeed(0)));

// CLIMB

    climbSubsystem.setDefaultCommand(new RunCommand(
        () -> {
            double speed = operator.getRightY(); // Invert for correct control
            if (Math.abs(speed) < 0.2) { // Apply deadband to ignore small movements
                speed = 0;
            }  
            climbSubsystem.climbSpeed(speed);
        },
        climbSubsystem
    ));

    operator.povLeft().onTrue(climbSubsystem.ClimbPivot_PickupPosition());
    

/*......................TESTING CONTROLLER.............................. */

//ELEVATOR
 /*    test.leftBumper().whileTrue(new StartEndCommand(()-> elevatorSubsystem.elevatorSpeed(0.5),
    ()->elevatorSubsystem.elevatorSpeed(0))); 

    test.leftTrigger().whileTrue(new StartEndCommand(()-> elevatorSubsystem.elevatorSpeed(-0.5),
    ()->elevatorSubsystem.elevatorSpeed(0))); 

    //test.a().onTrue(elevatorSubsystem.rotateToPosition(-41.164551));
    test.a().onTrue(commandFactory.levelFour());

    test.b().onTrue(commandFactory.levelThree());

    test.rightBumper().onTrue(commandFactory.levelTwo());

    test.rightTrigger().onTrue(commandFactory.levelOne());

//CORAL

    test.povDown().whileTrue(
        new StartEndCommand(
            // When the button is held down, check if the elevator is at the L1 position
            () -> {
                if (elevatorSubsystem.isAtPositionSetpoint(elevatorSubsystem.getL1ElevatorPosition())) {
                    intakeSubsystem.intakeMotorSpeed(-0.2, 0.1);
                } else {
                    intakeSubsystem.intakeMotorSpeed(-0.3, 0.3);
                }
            },
            // When the button is released, stop the intake motor
            () -> intakeSubsystem.intakeMotorSpeed(0, 0)
        )
    );

    test.povUp().whileTrue(new StartEndCommand(()-> intakeSubsystem.intakeMotorSpeed(0.5, -0.5),
                                                ()->intakeSubsystem.intakeMotorSpeed(0, 0))); 

    //test.rightBumper().whileTrue(new StartEndCommand(()-> intakeSubsystem.intakePivotSpeed(0.25),
    //                                            ()->intakeSubsystem.intakePivotSpeed(0))); 

    //test.rightTrigger().whileTrue(new StartEndCommand(()-> intakeSubsystem.intakePivotSpeed(-0.25),
    //                                            ()->intakeSubsystem.intakePivotSpeed(0)));                                                
    
    //test.b().onTrue(intakeSubsystem.rotateToPosition(-0.278809));

//ALGAE
    //test.povRight().whileTrue(new StartEndCommand(()-> algaeSubsystem.algaeSpeed(0.5),
    //()->algaeSubsystem.algaeSpeed(0))); 

    //test.povLeft().whileTrue(new StartEndCommand(()-> algaeSubsystem.algaeSpeed(-0.5),
    //()->algaeSubsystem.algaeSpeed(0))); 

    //test.x().onTrue(algaeSubsystem.rotateToPosition(1));

//CLIMB
    test.y().onTrue(climbSubsystem.rotateToPositionPivot(-7));

    test.back().onTrue(climbSubsystem.rotateToPositionGrip(-2.75));

    test.povRight().whileTrue(new StartEndCommand(() -> climbSubsystem.climbSpeed(0.1), () -> climbSubsystem.climbSpeed(0)));

    test.povLeft().whileTrue(new StartEndCommand(() -> climbSubsystem.climbSpeed(-0.1), () -> climbSubsystem.climbSpeed(0)));

*/
}

public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    
}

public void setDriveMode()
{
    //drivebase.setDefaultCommand();    
}

public void setMotorBrake(boolean brake)
{
    //drivebase.setMotorBrake(brake);
}
}
