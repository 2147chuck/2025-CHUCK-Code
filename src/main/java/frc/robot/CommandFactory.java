package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand; 
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public class CommandFactory {
    
    private final AlgaeSubsystem algaeSubsystem;
    private final ClimbSubsystem climbSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public CommandFactory(
            AlgaeSubsystem algaeSubsystem,
            ClimbSubsystem climbSubsystem,
            IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem

    ) {
            this.algaeSubsystem = algaeSubsystem;
            this.climbSubsystem = climbSubsystem;
            this.intakeSubsystem = intakeSubsystem;
            this.elevatorSubsystem = elevatorSubsystem;
    }

    // Commands

   public Command levelOne() {

        return Commands.parallel(
               intakeSubsystem.L1_IntakePosition(),
               elevatorSubsystem.L1_ElevatorPosition()
        );
   }

   public Command levelTwo() {

        return Commands.parallel(
               intakeSubsystem.midBranch_IntakePosition(),
               elevatorSubsystem.L2_ElevatorPosition()
        );
   }

   public Command levelThree() {

        return Commands.parallel(
               intakeSubsystem.midBranch_IntakePosition(),
               elevatorSubsystem.L3_ElevatorPosition()
        );
   }

   public Command levelFour() {

        return Commands.parallel(
               intakeSubsystem.L4_IntakePosition(),
               elevatorSubsystem.L4_ElevatorPosition()
        );
   }

   public Command humanStation() {

        return Commands.parallel(
                intakeSubsystem.HumanStation_IntakePosition(),
                elevatorSubsystem.HumanStation_ElevatorPosition()
        );
   }

   public Command coralAuto() {

        return Commands.sequence(
                new RunCommand (() -> {
                    if (elevatorSubsystem.isAtPositionSetpoint(elevatorSubsystem.getL1ElevatorPosition())) {
                        intakeSubsystem.intakeMotorSpeed(-0.4, 0.35);
                    } else {
                        intakeSubsystem.intakeMotorSpeed(-0.3, 0.3);
                    }
                }).withTimeout(.5),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed (0, 0)).withTimeout(.25)
        );
   }

   public Command algaeAuto() {

        return Commands.sequence(
                new RunCommand (() -> algaeSubsystem.algaeSpeed(1)).withTimeout(.25),
                new RunCommand (() -> algaeSubsystem.algaeSpeed(0)).withTimeout(.25)
        );
   }

}
