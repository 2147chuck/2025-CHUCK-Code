package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

   public Command lowerAlgae() {

        return Commands.parallel(
               intakeSubsystem.L1_IntakePosition(),
               elevatorSubsystem.GroundAlgae_ElevatorPosition()
        );
   }

   public Command upperAlgae() {

        return Commands.parallel(
               intakeSubsystem.L1_IntakePosition(),
               elevatorSubsystem.L3_ElevatorPosition()
        );
   }

   public Command humanStation() {

        return Commands.parallel(
                intakeSubsystem.HumanStation_IntakePosition(),
                elevatorSubsystem.HumanStation_ElevatorPosition()
        );
   }

   public Command bargeSetpointStart() {

        return Commands.parallel(
                elevatorSubsystem.L4_ElevatorPosition(),
                intakeSubsystem.L1_IntakePosition()
        );
   }

   public Command bargeSetpointTheSequel() {

        return Commands.parallel(
                elevatorSubsystem.L4_ElevatorPosition(),
                intakeSubsystem.Barge_IntakePosition()
        );
   }

   public Command coralAutoBranch() {

        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-0.8, 0.8)).withTimeout(.4),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(.25)
        );
   }

   public Command coralAutoTrough() {

        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(-0.6, 0.35)).withTimeout(.4),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(.25)
        );
   }

   public Command coralIntake() {
          
        return Commands.sequence(
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(1, -1)).withTimeout(3),
                new RunCommand (() -> intakeSubsystem.intakeMotorSpeed(0, 0)).withTimeout(0.25)
        );
   }

   public Command climbBrake() {

        return Commands.sequence(
                new RunCommand (() -> climbSubsystem.ServoBrake())
        );
   }

   public Command climbLoose() {

        return Commands.sequence(
                new RunCommand (() -> climbSubsystem.ServoLoose())
        );
   }
}
