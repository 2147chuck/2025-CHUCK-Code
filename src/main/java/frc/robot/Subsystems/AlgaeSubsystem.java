package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeSubsystem extends SubsystemBase {

    private static AlgaeSubsystem instance;


    private final TalonFX algaeIntake;
    private final TalonFX algaePivot;
    private final TalonFXConfiguration algaePivotConfig;
    private final MotionMagicDutyCycle motionMagicControl;

    // Position Setpoints........................................
    double AlgaePivot_StowedPosition= 1;
    double AlgaePivot_GroundPosition=2;
    double AlgaePivot_ScrubberPosition= 3;


    public AlgaeSubsystem() {

        algaeIntake = new TalonFX(13);
        algaePivot = new TalonFX(12);

        algaePivotConfig = new TalonFXConfiguration();
            algaePivotConfig.Slot0.kP = 1.0;
            algaePivotConfig.Slot0.kI = 0;
            algaePivotConfig.Slot0.kD = 0.000;
            algaePivotConfig.Slot0.kS = 0.25; // IF 0.25 = Add 0.25 V output to overcome static friction
            algaePivotConfig.Slot0.kV = 0.00; // IF 0.12 = A velocity target of 1 rps results in 0.12 V output
            algaePivotConfig.Slot0.kA = 0.01; // IF 0.01 = An acceleration of 1 rps/s requires 0.01 V output

                        
            algaePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            algaePivotConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;

        // Set Motion Magic settings
        motionMagicControl = new MotionMagicDutyCycle(0);
            algaePivotConfig.MotionMagic.MotionMagicCruiseVelocity = 200; // rps
            algaePivotConfig.MotionMagic.MotionMagicAcceleration = 100; // rps
            algaePivotConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk rps/s/s (0.1 seconds)

        algaePivot.getConfigurator().apply(algaePivotConfig);


        TalonFXConfiguration brakeConfigs = new TalonFXConfiguration();
            brakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            algaeIntake.getConfigurator().apply(brakeConfigs);
    }

    public static synchronized AlgaeSubsystem getInstance() {
        if (instance == null) {
            instance = new AlgaeSubsystem();
        }
        return instance;
    }

    public void setPosition(double position) {
        algaePivot.setControl(motionMagicControl.withPosition(position));
    }

    public double getPosition() {
        return algaePivot.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(algaePivot.getPosition().getValueAsDouble() - position) < 0.2; // 0.2 pivot error
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }

    public Command AlgaePivot_StowedPosition() {
        return this.run(() -> setPosition(AlgaePivot_StowedPosition))
                .until(() -> isAtPositionSetpoint(AlgaePivot_StowedPosition));
    }

    public Command AlgaePivot_GroundPosition() {
        return this.run(() -> setPosition(AlgaePivot_GroundPosition))
                .until(() -> isAtPositionSetpoint(AlgaePivot_GroundPosition));
    }

    public Command AlgaePivot_ScrubberPosition() {
        return this.run(() -> setPosition(AlgaePivot_ScrubberPosition))
                .until(() -> isAtPositionSetpoint(AlgaePivot_ScrubberPosition));
    }

    public void resetAlgaePivotEncoder() {
        algaePivot.setPosition(0);
    }

    public void algaePivotSpeed(double speed) {
        algaePivot.setControl(new DutyCycleOut(speed));
    }

    public void algaeSpeed(double speed) {
        algaeIntake.setControl(new DutyCycleOut(speed));
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
