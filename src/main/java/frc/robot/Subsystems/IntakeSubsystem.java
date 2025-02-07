package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem instance;

    private final SparkFlex intakeLeft = new SparkFlex(9, MotorType.kBrushless);
    private final SparkFlex intakeRight = new SparkFlex(10, MotorType.kBrushless);
    private final SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();
    private final SparkFlexConfig intakeRightConfig = new SparkFlexConfig();

    private final TalonFX intakePivot;
    private final TalonFXConfiguration intakePivotConfig;
    private final MotionMagicDutyCycle motionMagicControl;

    // Position Setpoints........................................
    double L1_IntakePosition = -2.307188;
    double midBranch_IntakePosition = -2;
    double L4_IntakePosition = -1.72;
    double HumanStation_IntakePosition = -3.87;


    public IntakeSubsystem() {

        //Rev Motors
        intakeRightConfig.idleMode(IdleMode.kCoast);
        intakeRightConfig.inverted(false);
        intakeLeftConfig.idleMode(IdleMode.kCoast);
        intakeLeftConfig.inverted(true);
        
        //Kraken Motors
        intakePivot = new TalonFX(11);

        intakePivotConfig = new TalonFXConfiguration();
            intakePivotConfig.Slot0.kP = 1.0;
            intakePivotConfig.Slot0.kI = 0;
            intakePivotConfig.Slot0.kD = 0.000;
            intakePivotConfig.Slot0.kS = 0.25; // IF 0.25 = Add 0.25 V output to overcome static friction
            intakePivotConfig.Slot0.kV = 0.00; // IF 0.12 = A velocity target of 1 rps results in 0.12 V output
            intakePivotConfig.Slot0.kA = 0.01; // IF 0.01 = An acceleration of 1 rps/s requires 0.01 V output
            
            intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            intakePivotConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;
    
        // Set Motion Magic settings
        motionMagicControl = new MotionMagicDutyCycle(0);
            intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = 200; // rps
            intakePivotConfig.MotionMagic.MotionMagicAcceleration = 100; // rps
            intakePivotConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk rps/s/s (0.1 seconds)

        intakePivot.getConfigurator().apply(intakePivotConfig);
    }

    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public void setPosition(double position) {
        intakePivot.setControl(motionMagicControl.withPosition(position));
    }

    public double getPosition() {
        return intakePivot.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(intakePivot.getPosition().getValueAsDouble() - position) < 0.2; // 0.2 pivot error
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }


    public Command L1_IntakePosition() {
        return this.run(() -> setPosition(L1_IntakePosition))
                .until(() -> isAtPositionSetpoint(L1_IntakePosition));
    }

    public Command midBranch_IntakePosition() {
        return this.run(() -> setPosition(midBranch_IntakePosition))
                .until(() -> isAtPositionSetpoint(midBranch_IntakePosition));
    }

    public Command L4_IntakePosition() {
        return this.run(() -> setPosition(L4_IntakePosition))
                .until(() -> isAtPositionSetpoint(L4_IntakePosition));
    }

    public Command HumanStation_IntakePosition() {
        return this.run(() -> setPosition(HumanStation_IntakePosition))
                .until(() -> isAtPositionSetpoint(HumanStation_IntakePosition));
    }

    public void resetCoralPivotEncoder() {
        intakePivot.setPosition(0);
    }

    public void intakeMotorSpeed(double leftSpeed, double rightSpeed) {
        intakeRight.set(leftSpeed);
        intakeLeft.set(rightSpeed);
    }

    public void intakePivotSpeed(double speed) {
        intakePivot.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    } 
} 
