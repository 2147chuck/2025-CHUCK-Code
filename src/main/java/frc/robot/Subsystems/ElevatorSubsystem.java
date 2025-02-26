package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends SubsystemBase {
    
    private static ElevatorSubsystem instance;

    private final TalonFX elevatorMaster;
    private final TalonFX elevatorFollower;
    private final TalonFXConfiguration elevatorMasterConfig;
    private final MotionMagicDutyCycle motionMagicControl;

    // Position Setpoints
    double L1_ElevatorPosition = -0.2;
    double L2_ElevatorPosition = -9.010547;
    double L3_ElevatorPosition = -23.569336;
    double L4_ElevatorPosition = -47.020508;

    double HumanStation_ElevatorPosition= -6.7;
    double Processor_ElevatorPosition = 2;
    double GroundAlgae_ElevatorPosition = 3;
    double Stow_ElevatorPosition= -0.2;

    public double getL1ElevatorPosition() {
        return L1_ElevatorPosition;
    }

    public double getHumanStationElevatorPosition() {
        return HumanStation_ElevatorPosition;
    }

    public ElevatorSubsystem() {

        elevatorMaster = new TalonFX(14);
        elevatorFollower = new TalonFX(15);
        elevatorFollower.setControl(new Follower(elevatorMaster.getDeviceID(), false));  //needed because factory default invert is NOT the same


        elevatorMasterConfig = new TalonFXConfiguration();
            elevatorMasterConfig.Slot0.kP = 0.25;
            elevatorMasterConfig.Slot0.kI = 0.0;
            elevatorMasterConfig.Slot0.kD = 0.0;
            elevatorMasterConfig.Slot0.kV = 0.05; 
            elevatorMasterConfig.Slot0.kG = 0.02;

            elevatorMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            elevatorMasterConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;

        //Set Motion Magic settings
        motionMagicControl = new MotionMagicDutyCycle(0);
            elevatorMasterConfig.MotionMagic.MotionMagicCruiseVelocity = 10000; 
            elevatorMasterConfig.MotionMagic.MotionMagicAcceleration = 2000; 
    
        elevatorMaster.getConfigurator().apply(elevatorMasterConfig);
    }

    public static synchronized ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystem();
        }
        return instance; 
    }

    public void setPosition(double position){
        elevatorMaster.setControl(motionMagicControl.withPosition(position));
    }

    public double getPosition() {
        return elevatorMaster.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(elevatorMaster.getPosition().getValueAsDouble() - position) < 0.1; //0.1 pivot error
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position))
        .until(() -> isAtPositionSetpoint(position));
    }

    public Command L1_ElevatorPosition() {
        return this.run(() -> setPosition(L1_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(L1_ElevatorPosition));
    }

    public Command L2_ElevatorPosition() {
        return this.run(() -> setPosition(L2_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(L2_ElevatorPosition));
    }

    public Command L3_ElevatorPosition() {
        return this.run(() -> setPosition(L3_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(L3_ElevatorPosition));
    }

    public Command L4_ElevatorPosition() {
        return this.run(() -> setPosition(L4_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(L4_ElevatorPosition));
    }
    
    public Command HumanStation_ElevatorPosition() {
        return this.run(() -> setPosition(HumanStation_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(HumanStation_ElevatorPosition));
    }

    public Command Processor_ElevatorPosition() {
        return this.run(() -> setPosition(Processor_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(Processor_ElevatorPosition));
    }

    public Command GroundAlgae_ElevatorPosition() {
        return this.run(() -> setPosition(GroundAlgae_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(GroundAlgae_ElevatorPosition));
    }

    public Command Stow_ElevatorPosition() {
        return this.run(() -> setPosition(Stow_ElevatorPosition))
        .until(() -> isAtPositionSetpoint(Stow_ElevatorPosition));
    }

    public void resetElevatorEncoder() {
        elevatorMaster.setPosition(0);
        elevatorFollower.setPosition(0);
    }
    

    public void elevatorSpeed(double speed)   {
        elevatorMaster.setControl(new DutyCycleOut(speed));
        //elevatorFollower.setControl(new DutyCycleOut(speed));
    }

    
    @Override
    public void periodic() {
         // This method will be called once per scheduler run
    }
}
