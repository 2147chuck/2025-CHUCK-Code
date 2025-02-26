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
import edu.wpi.first.wpilibj.Servo;

public class ClimbSubsystem extends SubsystemBase {
    
    private static ClimbSubsystem instance;

    private final TalonFX climbPivotMaster;
    private final TalonFX climbPivotFollower;
    private final TalonFX climbGrip;
    private final TalonFXConfiguration climbPivotMasterConfig;
    private final TalonFXConfiguration climbGripConfig;
    private final MotionMagicDutyCycle motionMagicControlPivot;
    private final MotionMagicDutyCycle motionMagicControlGrip;

    Servo brakeServo = new Servo(2);


    // Setpoints
    double ClimbPivot_StowedPosition = 1;
    double ClimbPivot_PickupPosition = 2;
    double GripMotor_InPosition = 1;
    double GripMotor_OutPosition = 2;

    // Servo Angle
    double brakeAngle = 135;
    double looseAngle = 90;



    public ClimbSubsystem() {
        climbPivotMaster = new TalonFX(16);
        climbPivotFollower = new TalonFX(17);
        climbPivotFollower.setControl(new Follower(climbPivotMaster.getDeviceID(),false)); 

        climbGrip = new TalonFX(18);

        //PIVOT Config...................................................
        climbPivotMasterConfig = new TalonFXConfiguration();
            climbPivotMasterConfig.Slot0.kP = 0.5;
            climbPivotMasterConfig.Slot0.kI = 0;
            climbPivotMasterConfig.Slot0.kD = 0.000;
            climbPivotMasterConfig.Slot0.kS = 0.25; // IF 0.25 = Add 0.25 V output to overcome static friction
            climbPivotMasterConfig.Slot0.kV = 0.01; // IF 0.12 = A velocity target of 1 rps results in 0.12 V output
            climbPivotMasterConfig.Slot0.kA = 0.01; // IF 0.01 = An acceleration of 1 rps/s requires 0.01 V output
 
            climbPivotMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            climbPivotMasterConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;

        //Set Motion Magic PIVOT settings
        motionMagicControlPivot = new MotionMagicDutyCycle(0);
            climbPivotMasterConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
            climbPivotMasterConfig.MotionMagic.MotionMagicAcceleration = 100;
            climbPivotMasterConfig.MotionMagic.MotionMagicJerk = 800;
        
        climbPivotMaster.getConfigurator().apply(climbPivotMasterConfig); 
        
        //GRIP Config.....................................................
        climbGripConfig = new TalonFXConfiguration();
            climbGripConfig.Slot0.kP = 1.0;
            climbGripConfig.Slot0.kI = 0;
            climbGripConfig.Slot0.kD = 0.000;
            climbGripConfig.Slot0.kS = 0.25; // IF 0.25 = Add 0.25 V output to overcome static friction
            climbGripConfig.Slot0.kV = 0.00; // IF 0.12 = A velocity target of 1 rps results in 0.12 V output
            climbGripConfig.Slot0.kA = 0.01; // IF 0.01 = An acceleration of 1 rps/s requires 0.01 V output

            climbGripConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            climbGripConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;


        //Set Motion Magic GRIP settings
        motionMagicControlGrip = new MotionMagicDutyCycle(0);
            climbGripConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
            climbGripConfig.MotionMagic.MotionMagicAcceleration = 100;
            climbGripConfig.MotionMagic.MotionMagicJerk = 1600;
        
        climbGrip.getConfigurator().apply(climbGripConfig); 
    }

     public static synchronized ClimbSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimbSubsystem();
        }
        return instance;
    }

    public void setPositionPivot(double position) {
        climbPivotMaster.setControl(motionMagicControlPivot.withPosition(position));
    }

    public double getPositionPivot() {
        return climbPivotMaster.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpointPivot(double position) {
        return Math.abs(climbPivotMaster.getPosition().getValueAsDouble() - position) < 0.2;
    }

    public void setPositionGrip(double position) {
        climbGrip.setControl(motionMagicControlGrip.withPosition(position));
    }

    public double getPositionGrip() {
        return climbGrip.getPosition().getValueAsDouble();
    }

    public boolean isAtPositionSetpointGrip(double position) {
        return Math.abs(climbGrip.getPosition().getValueAsDouble() - position) < 0.2;
    }
    
    
    
    // Commands
    public Command rotateToPositionPivot(double position) {
        return this.run(() -> setPositionPivot(position))
        .until(() -> isAtPositionSetpointPivot(position));
    }

    public Command rotateToPositionGrip(double position) {
        return this.run(() -> setPositionGrip(position))
        .until(() -> isAtPositionSetpointGrip(position));
    }

    public Command ClimbPivot_StowedPosition() {
        return this.run(() -> setPositionPivot(ClimbPivot_StowedPosition))
        .until(() -> isAtPositionSetpointPivot(ClimbPivot_StowedPosition));
    }

    public Command ClimbPivot_PickupPosition() {
        return this.run(() -> setPositionPivot(ClimbPivot_PickupPosition))
        .until(() -> isAtPositionSetpointPivot(ClimbPivot_PickupPosition));
    }

    public Command GripMotor_InPosition() {
        return this.run(() -> setPositionGrip(GripMotor_InPosition))
        .until(() -> isAtPositionSetpointGrip(GripMotor_InPosition));
    }

    public Command GripMotor_OutPosition() {
        return this.run(() -> setPositionGrip(GripMotor_OutPosition))
        .until(() -> isAtPositionSetpointGrip(GripMotor_OutPosition));
    } 

    public void climbSpeed(double speed)   {
        climbPivotMaster.setControl(new DutyCycleOut(speed));
        //elevatorFollower.setControl(new DutyCycleOut(speed));
    }

    public void ServoBrake() {
        brakeServo.setAngle(brakeAngle);
    }

    public void ServoLoose() {
        brakeServo.setAngle(looseAngle);
    }

    public void resetEncoder() {
        climbPivotMaster.setPosition(0);
    }


    @Override
    public void periodic() {
         // This method will be called once per scheduler run
    }
}
