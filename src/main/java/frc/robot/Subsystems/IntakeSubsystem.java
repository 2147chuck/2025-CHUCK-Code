package frc.robot.Subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem instance;

    /*Changed to minion on compbot
    private final SparkFlex intakeLeft = new SparkFlex(9, MotorType.kBrushless);
    private final SparkFlex intakeRight = new SparkFlex(10, MotorType.kBrushless);
    private final SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();
    private final SparkFlexConfig intakeRightConfig = new SparkFlexConfig();
    */

  private final CANrange rangeSensor;

  private final CANdle canIdle; 
  private boolean blinkState = false;
  private int blinkCounter = 0;

  private static final Animation FireAnimation = null;
  private static final Animation ColorFlowAnimation = null;
  private static final Animation LarsonAnimation = null;
  private static final Animation RainbowAnimation = null;
  private static final Animation RgbFadAnimation = null;
  private static final Animation SingleAnimation = null;
  private static final Animation StrobeAnimation = null;
  private static final Animation TwinkleAnimation = null;
  private static final Animation TwinkleOff = null;


    private final TalonFX intakePivot;
    private final TalonFXConfiguration intakePivotConfig;
    private final MotionMagicDutyCycle motionMagicControl;
    private final TalonFXS miniIntakeLeft;
    private final TalonFXS miniIntakeRight;

    // Position Setpoints........................................
    double L1_IntakePosition = 2.357188;
    double midBranch_IntakePosition = 3.05;
    double L4_IntakePosition = 3.342285;
    double HumanStation_IntakePosition = 1.42;
    double bargePosition = 0.7;


    public IntakeSubsystem() {

        //Kraken Motors
        intakePivot = new TalonFX(11);

        //Minion Motors
        miniIntakeLeft = new TalonFXS(9);
        miniIntakeRight = new TalonFXS(10);
        
        canIdle= new CANdle(32);
        rangeSensor = new CANrange(31);

        intakePivotConfig = new TalonFXConfiguration();
            intakePivotConfig.Slot0.kP = 0.5;
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

        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
            rangeSensor.getConfigurator().apply(rangeConfig);
  

        TalonFXSConfiguration configs = new TalonFXSConfiguration();
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

            miniIntakeLeft.getConfigurator().apply(configs);
            miniIntakeRight.getConfigurator().apply(configs);

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

    public Command Barge_IntakePosition() {
        return this.run(() -> setPosition(bargePosition))
                .until(() -> isAtPositionSetpoint(bargePosition));
    }

    public void resetCoralPivotEncoder() {
        intakePivot.setPosition(0);
    }

    public double rangeDistance() {
        return rangeSensor.getDistance().getValueAsDouble();  //If think it is in meters
    } 

    
    public void intakeMotorSpeed(double leftSpeed, double rightSpeed) {  
        miniIntakeLeft.set(leftSpeed);
        miniIntakeRight.set(-leftSpeed);
    }

    
    public void intakePivotSpeed(double speed) {
        intakePivot.set(speed);
    }

    @Override
    public void periodic() {
   double currentDistance = rangeDistance();
    SmartDashboard.putNumber("CANRange Distance (m)", currentDistance); //restart dashboard if not responding
       
    System.out.println("Distance: " + currentDistance); // Debugging console output
 
        // Blink CANdle if distance is in range
        if (currentDistance < 0.1) {
          blinkCounter++;
          if (blinkCounter >= 10) { // Blink every 10 cycles (~0.2 seconds at 50Hz)
            blinkState = !blinkState;
            canIdle.setLEDs(blinkState ? 0 : 0, 255, 0); // Red LED blink
            System.out.println("Blinking: " + blinkState); // Debugging blink state
            blinkCounter = 0;

          }
      } else {
          canIdle.animate(FireAnimation); // (Larson, rainbow, twinkle, color fades, etc.)
          System.out.println("Fire: "); // Debugging
      }
    } 
} 
