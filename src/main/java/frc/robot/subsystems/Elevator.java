package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax motorOne;
    private final SparkMax motorTwo;
    private final SparkClosedLoopController closedLoopController;
    private final DutyCycleEncoder elevatorEncoder;

        private double L2 = 1.5; // This is only put here bc Andy has all the calculated constants (will have to be in meters)
    
        public Elevator() {
        motorOne = new SparkMax(9, MotorType.kBrushless);
        motorTwo = new SparkMax(10, MotorType.kBrushless);
        
        //how to make motorTwo follow motorOne?? does this work?
        SparkMaxConfig follow = new SparkMaxConfig();
        follow.follow(9, true); 
        motorTwo.configure(follow, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        closedLoopController = motorOne.getClosedLoopController();

        elevatorEncoder = new DutyCycleEncoder(0);

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
            .p(ElevatorConstants.ELEVATOR_KP) 
            .i(ElevatorConstants.ELEVATOR_KI) 
            .d(ElevatorConstants.ELEVATOR_KD) 
            .outputRange(-1, 1); 

        motorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.MAX_VELOCITY / ElevatorConstants.ENCODER_TO_METERS) 
            .maxAcceleration(ElevatorConstants.MAX_ACCELERATION / ElevatorConstants.ENCODER_TO_METERS) 
            .allowedClosedLoopError(0.01 / ElevatorConstants.ENCODER_TO_METERS); 

        motorOne.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
}

    public void setElevatorHeight(double targetHeight) {
        double targetPosition = metersToEncoderUnits(targetHeight);

        closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    private double metersToEncoderUnits(double meters) {
        return meters / ElevatorConstants.ENCODER_TO_METERS;
    }

    private double encoderUnitsToMeters(double encoderUnits) {
        return encoderUnits * ElevatorConstants.ENCODER_TO_METERS;
    }

    public double getElevatorHeight() {
        return encoderUnitsToMeters(elevatorEncoder.get());
    }

    public boolean isAtHeight(double targetHeight, double tolerance) {
        double currentHeight = getElevatorHeight();
        return Math.abs(currentHeight - targetHeight) < tolerance;
    }

    public Command moveToL2Command() {
        return run(() -> setElevatorHeight(L2)).until(() -> isAtHeight(L2, 0.01)); // tolerance will be tuned or whatever later
           
    }
}