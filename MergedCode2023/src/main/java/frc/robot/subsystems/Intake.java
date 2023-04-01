package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public Intake() {
        m_motor = new CANSparkMax(CAN_IDs.intakeArm, MotorType.kBrushless);
        m_motor.setInverted(IntakeConstants.kMotorInverted);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        m_motor.burnFlash();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(IntakeConstants.kPositionFactor);
        m_encoder.setVelocityConversionFactor(IntakeConstants.kVelocityFactor);
        resetEncoders();

        SmartDashboard.putData(this);
    }

    private void resetEncoders() {
        m_encoder.setPosition(0);
    }

    private void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public CommandBase stopCmd() {
        return this.runOnce(() -> setSpeed(0))
                  .withName("Intake Stop Command");
      }
    
      public boolean intakeIsNotSafe() {
        double position = m_encoder.getPosition();
        return ((position < IntakeConstants.kMinTravelInInches) ||
              (position > IntakeConstants.kMaxTravelInInches));
      }
    
      public boolean intakeIsAtPosition(double position) {
        double curPosition = m_encoder.getPosition() / 2;
        return (Math.abs(curPosition - position) < IntakeConstants.kToleranceInInches);
      }
    
      public CommandBase out() {
        return this.runOnce(() -> setSpeed(IntakeConstants.kIntakeSpeedOut))
                  .unless(() -> intakeIsNotSafe());
      }
    
      public CommandBase in() {
        return this.runOnce(() -> setSpeed(IntakeConstants.kIntakeSpeedIn))
                  .unless(() -> intakeIsNotSafe());
      }
    
      public CommandBase outToPosition(double desiredPosition) {
        return this.run(() -> setSpeed(IntakeConstants.kIntakeSpeedOut))
                  .unless(() -> (intakeIsNotSafe() ||
                          (desiredPosition <= getCurPosition()) ||
                          intakeIsAtPosition(desiredPosition)))
                  .until(() -> intakeIsAtPosition(desiredPosition))
                  .finallyDo((interrupted) -> setSpeed(0))
                  .withName("outToPosition");
      }
    
      public CommandBase inToPosition(double desiredPosition) {
        return this.run(() -> setSpeed(IntakeConstants.kIntakeSpeedIn))
                    .unless(() -> (intakeIsNotSafe() ||
                        (desiredPosition <= getCurPosition()) ||
                        intakeIsAtPosition(desiredPosition)))
                    .until(() -> intakeIsAtPosition(desiredPosition))
                    .finallyDo((interrupted) -> setSpeed(0))
                    .withName("inToPosition");
      }
    
      private double getCurPosition() {
        return m_encoder.getPosition() / 2;
      }
    
      public CommandBase setPosition(double desiredPosition) {
        return new ConditionalCommand(outToPosition(desiredPosition), 
                                      inToPosition(desiredPosition), 
                                      () -> (desiredPosition > getCurPosition()))
                    .unless(() -> Math.abs(getCurPosition() - desiredPosition) < IntakeConstants.kToleranceInInches)
                    .finallyDo((interrupted) -> setSpeed(0));
      }
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("IntakePosition", m_encoder.getPosition());
        SmartDashboard.putBoolean("Is Intake in Safe position?", !intakeIsNotSafe());
      }
}
