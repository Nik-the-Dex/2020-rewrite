package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase
{
    public TalonFX rightTalon1 = new TalonFX(frc.robot.Constants.DRIVETRAIN_RIGHT1);
    public TalonFX rightTalon2 = new TalonFX(frc.robot.Constants.DRIVETRAIN_RIGHT2);
    public TalonFX leftTalon1  = new TalonFX(frc.robot.Constants.DRIVETRAIN_LEFT1);
    public TalonFX leftTalon3  = new TalonFX(frc.robot.Constants.DRIVETRAIN_LEFT3);

    private static final DoubleSolenoid gearShifter = new DoubleSolenoid(
        frc.robot.Constants.PCM2,
        PneumaticsModuleType.CTREPCM,
        frc.robot.Constants.DRIVETRAIN_SHIFTER_FWD,
        frc.robot.Constants.DRIVETRAIN_SHIFTER_REV
    );

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public int moveState = 0;
    public int turnState = 0;

    private static final double ENC_TICKS_PER_INCH = 131113.5 / 120.0;
    private static final double TURN_KP            = 0.008;
    private static final double DISTANCE_KP        = 0.0000055;

    public int driveStraightState = 0;

    private double desiredPosition        = 0;
    private double averageEncoderPosition = 0;
    private double distanceAdjust         = 0;
    private double currentAngle           = 0;

    public int autoState   = 0;
    public int autoCounter = 0;

    public Drivetrain()
    {
        TalonFXConfiguration right1Config = new TalonFXConfiguration();
        right1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightTalon1.getConfigurator().apply(right1Config);

        rightTalon2.setControl(new Follower(rightTalon1.getDeviceID(), MotorAlignmentValue.Aligned));
        leftTalon3.setControl(new Follower(leftTalon1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void sigmaDrive(double leftSpeed, double rightSpeed)
    {
        rightTalon1.setControl(dutyCycleRequest.withOutput(rightSpeed));
        leftTalon1.setControl(dutyCycleRequest.withOutput(leftSpeed));
    }

    public void highGear(boolean gearState)
    {
        if (gearState)
        {
            gearShifter.set(Value.kReverse);
        }
        else
        {
            gearShifter.set(Value.kForward);
        }
    }

    public boolean turnAngle(double angle)
    {
        double speed = (angle - Robot.navX.angle) * 0.01;
        sigmaDrive(-speed, speed);

        if (Math.abs(angle - Robot.navX.angle) < 5)
        {
            sigmaDrive(0, 0);
            return true;
        }
        return false;
    }

    public boolean turnAngle(double angle, double scale)
    {
        double speed = (angle - Robot.navX.angle) * 0.01;
        sigmaDrive(-speed * scale, speed * scale);

        if (Math.abs(angle - Robot.navX.angle) < 5)
        {
            sigmaDrive(0, 0);
            return true;
        }
        return false;
    }

    public boolean driveStraight(double distanceInches, double speedScaleR, double speedScaleL)
    {
        switch (driveStraightState)
        {
            case 0:
                averageEncoderPosition = -rightTalon1.getPosition().getValueAsDouble();
                desiredPosition = averageEncoderPosition + (distanceInches * ENC_TICKS_PER_INCH);
                currentAngle = Robot.navX.angle;
                driveStraightState = 1;
                break;

            case 1:
                double angleAdjust = (currentAngle - Robot.navX.angle) * TURN_KP;
                distanceAdjust = (desiredPosition - (-rightTalon1.getPosition().getValueAsDouble())) * DISTANCE_KP;
                sigmaDrive(
                    (-distanceAdjust - angleAdjust) * speedScaleL,
                    (-distanceAdjust + angleAdjust) * speedScaleR
                );
                if (Math.abs(distanceAdjust) < 0.3)
                {
                    driveStraightState = 2;
                }
                break;

            case 2:
                sigmaDrive(0, 0);
                return true;

            default:
                break;
        }
        return false;
    }

    public boolean driveStraight(double distanceInches)
    {
        switch (driveStraightState)
        {
            case 0:
                averageEncoderPosition = -rightTalon1.getPosition().getValueAsDouble();
                desiredPosition = averageEncoderPosition + (distanceInches * ENC_TICKS_PER_INCH);
                currentAngle = Robot.navX.angle;
                driveStraightState = 1;
                break;

            case 1:
                double angleAdjust = (currentAngle - Robot.navX.angle) * TURN_KP;
                distanceAdjust = (desiredPosition - (-rightTalon1.getPosition().getValueAsDouble())) * DISTANCE_KP;
                sigmaDrive(-distanceAdjust - angleAdjust, -distanceAdjust + angleAdjust);
                if (Math.abs(distanceAdjust) < 0.3)
                {
                    driveStraightState = 2;
                }
                break;

            case 2:
                sigmaDrive(0, 0);
                return true;

            default:
                break;
        }
        return false;
    }

    public boolean driveToAngle(double distanceInches, double endPose)
    {
        switch (driveStraightState)
        {
            case 0:
                averageEncoderPosition = -rightTalon1.getPosition().getValueAsDouble();
                desiredPosition = averageEncoderPosition + (distanceInches * ENC_TICKS_PER_INCH);
                driveStraightState = 1;
                break;

            case 1:
                double angleAdjust = (endPose - Robot.navX.angle) * 0.028;
                distanceAdjust = (desiredPosition - (-rightTalon1.getPosition().getValueAsDouble())) * DISTANCE_KP;
                sigmaDrive(-distanceAdjust - angleAdjust, -distanceAdjust + angleAdjust);
                if (Math.abs(distanceAdjust) < 0.2 && Math.abs(endPose - Robot.navX.angle) < 5)
                {
                    driveStraightState = 2;
                }
                break;

            case 2:
                sigmaDrive(0, 0);
                return true;

            default:
                break;
        }
        return false;
    }

    public void autonomous()
    {
        switch (autoState)
        {
            case 0:
                if (driveToAngle(100, 20))
                {
                    driveStraightState = 0;
                    autoState = 1;
                }
                break;

            case 1:
                Robot.ballMech.intake(-0.9);
                if (Robot.ballMech.ballIsInRobot())
                {
                    autoState = 2;
                }
                break;

            case 2:
                Robot.ballMech.stopIntake();
                if (driveToAngle(0, 180))
                {
                    driveStraightState = 0;
                    autoState = 3;
                    autoCounter = 0;
                }
                break;

            case 3:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 150)
                {
                    driveStraightState = 0;
                    autoState = 4;
                }
                autoCounter++;
                break;

            case 4:
                if (driveToAngle(50, 0))
                {
                    driveStraightState = 0;
                    autoState = 5;
                }
                break;

            case 5:
                Robot.ballMech.intake(-0.9);
                if (driveToAngle(100, 0))
                {
                    driveStraightState = 0;
                    autoState = 6;
                }
                break;

            case 6:
                Robot.ballMech.stopIntake();
                if (driveToAngle(0, 180))
                {
                    driveStraightState = 0;
                    autoState = 7;
                    autoCounter = 0;
                }
                break;

            case 7:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 150)
                {
                    driveStraightState = 0;
                    autoState = 4;
                    autoCounter = 0;
                }
                autoCounter++;
                break;

            default:
                break;
        }
    }

    public void sixBallAuto()
    {
        switch (autoState)
        {
            case 0:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 200)
                {
                    Robot.ballMech.stopShooter();
                    Robot.ballMech.runRoller(0);
                    Robot.ballMech.counter      = 0;
                    Robot.ballMech.shooterState = 0;
                    autoState = 1;
                }
                autoCounter++;
                break;

            case 1:
                if (turnAngle(-181))
                {
                    driveStraightState = 0;
                    autoState = 2;
                }
                break;

            case 2:
                Robot.ballMech.intake(-1);
                if (driveStraight(390, 1, 1))
                {
                    driveStraightState = 0;
                    autoState = 3;
                }
                break;

            case 3:
                Robot.ballMech.stopIntake();
                if (turnAngle(0))
                {
                    driveStraightState = 0;
                    autoState = 4;
                }
                break;

            case 4:
                if (driveStraight(200, 1, 1))
                {
                    driveStraightState = 0;
                    autoState = 5;
                }
                break;

            case 5:
                Robot.ballMech.variableDistanceShooter();
                break;

            default:
                break;
        }
    }

    public void sixBallAutoTwo()
    {
        switch (autoState)
        {
            case 0:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 250)
                {
                    sigmaDrive(0, 0);
                    Robot.ballMech.stopShooter();
                    Robot.ballMech.runRoller(0);
                    Robot.ballMech.counter      = 0;
                    Robot.ballMech.shooterState = 0;
                    autoState = 1;
                }
                autoCounter++;
                break;

            case 1:
                if (turnAngle(-181))
                {
                    driveStraightState = 0;
                    autoState = 2;
                }
                break;

            case 2:
                Robot.ballMech.intakeTwo(-1);
                if (driveStraight(400))
                {
                    driveStraightState = 0;
                    autoCounter = 0;
                    autoState = 3;
                }
                break;

            case 3:
                Robot.ballMech.shooterMotor1.setControl(new VelocityVoltage(-85.9));
                Robot.ballMech.stopIntake();
                autoCounter++;
                if (autoCounter > 35)
                {
                    driveStraight(-250, 1, -1);
                }
                else
                {
                    driveStraight(-250, 1.5, 1.5);
                }
                if (Math.abs(Robot.navX.angle) < 15)
                {
                    sigmaDrive(0, 0);
                    driveStraightState = 0;
                    autoState = 5;
                }
                break;

            case 4:
                if (turnAngle(0))
                {
                    driveStraightState = 0;
                    autoState = 5;
                }
                break;

            case 5:
                if (Robot.ballMech.shooterState == 1)
                {
                    Robot.ballMech.intakeMotor.set(-1);
                    Robot.ballMech.shooterState = 2;
                }
                Robot.ballMech.variableDistanceShooter();
                break;

            default:
                break;
        }
    }

    public void eightBallAuto(double distanceInches, double endPose)
    {
        switch (autoState)
        {
            case 0:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 200)
                {
                    Robot.ballMech.stopShooter();
                    Robot.ballMech.runRoller(0);
                    Robot.ballMech.counter      = 0;
                    Robot.ballMech.shooterState = 0;
                    autoState = 1;
                }
                autoCounter++;
                break;

            case 1:
                if (turnAngle(-181))
                {
                    driveStraightState = 0;
                    autoState = 2;
                }
                break;

            case 2:
                Robot.ballMech.intakeTwo(-1);
                if (driveStraight(450))
                {
                    driveStraightState = 0;
                    autoState = 3;
                }
                break;

            case 3:
                Robot.ballMech.stopIntake();
                if (driveStraight(-180))
                {
                    driveStraightState = 0;
                    autoState = 4;
                }
                break;

            case 4:
                if (turnAngle(0))
                {
                    driveStraightState = 0;
                    autoState = 5;
                }
                break;

            case 5:
                Robot.ballMech.intakeMotor.set(-1);
                Robot.ballMech.variableDistanceShooter();
                break;

            default:
                break;
        }
    }

    public void tenBallAuto()
    {
        switch (autoState)
        {
            case 0:
                Robot.ballMech.intake(-1);
                if (driveToAngle(70, 20))
                {
                    driveStraightState = 0;
                    autoState = 1;
                }
                break;

            case 1:
                Robot.ballMech.stopIntake();
                if (turnAngle(-180))
                {
                    autoState = 2;
                    autoCounter = 0;
                }
                break;

            case 2:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 50)
                {
                    Robot.ballMech.stopShooter();
                    Robot.ballMech.runRoller(0);
                    Robot.ballMech.counter      = 0;
                    Robot.ballMech.shooterState = 0;
                    autoState = 3;
                }
                autoCounter++;
                break;

            case 3:
                if (driveToAngle(80, 0))
                {
                    driveStraightState = 0;
                    autoState = 4;
                }
                break;

            case 4:
                Robot.ballMech.intake(-1);
                if (driveStraight(100))
                {
                    driveStraightState = 0;
                    autoState = 5;
                }
                break;

            case 5:
                if (driveStraight(-100))
                {
                    driveStraightState = 0;
                    autoState = 6;
                }
                break;

            case 6:
                if (turnAngle(-180))
                {
                    autoState = 7;
                }
                break;

            case 7:
                Robot.ballMech.intakeMotor.set(-1);
                Robot.ballMech.variableDistanceShooter();
                break;

            default:
                break;
        }
    }

    public void threeBallAuto()
    {
        switch (autoState)
        {
            case 0:
                Robot.ballMech.variableDistanceShooter();
                if (autoCounter > 250)
                {
                    Robot.ballMech.stopShooter();
                    Robot.ballMech.runRoller(0);
                    Robot.ballMech.counter      = 0;
                    Robot.ballMech.shooterState = 0;
                    driveStraightState = 0;
                    autoState = 1;
                }
                autoCounter++;
                break;

            case 1:
                driveStraight(-70);
                break;

            default:
                break;
        }
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("rightenc", rightTalon1.getPosition().getValueAsDouble());
    }
}
