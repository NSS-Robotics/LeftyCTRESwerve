package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static class Swerve {
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    public static class Indexer {
        public static final int coralHolderLaserCANID = 52;
        public static final double velocity = -0.45;
        public static final double halfIntakeVelocity = 0.2;
    }

    public static class Intake {
        public static final double intakeKP = 1.9006E-31;
        public static final double intakeKI = 0;
        public static final double intakeKD = 0;
        public static final double intakeKS = -0.0098585;
        public static final double intakeKV = 1.0013;
        public static final double intakeKA = 0.0092053;
        public static final double velocity = 540;
        public static final double stationVelocity = 50;
        public static final double halfIntakeVelocity = 3000;
        public static final double l1velocity = 1000;


        public static final double upKP = 0.16413;
        public static final double upKI = 0;
        public static final double upKD = 0;
        public static final double downKP = 8.25;
        public static final double downKI = 0;
        public static final double downKD = 0;
        public static final double kG = 0;
        public static final double pivotMaxRotations = 1.3;
        public static final double intakeStartPos = 0.0927734375;
        public static final double algaePosition = 0;
        public static final double upPosition = 0.438;
        public static final double algaeIntake = -0.142822265625;
        public static final double ationPositionIntake = 0.365478515625;
        public static final double ationPositionOutake = 0.33349609375;
    }

    public static class ElevatorConstants {

        public static final int motorID = 30;
        public static final int followerMotorID = 31;
        public static final int encoder = 33;

        public static final double magnetSensorOffset = 0.599609375;
        public static final double currentLimit = 40;
        // l4
        public static final double upL4KP = 63;
        public static final double upL4KI = 0;
        public static final double upL4KD = 0;
        //l3,2,1
    
        public static final double upKP = 28.5014116;
        public static final double upKI = 0;
        public static final double upKD = 0.0004;
        public static final double upKS = 0.0042197;
        public static final double upKV = 0.98439;
        public static final double upKA = 0.015495;

        public static final double downKP = 4.0014116;
        public static final double downKI = 0;
        public static final double downKD = 0;
        public static final double downKS = 0.0042197;
        public static final double downKV = 0.98439;
        public static final double downKA = 0.015495;

        public static final double[] pos = {
            // ELEVATOR
            -0.39, // start  
            0, // coralIntake
            3.8212890625, // l4
            0.426513671875, // l3
            -1.95595703125, // l2
            0, // l1
            3.574267578125, // algaeReefHigh
            1.67841796875, // algaeReefLow
            4.173515625, // barge
            0, // processor
            -1.83837890625, // algaeGround
            0 // climb
        };

        public static final double maxRotations = 3.71484375;
        public static final double minRotationsWithArmIn = -0.27919921875;
        public static final double minRotationsWithArmScoring = -0.7934326171875;
    }

    public static class ArmPivotConstants {
        public static final int motorID = 32;
        public static final int encoderID = 34;
        // PID and feedforward
        public static final double upKP = 8.88268699; // 25 // 23 // 13
        public static final double upKI = 0;
        public static final double upKD = 0.18; // 0.001
        public static final double downKP = 2.58068699;
        public static final double downKI = 0;
        public static final double downKD = 0.1;
        public static final double kS = -0.059677;
        public static final double kV = 1.0622;
        public static final double kA = 0.025171;
        public static final double kG = 4;

        public static final double pos[] = {
            // ARM PIVOT
            0.461376953125, // start
            0, // coralIntake
            -1.38427734375, // l4
            -1.9296875, // l3
            -1.9296875, // l2
            0, // l1
            -0.894287109375, // algaeReefHigh
            -0.894287109375, // algaeReefLow
            -1.949951171875, // barge   // barge backward -2.963134765625
            0, // processor
            -0.48037109375, // algaeGround
            0 // climb
        
        };

        public static final double minRotations = -2.19384765625;
        public static final double horizontal = -0.894287109375;
        public static final double maxRotations = 0.546630859375;        
    }

    public static class Vision {
        public static final double maxAmbiguity = 0.7;
    }

    public static class CanIDs {
        public static final int elevatorMotorID = 30;
        public static final int elevatorFollowerMotorID = 31;
        public static final int elevatorEncoderID = 33;

        public static final int armPivotMotorID = 32;
        public static final int armPivotEncoderID = 34;

        public static final int intakePivotMotorID = 40;
        public static final int intakeMotorID = 41;
        public static final int intakePivotEncoderID = 42;

        public static final int indexerMotorID = 50;
        public static final int indexerLaserCANID = 51;
    }
}
