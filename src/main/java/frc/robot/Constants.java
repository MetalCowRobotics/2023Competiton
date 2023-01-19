// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

        //Pigeon
        public static final int DRIVETRAIN_PIGEON_ID = 14;

        //Front Left Module
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 6;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(90);

        //Front Right Module
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(91+57);

        //Back Left Module
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(268);

        //Back Right Module
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(88);

    //Drivetrain
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5;
    public static final double DRIVETRAIN_RAMP_SPEED = 0.5;
}
