package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getPositionMeters();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);
}
