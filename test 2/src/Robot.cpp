#include "WPILib.h"
#include <cmath>
#define PI 3.14159265

/**
 * This is a demo program showing the use of the RobotBase class.
 *The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */
class RobotDemo: public IterativeRobot
{
	Talon *FL, *RL, *FR, *RR;
	RobotDrive *myRobot; // robot drive system
	Talon *roller;
	Talon *shooter;
	Victor *upDown;
	PWM *redLED;
	PWM *greenLED;
	PWM *blueLED;
	Joystick *DriverPad;
	Joystick *ManipulatorPad;

	Gyro *fieldOrientation;
	DigitalInput *shooterSwitch;
	Encoder *shooterEncoder;

	double joyX, joyY, joyZ, joyAngle, joyMagnitude;
	double threshold;
	double temp;
	char distance[10];
	double fullShot;
	double driveSpeed;

public:
	RobotDemo()
{
		driveSpeed = 1;
		FL = new Talon(1);
		RL = new Talon(2);
		FR = new Talon(3);
		RR = new Talon(4);
		redLED = new PWM(8);
		greenLED = new PWM(9);
		blueLED = new PWM(10);

		myRobot = new RobotDrive(FL, RL, FR, RR);
		myRobot->SetExpiration(0.1);

		myRobot->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearRightMotor, true);

		roller = new Talon(5);
		roller->Set(0);
		roller->SetExpiration(10);

		DriverPad = new Joystick(1);
		ManipulatorPad = new Joystick(2);

		fieldOrientation = Gyro(1);

	}

private:

	double polarSqrt(double in, double root = 1.8) {
		if (in < 0)
			return -pow(-in, 1 / root);
		else
			return pow(in, 1 / root);
	}

	double limit(double input, double limit = 1) {
		if (input > limit)
			return limit;
		if (input < -limit)
			return -limit;
		return input;
	}

	void RobotDemo::TeleopPeriodic() {
		myRobot->SetSafetyEnabled(false);
		shooter->SetSafetyEnabled(false);

		if (DriverPad->GetRawButton(5)) {
			driveSpeed = 0.7;
		} else if (DriverPad->GetRawButton(10)) {
			driveSpeed = 0.8;
		} else {
			driveSpeed = 1;
		}

		joyX = DriverPad->GetX() * driveSpeed;
		if (fabs(joyX) < threshold * driveSpeed)
			joyX = 0;
		joyY = DriverPad->GetY() * driveSpeed;
		if (fabs(joyY) < threshold * driveSpeed)
			joyY = 0;
		joyZ = DriverPad->GetRawAxis(3) * driveSpeed;
		if (fabs(joyZ) < threshold * driveSpeed)
			joyZ = 0;

		joyX = pow(joyX, 3.0);
		joyY = pow(joyY, 3.0);
		joyZ = pow(joyZ, 3.0);
		joyMagnitude = DriverPad->GetMagnitude();
		joyAngle = DriverPad->GetDirectionDegrees();

		if (DriverPad->GetRawButton(1)) {
			fieldOrientation->Reset();
		}

		if (DriverPad->GetRawButton(2))
			myRobot->MecanumDrive_Cartesian(joyX, joyY,
					-polarSqrt(
							-cos(
									fmod(
											((fieldOrientation->GetAngle() / 360)
													* PI) + (0.5 * PI), PI))),
					fieldOrientation->GetAngle());
		else
			myRobot->MecanumDrive_Cartesian(joyX, joyY, -joyZ,
					fieldOrientation->GetAngle());
	}
};
START_ROBOT_CLASS(RobotDemo);
