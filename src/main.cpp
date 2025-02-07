#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller sticks(pros::E_CONTROLLER_MASTER);

pros::Motor tL(1, pros::MotorGears::green);
pros::Motor mL(2, pros::MotorGears::blue);
pros::Motor bL(-3, pros::MotorGears::blue);

pros::Motor tR(-10, pros::MotorGears::green);
pros::Motor mR(-9, pros::MotorGears::blue);
pros::Motor bR(7, pros::MotorGears::blue);

pros::MotorGroup leftMotors({1, 2, -3});
pros::MotorGroup rightMotors({-10, -9, 7});

pros::adi::DigitalOut clamp('c');

pros::Motor firstStageIntake(11, pros::MotorGears::blue);
pros::Motor secondStageIntake(16, pros::MotorGears::blue);

pros::MotorGroup intake({11, 16});

pros::MotorGroup armMotor({-6, 4});

pros::Optical optical(20);
pros::IMU inertial(19);
// ai vision 13
pros::Distance distanceSensor(12);

lemlib::Drivetrain drivetrain(&leftMotors,
							&rightMotors,
							14.55,
							lemlib::Omniwheel::NEW_275,
							400,
							8);

struct Toggle{
    int states = 2;
    int state = 0;
    bool latch = false;

    Toggle(int s_) : states(s_) {}

    void check(bool cond){
      if (cond){
        if (!latch){
          state = (state+1)%states;
          latch = true;
        }
        }else{
        latch = false;
        }
    }
};

Toggle clampLatch(2);
Toggle armLatch(3);


pros::adi::Encoder leftEncoder({14,'a', 'b'}, true);
pros::adi::Encoder rightEncoder({14,'f', 'e'}, true);
pros::adi::Encoder backEncoder({14,'c', 'd'}, true);

pros::adi::Encoder armEncoder('a', 'b');


lemlib::TrackingWheel leftWheel(&leftEncoder, lemlib::Omniwheel::NEW_275, -4.5);
lemlib::TrackingWheel rightWheel(&rightEncoder, lemlib::Omniwheel::NEW_275, 4.5);
lemlib::TrackingWheel backWheel(&backEncoder, lemlib::Omniwheel::NEW_275, 1.9);

lemlib::OdomSensors odom(&leftWheel,
						&rightWheel,
						&backWheel,
						nullptr,
						nullptr
);



lemlib::ControllerSettings lateralPID(10,
									0,
									3,
									3,
									1,
									100,
									3,
									500,
									20
);

lemlib::ControllerSettings steeringPID(2,
									0,
									10,
									3,
									1,
									100,
									3,
									500,
									0
);

lemlib::PID armPID(1,
				0,
				3,
				2	,
				false
);




lemlib::ExpoDriveCurve lateralCurve(3,
									10,
									1.019
);

lemlib::ExpoDriveCurve steeringCurve(3,
									10,
									1.019
);


lemlib::Chassis chassis(drivetrain,
						lateralPID,
						steeringPID,
						odom,
						&lateralCurve,
						&steeringCurve
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	armMotor.set_brake_mode_all(pros::MotorBrake::hold);
	intake.set_brake_mode_all(pros::MotorBrake::brake);
	armEncoder.reset();
	

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
			double x = armEncoder.get_value();
			pros::lcd::print(0, "armreading: %f", x); // x
            // print robot location to the brain screen
			
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	double desiredPos = 0;
	while(true){
		int lateral = sticks.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int steering = sticks.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(lateral, steering, false, .6);
		
		if (sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move_velocity(600);
		else if (sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move_velocity(-600);
		else intake.move_velocity(0);

		clampLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
		clamp.set_value(clampLatch.state);

		armLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
		if (armLatch.state == 0) desiredPos = 0;
		else if (armLatch.state == 1) desiredPos = 124;
		else if (armLatch.state == 2) desiredPos = 666;
		double output = armPID.update(desiredPos - armEncoder.get_value());
		armMotor.move_velocity(output);
		
		pros::delay(10);
	}
}