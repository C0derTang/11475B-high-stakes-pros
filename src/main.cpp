#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"

pros::Controller sticks(pros::E_CONTROLLER_MASTER);

pros::Motor tL(1, pros::MotorGears::green);
pros::Motor mL(3, pros::MotorGears::blue);
pros::Motor bL(-4, pros::MotorGears::blue);

pros::Motor tR(-10, pros::MotorGears::green);
pros::Motor mR(-9, pros::MotorGears::blue);
pros::Motor bR(7, pros::MotorGears::blue);

pros::MotorGroup leftMotors({1, 3, -4});
pros::MotorGroup rightMotors({-10, -9, 7});

pros::adi::DigitalOut clamp('c');
pros::adi::DigitalOut doinker('d');

pros::Motor firstStageIntake(11, pros::MotorGears::blue);
pros::Motor secondStageIntake(16, pros::MotorGears::blue);

pros::MotorGroup intake({11, 16});

pros::MotorGroup armMotor({-6, 5});

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
Toggle doinkerLatch(2);
Toggle ringType(2);

pros::adi::Encoder leftEncoder({14,'a', 'b'}, true);
pros::adi::Encoder rightEncoder({14,'f', 'e'}, true);
pros::adi::Encoder backEncoder({14,'c', 'd'}, true);

pros::adi::Encoder armEncoder('a', 'b');


lemlib::TrackingWheel leftWheel(&leftEncoder, lemlib::Omniwheel::NEW_275, -4.5);
lemlib::TrackingWheel rightWheel(&rightEncoder, lemlib::Omniwheel::NEW_275, 4.5);
lemlib::TrackingWheel backWheel(&backEncoder, lemlib::Omniwheel::NEW_275, 1.9);

lemlib::OdomSensors odom(&leftWheel,
						nullptr,
						&backWheel,
						nullptr,
						&inertial
);



lemlib::ControllerSettings lateralPID(
	17.4,
	.1,
	60,
	3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20
);

lemlib::ControllerSettings steeringPID(
	8,
	1,
	50,
	3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20
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
 int targetType = 1;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	armMotor.set_brake_mode_all(pros::MotorBrake::hold);
	intake.set_brake_mode_all(pros::MotorBrake::brake);
	armEncoder.reset();	
    // print position to brain screen
	ringType.state = 1;

	optical.set_led_pwm(100);
    pros::Task screen_task([&]() {
        while (true) {
			double x = optical.get_hue();
			pros::lcd::print(0, "Hue: %f", x); // x
            // print robot location to the brain screen
			
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading

			if (ringType.state == 1)sticks.set_text(1, 0, "red");
			else sticks.set_text(1, 0, "blue");
            // delay to save resources
            pros::delay(50);
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
 int spintake = 0;

void blueSoloAWP(){
	chassis.setPose(0, 0, 270);
	chassis.moveToPoint(20, 0 ,2000, {.forwards=false}, false);
	chassis.moveToPoint(16, 0,  2000, {}, false);
	chassis.turnToHeading(180, 2000, {}, false);
	chassis.moveToPoint(16,-8,500,{},false);
	spintake = 1;
	pros::delay(600);
	spintake = 0;

	chassis.moveToPoint(16,2,1000,{.forwards=false},false);
	spintake = 1;
	chassis.moveToPose(-30,34, -270, 2000, {.forwards=false, .lead=-.4,}, false);
	pros::delay(300);
	spintake = 0;
	chassis.moveToPose(-8,30, 90,1500,{.lead=.3}, false);
	spintake = 1;

	//chassis.moveToPose(25, 35, 175, 2000);
	//chassis.moveToPose(12, 45.5, 290, 2000);
	//chassis.moveToPose(20, 35.5, 120, 2000);
	//chassis.moveToPose(14, 39.5, 120, 2000);
	//chassis.moveToPose(41, 6.5, 320, 2000);
	//chassis.moveToPose(28, -14, 200, 2000);
	//chassis.moveToPose(24, -39, 0, 2000);
	//chassis.moveToPose(15, -21, 150, 2000);

}
bool AUTON = false;
void autonomous() {
	AUTON = true;
	// 0 = off
	// 1 = forward
	// 2 = reverse
	pros::Task intakeFilter([&]() {
        while (AUTON) {
			double value = optical.get_hue();
			// 0 is none, 1 is red, 2 is blue;
			int ringType = 0;
			if (value > 7 && value <= 15) ringType = 1;
			else if (value >= 180) ringType = 2;
			if (ringType!= 0 && ringType != targetType){
				intake.move_voltage(12000);
				pros::delay(155);
				intake.move_voltage(-6000);
				pros::delay(100);
				continue;
			}

			if (spintake == 1) intake.move_velocity(600);
			else if (spintake == 2) intake.move_velocity(-600);
			else intake.move_velocity(0);
	
            pros::delay(5);
        }
    });

	bool clampready = true;
	pros::Task autoclamp([&]() {
        while (AUTON) {
			if (clampready && distanceSensor.get() < 50){
				pros::delay(50);
				clamp.set_value(true);
			}
			else{
				clamp.set_value(false);
			}
	
            pros::delay(5);
        }
    });


	blueSoloAWP();
	//chassis.moveToPose(0, , 0, 2000);
}

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
	AUTON = false;
	pros::Task intakeFilter([&]() {
        while (true) {
			double value = optical.get_hue();
			// 0 is none, 1 is red, 2 is blue;
			int ringType = 0;
			if (value > 7 && value <= 15) ringType = 1;
			else if (value >= 180) ringType = 2;
			if (ringType!= 0 && ringType != targetType){
				intake.move_voltage(12000);
				pros::delay(155);
				intake.move_voltage(-6000);
				pros::delay(100);
				continue;
			}

			if (sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move_velocity(600);
			else if (sticks.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move_velocity(-600);
			else intake.move_velocity(0);
	
            pros::delay(5);
        }
    });

	double desiredPos = 0;
	while(true){
		int lateral = sticks.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int steering = sticks.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		//blue is 180 220
		//red is 10 40

		chassis.arcade(lateral, steering);
		
		clampLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
		clamp.set_value(clampLatch.state);

		doinkerLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_UP));
		doinker.set_value(doinkerLatch.state);

		ringType.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));
		targetType = 3-(ringType.state+1);


		armLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_Y));
		if (armLatch.state == 0) desiredPos = -5;
		else if (armLatch.state == 1) desiredPos = 60;
		else if (armLatch.state == 2) desiredPos = 590;
		double output = armPID.update(desiredPos - armEncoder.get_value());
		armMotor.move_velocity(output);
		
		pros::delay(10);
	}
}