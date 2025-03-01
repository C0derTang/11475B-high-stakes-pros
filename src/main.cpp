#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

lv_obj_t* image;
LV_IMG_DECLARE(robotics);


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



lemlib::ControllerSettings lateralPID(25,
									.5,
									70,
									3,
									1,
									100,
									3,
									500,
									0
);

lemlib::ControllerSettings steeringPID(6,
									0,
									34,
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
 int targetType = 1;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	armMotor.set_brake_mode_all(pros::MotorBrake::hold);
	intake.set_brake_mode_all(pros::MotorBrake::brake);
	armEncoder.reset();

	image = lv_img_create(lv_scr_act());
	lv_img_set_src(image, &robotics);
	lv_obj_set_size(image, 480, 240);
	lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
	
    // print position to brain screen
	/*
    pros::Task screen_task([&]() {
        while (true) {
			double x = optical.get_hue();
			pros::lcd::print(0, "Hue: %f", x); // x
            // print robot location to the brain screen
			
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });*/
	
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
void redRingRush(){
	chassis.setPose(8.5,-12,180);
	chassis.moveToPose(18, 28, 222, 1350, {.forwards=false,  .horizontalDrift=8, .lead=.3,}, false);
	doinker.set_value(true);
	pros::delay(300);
	chassis.moveToPose(12, 12, 180, 4000, {.horizontalDrift=8, .lead=.3,});
	pros::delay(700);
	doinker.set_value(false);
	chassis.moveToPoint(-12, 36, 1300,{.maxSpeed=80}, false);
	clamp.set_value(true);
	pros::delay(300);
	chassis.moveToPoint(12, 12, 1400,{.forwards=false}, false);
	intake.move_velocity(500);
	pros::delay(4000);
	chassis.moveToPose(30, 32, 180, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3,}, false);
	pros::delay(10000);
	intake.move_velocity(0);
	
	pros::delay(15000);
}

void blueRingRush(){
	chassis.setPose(-8.5,-12,180);
	chassis.moveToPose(-15, 32, 170, 1500, {.forwards=false,   .horizontalDrift=8, .lead=.3, .maxSpeed=110,}, false);
	doinker.set_value(true);
	pros::delay(450);
	chassis.moveToPose(-12, 12, 180, 4000, {.horizontalDrift=8, .lead=.3,});
	pros::delay(700);
	doinker.set_value(false);
	chassis.moveToPoint(15, 39, 1300,{.maxSpeed=80}, false);
	clamp.set_value(true);
	pros::delay(300);
	chassis.moveToPoint(-12, 12, 1400,{.forwards=false}, false);
	pros::delay(500);
	intake.move_velocity(500);
	pros::delay(1000);
	chassis.moveToPose(-30, 28, 180, 2000, {.forwards=false, .horizontalDrift=8, .lead=.3,}, false);
	pros::delay(2000);
	chassis.turnToHeading(0, 1400);
	intake.move_velocity(0);
	
	pros::delay(15000);
}

void skills(){
	int desiredPos = 0;
	pros::Task armControl([&]() {
        while (true) {
			double output = armPID.update(desiredPos - armEncoder.get_value());
			armMotor.move_velocity(output);	
            pros::delay(20);
        }
    });

	

	chassis.setPose(-1,-13,180);
	//preload
	intake.move_velocity(600);
	pros::delay(800);
	intake.move_velocity(0);
	//getfirstclamp
	chassis.moveToPose(0, 0, 180, 2000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	chassis.moveToPose(-24, 0, 270, 4000, {.horizontalDrift=8, .lead=.3, .maxSpeed=110});
	pros::delay(1200);
	clamp.set_value(true);
	pros::delay(300);
	//go for first 2 disks
	intake.move_velocity(550);
	chassis.moveToPose(-24, 24, 180, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	chassis.moveToPose(-48, 24, 135, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(500);
	//prep and get arm
	desiredPos=124;
	chassis.moveToPose(-60, 48, 90, 2000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(1000);
	intake.move_velocity(0);
	desiredPos=666;
	pros::delay(500);
	chassis.moveToPose(-48, 48, 90, 4000, {.horizontalDrift=8, .lead=.3}, false);
	desiredPos=0;
	// more rings
	intake.move_velocity(600);
	chassis.moveToPose(-48, 10, 0, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(400);
	chassis.moveToPose(-48, 0, 0, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(500);
	chassis.moveToPose(-48, 10, 0, 4000, {.horizontalDrift=8, .lead=.3}, false);
	chassis.moveToPose(-60, 0, 90, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	//push into corner
	chassis.moveToPose(-60, -12, 225, 4000, {.horizontalDrift=8, .lead=.3}, false);
	clamp.set_value(false);
	chassis.moveToPose(-48, 0, 225, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);


	//MIRROR
	chassis.moveToPose(24, 0, 90, 5000, { .horizontalDrift=8, .lead=.3, .maxSpeed=80,}, false);
	clamp.set_value(true);
	pros::delay(300);
	//go for first 2 disks
	intake.move_velocity(550);
	chassis.moveToPose(24, 24, 180, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	chassis.moveToPose(48, 24, 225, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(500);
	//prep and get arm
	desiredPos=124;
	chassis.moveToPose(60, 48, 270, 2000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(1000);
	intake.move_velocity(0);
	desiredPos=666;
	pros::delay(500);
	chassis.moveToPose(48, 48, 270, 4000, {.horizontalDrift=8, .lead=.3}, false);
	desiredPos=0;
	// more rings
	intake.move_velocity(600);
	chassis.moveToPose(48, 10, 0, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(400);
	chassis.moveToPose(48, 0, 0, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	pros::delay(500);
	chassis.moveToPose(48, 10, 0, 4000, {.horizontalDrift=8, .lead=.3}, false);
	chassis.moveToPose(60, 0, 270, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);
	//push into corner
	chassis.moveToPose(60, -12, 135, 4000, {.horizontalDrift=8, .lead=.3}, false);
	clamp.set_value(false);

	chassis.moveToPose(20, 104, 225, 5000, {.horizontalDrift=8, .lead=.3}, false);
	clamp.set_value(true);
	pros::delay(300);
	chassis.moveToPose(60, 108, 45, 4000, {.horizontalDrift=8, .lead=.3}, false);
	clamp.set_value(false);
	chassis.moveToPose(-24, 108, 270, 4000, {.horizontalDrift=8, .lead=.3}, false);
	clamp.set_value(true);
	pros::delay(300);
	chassis.moveToPose(-60, 108, 315, 4000, {.horizontalDrift=8, .lead=.3}, false);
	clamp.set_value(false);
	chassis.moveToPose(0, 0, 315, 4000, {.forwards=false, .horizontalDrift=8, .lead=.3}, false);


}

void redFarSide(){
	chassis.setPose(-8.5,-12,0);

}


void autonomous() {
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, 7, 0, 2000);
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
    pros::Task intakeFilter([&]() {
		optical.set_led_pwm(100);
        while (true) {
			double value = optical.get_hue();
			// 0 is none, 1 is red, 2 is blue;
			int ringType = 0;
			if (value >= 10 && value <= 30) ringType = 1;
			else if (value >= 80) ringType = 2;
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
	ringType.state = 1;
	while(true){
		int lateral = sticks.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int steering = sticks.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		//blue is 180 220
		//red is 10 40

		chassis.curvature(lateral, steering);
		
		clampLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
		clamp.set_value(clampLatch.state);

		doinkerLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_UP));
		doinker.set_value(doinkerLatch.state);

		ringType.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));
		targetType = 3-(ringType.state+1);


		armLatch.check(sticks.get_digital(pros::E_CONTROLLER_DIGITAL_Y));
		if (armLatch.state == 0) desiredPos = 0;
		else if (armLatch.state == 1) desiredPos = 124;
		else if (armLatch.state == 2) desiredPos = 666;
		double output = armPID.update(desiredPos - armEncoder.get_value());
		armMotor.move_velocity(output);
		
		pros::delay(10);
	}
}