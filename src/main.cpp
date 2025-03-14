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


pros::Motor armMotor1(-6, pros::MotorGears::green);
pros::Motor armMotor2(5, pros::MotorGears::green);

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
pros::adi::Encoder backEncoder({14,'c', 'd'}, true);

pros::adi::Encoder armEncoder('a', 'b');


lemlib::TrackingWheel leftWheel(&leftEncoder, lemlib::Omniwheel::NEW_275, -4.5);
lemlib::TrackingWheel backWheel(&backEncoder, lemlib::Omniwheel::NEW_275, 1.9);

lemlib::OdomSensors odom(&leftWheel,
						nullptr,
						&backWheel,
						nullptr,
						&inertial
);



lemlib::ControllerSettings lateralPID(
	11,
	0,
	54,
	3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    12
);

lemlib::ControllerSettings steeringPID(
	5,
	0,
	30,
	3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20
);

lemlib::ControllerSettings mogolateralPID(
	14.2,
	0,
	50,
	3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500,
	12
);

lemlib::ControllerSettings mogosteeringPID(
	7,
	0,
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


lemlib::Chassis mogochassis(drivetrain,
	mogolateralPID,
	mogosteeringPID,
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
int autonum = 0;
bool selecting=true;

void prevauto(){
	--autonum;
}
void selectauto(){
	selecting=false;
}
void nextauto(){
	++autonum;
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	pros::lcd::register_btn0_cb(prevauto);
	pros::lcd::register_btn1_cb(selectauto);
	pros::lcd::register_btn2_cb(nextauto);
	
	//minimal auton selector
	/*
	while(selecting){
		pros::lcd::print(1, "Battery %: %d", pros::battery::get_capacity());
		pros::lcd::set_text(3, "Selected Auton:");

		autonum = (autonum+9)%9;

		switch(autonum){
			case 0:
				pros::lcd::set_text(4, "Skills");
				break;
			case 1:
				pros::lcd::set_text(4, "Red Ringside");
				break;
			case 2:
				pros::lcd::set_text(4, "Blue Ringside");
				break;
			case 3:
				pros::lcd::set_text(4, "Red Stakeside");
				break;
			case 4:
				pros::lcd::set_text(4, "Blue Stakeside");
				break;
			case 5:
				pros::lcd::set_text(4, "Red Ring Finals");
				break;
			case 6:
				pros::lcd::set_text(4, "Blue Ring Finals");
				break;
			case 7:
				pros::lcd::set_text(4, "Red Stake Finals");
				break;
			case 8:
				pros::lcd::set_text(4, "Blue Stake Finals");
				break;
		}
		pros::delay(20);
	}
	*/
	autonum=4;



    chassis.calibrate(); // calibrate sensors
	armMotor.set_brake_mode_all(pros::MotorBrake::hold);
	intake.set_brake_mode_all(pros::MotorBrake::brake);
	armEncoder.reset();	

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
 bool clampready = true;

void redRingSide(){
		//7 inches forward is edge of tile
	// 7.5 inches off left
	// alliance
	chassis.setPose(0, 0, 270);
	chassis.moveToPoint(18, 0 ,1200, {.forwards=false}, false);
	chassis.moveToPoint(13, 0,  1000, {}, false);
	chassis.turnToHeading(180, 800, {}, false);
	chassis.moveToPoint(13,-8,500,{},false);
	spintake = 1;
	pros::delay(600);
	spintake = 0;
	// grab first disc
	chassis.moveToPoint(16,2,500,{.forwards=false},false);
	spintake = 1;
	chassis.moveToPose(-32,36, 0, 2000, {.forwards=false, .lead=-.3, .maxSpeed=110}, false);
	pros::delay(130);
	spintake = 0;
	//grab mogo
	chassis.moveToPose(-2,29, 90,1600,{.lead=.3, }, false);
	spintake = 1;
	pros::delay(300);
	//middle stack disks
	mogochassis.moveToPose(-27,51.5, 180,1800,{.forwards=false, .maxSpeed=85}, false);
	mogochassis.moveToPose(-27,30, 180,1000,{.lead=.5}, false);
	// touch bar
	armLatch.state=2;
	mogochassis.moveToPose(4,40, 225,2000,{.forwards=false, .lead=.2}, false);
};
void blueRingSide(){
	chassis.setPose(0, 0, 90);
	chassis.moveToPoint(-18, 0 ,1200, {.forwards=false}, false);
	chassis.moveToPoint(-13, 0,  1000, {}, false);
	chassis.turnToHeading(180, 800, {}, false);
	chassis.moveToPoint(-13,-8,500,{},false);
	spintake = 1;
	pros::delay(600);
	spintake = 0;
	// grab first disc
	chassis.moveToPoint(-16,2,500,{.forwards=false},false);
	spintake = 1;
	chassis.moveToPose(30,36, 0, 2000, {.forwards=false, .lead=-.3, .maxSpeed=110}, false);
	pros::delay(130);
	spintake = 0;
	//grab mogo
	chassis.moveToPose(2,32, 270,1600,{.lead=.3, }, false);
	spintake = 1;
	pros::delay(300);
	//middle stack disks
	mogochassis.moveToPose(27,51.5, 180,1800,{.forwards=false, .maxSpeed=85}, false);
	mogochassis.moveToPose(27,30, 180,1000,{.lead=.5}, false);
	// touch bar
	armLatch.state=2;
	mogochassis.moveToPose(-2,38, 135,2000,{.forwards=false, .lead=.2}, false);

};
void redRingFinals(){
	//7 inches forward is edge of tile
// 7.5 inches off left
// alliance
chassis.setPose(0, 0, 270);
chassis.moveToPoint(18, 0 ,2000, {.forwards=false}, false);
chassis.moveToPoint(13, 0,  2000, {}, false);
chassis.turnToHeading(180, 2000, {}, false);
chassis.moveToPoint(13,-8,500,{},false);
spintake = 1;
pros::delay(600);
spintake = 0;
// grab 1st disk
chassis.moveToPoint(16,2,1000,{.forwards=false},false);
spintake = 1;
chassis.moveToPose(-32,36, 0, 2000, {.forwards=false, .lead=-.3,}, false);
pros::delay(100);
spintake = 0;
// clamp
chassis.moveToPose(-4,32, 90,1500,{.lead=.3}, false);
spintake = 1;
//middle stack disks
mogochassis.moveToPose(-27,51, 180,2000,{.forwards=false, .maxSpeed=80}, false);
mogochassis.moveToPose(-27,30, 180,1500,{.lead=.5}, false);
mogochassis.moveToPose(-33,48, 180,2000,{.forwards=false, .lead=.5, .maxSpeed=80}, false);
mogochassis.moveToPose(-33, 20, 180,1500,{ .lead=-.5, .maxSpeed=80}, false);
mogochassis.moveToPose(69,-2, 90,3000,{.lead=.2}, false);
};
void blueRingFinals(){
//7 inches forward is edge of tile
// 7.5 inches off left
// alliance
chassis.setPose(0, 0, 90);
	chassis.moveToPoint(-18, 0 ,2000, {.forwards=false}, false);
	chassis.moveToPoint(-13, 0,  2000, {}, false);
	chassis.turnToHeading(180, 2000, {}, false);
	chassis.moveToPoint(-13,-8,500,{},false);
	spintake = 1;
	pros::delay(600);
	spintake = 0;
	// grab first disc
	chassis.moveToPoint(-16,2,1000,{.forwards=false},false);
	spintake = 1;
	chassis.moveToPose(32,36, 0, 2000, {.forwards=false, .lead=-.3,}, false);
	pros::delay(100);
	spintake = 0;
	//grab mogo
	chassis.moveToPose(4,32, 270,1500,{.lead=.3}, false);
	spintake = 1;//middle stack disks
mogochassis.moveToPose(27,51, 180,2000,{.forwards=false, .maxSpeed=80}, false);
mogochassis.moveToPose(27,30, 180,1500,{.lead=.5}, false);
mogochassis.moveToPose(33,48, 180,2000,{.forwards=false, .lead=.5, .maxSpeed=80}, false);
mogochassis.moveToPose(33, 20, 180,1500,{ .lead=-.5, .maxSpeed=80}, false);

mogochassis.moveToPose(-69,-2, 270,3000,{.lead=.2}, false);

};


void redStakeSide(){
	// alliance

	chassis.setPose(0, 0, 180);
	chassis.moveToPose(-3, 37, 170, 1100, {.forwards=false, .lead=.2}, false);
	doinker.set_value(true);
	pros::delay(200);
	chassis.moveToPose(0, 20, 180, 500, { .lead=.2}, false);
	doinker.set_value(false);
	chassis.moveToPose(0, 10, 180, 1000, { .lead=.2}, false);

	chassis.moveToPose(-37.5, 32.5, 315, 3000, {.lead=.2, .maxSpeed=100}, false);
	spintake=1;
	mogochassis.moveToPose(-10, 33, 180, 3000, {.forwards=false, .lead=.3}, false);
}
void blueStakeSide(){
	chassis.setPose(0, 0, 180);
	chassis.moveToPose(5, 40, 220, 1300, {.forwards=false, .lead=.2}, false);
	doinker.set_value(true);
	pros::delay(300);
	//chassis.moveToPose(0, 20, 180, 500, { .lead=.2}, false);
	chassis.moveToPose(5, 10, 180, 1000, { .lead=.2}, false);
	doinker.set_value(false);


	chassis.moveToPose(37.5, 32.5, 45, 3000, {.lead=.2, .maxSpeed=100}, false);
	spintake=1;
	mogochassis.moveToPose(10, 35, 180, 3000, {.forwards=false, .lead=.3}, false);

}
void redStakeFinals(){
	chassis.setPose(0, 0, 180);
	chassis.moveToPose(-2, 48, 175, 1600, {.forwards=false, .lead=.2}, false);
	doinker.set_value(true);
	chassis.moveToPose(0, 20, 180, 2000, { .lead=.2}, false);
	doinker.set_value(false);
}
void blueStakeFinals(){}


void redSoloAWP(){
	//7 inches forward is edge of tile
	// 7.5 inches off left
	// alliance stake
	chassis.setPose(0, 0, 270);
	chassis.moveToPoint(18, 0 ,2000, {.forwards=false}, false);
	chassis.moveToPoint(13, 0,  2000, {}, false);
	chassis.turnToHeading(180, 800, {}, false);
	chassis.moveToPoint(13,-8,500,{},false);
	spintake = 1;
	pros::delay(600);
	spintake = 0;
	// grab first disc
	chassis.moveToPoint(16,2,500,{.forwards=false},false);
	spintake = 1;
	chassis.moveToPose(-32,36, 0, 2000, {.forwards=false, .lead=-.3, .maxSpeed=110}, false);
	pros::delay(130);
	spintake = 0;
	//grab mogo
	chassis.moveToPose(-3,29, 90,1500,{.lead=.3, }, false);
	spintake = 1;
	pros::delay(300);
	// deposit
	mogochassis.moveToPose(16.5,2, 90,1000,{.lead=.6}, false);
	mogochassis.moveToPose(42,0, 90,2000,{.forwards=false, .lead=.4}, false);
	clampready=false;
	spintake=0;
	//grab 2nd mogo
	chassis.moveToPose(38,0, 90,1000,{.forwards=false, .lead=.5, .maxSpeed=115}, false);
	clampready=true;
	chassis.moveToPose(38, 36, 0,2000,{ .lead=.1}, false);
	//grab 2nd disc
	spintake=true;
	mogochassis.moveToPose(60, 34, 270, 2000, {.forwards=false, .lead=.2}, false);
	// ladder touch
	armLatch.state=2;
	mogochassis.moveToPose(10, 40, 135, 2000, {.forwards=false, .lead=.3}, false);
	spintake=0;
}
void blueSoloAWP(){
	//7 inches forward is edge of tile
	// 7.5 inches off left
	// alliance stake
	chassis.setPose(0, 0, 90);
	chassis.moveToPoint(-18, 0 ,2000, {.forwards=false}, false);
	chassis.moveToPoint(-13, 0,  2000, {}, false);
	chassis.turnToHeading(180, 2000, {}, false);
	chassis.moveToPoint(-13,-8,500,{},false);
	spintake = 1;
	pros::delay(600);
	spintake = 0;
	// grab first disc
	chassis.moveToPoint(-16,2,1000,{.forwards=false},false);
	spintake = 1;
	chassis.moveToPose(32,36, 0, 2000, {.forwards=false, .lead=-.3,}, false);
	pros::delay(100);
	spintake = 0;
	//grab mogo
	chassis.moveToPose(4,32, 270,1500,{.lead=.3}, false);
	spintake = 1;
	// deposit
	mogochassis.moveToPose(-74,-2, 225,1500,{.lead=.2}, false);
	clampready=false;
	spintake=0;
	//grab 2nd mogo
	chassis.moveToPose(-44,0, 270,1500,{.forwards=false, .lead=.5}, false);
	clampready=true;
	chassis.moveToPose(-44, 30, 0,1500,{ .lead=.1}, false);
	//grab 2nd disc
	spintake=true;
	mogochassis.moveToPose(-70, 36, 90, 2000, {.forwards=false, .lead=.2}, false);
	// ladder touch
	armLatch.state=2;
	mogochassis.moveToPose(-36, 40, 225, 2000, {.forwards=false, .lead=.3}, false);
	spintake=0;
}

void skills(){
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	mogochassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.setPose(0,-16,180);
	mogochassis.setPose(0,-16,180);

	//alliance
	spintake = 1;
	pros::delay(500);
	spintake = 0;
	//pickup first mogo
	chassis.moveToPoint(0, -4, 500, {.forwards=false}, false);
	chassis.moveToPose(-26, 1, -90, 1700, {.lead=.3}, false);
	//pickup first two disks
	spintake=1;
	mogochassis.moveToPose(-24, 24, 180, 2000, {.forwards=false, .lead=.3}, false);
	mogochassis.moveToPose(-45, 75, 180, 2500, {.forwards=false, .lead=.5}, false);
	pros::delay(400);
	//ladybrown first
	mogochassis.moveToPoint(-42, 46,  2000, {}, false);
	armLatch.state=1;
	spintake=1;
	mogochassis.moveToPose(-60, 46.5, 90, 2000, {.forwards=false, .lead=.3}, false);
	pros::delay(50);
	spintake=0;
	armLatch.state=2;
	mogochassis.moveToPose(-62.5, 46.5, 90, 1500, {.forwards=false});
	pros::delay(400);
	mogochassis.moveToPose(-56, 48, 90, 1200, {}, false);
	armLatch.state=0;
	//corner rings
	spintake=1;
	mogochassis.moveToPose(-46, 22, 0, 2000, {.forwards=false, .lead=.3}, false);
	mogochassis.moveToPose(-46, 0, 0, 2000, {.forwards=false, .lead=.3}, false);
	mogochassis.moveToPose(-46, -13, 0, 2000, {.forwards=false, .lead=.3}, false);
	mogochassis.moveToPose(-46, 3, 0, 2000, {.lead=.3}, false);
	mogochassis.moveToPose(-60, 0, 90, 2000, {.forwards=false, .lead=.3}, false);
	//deposit
	mogochassis.moveToPose(-61, -12, 225, 2000, {.lead=-.2}, false);
	clampready=false;
	spintake=0;
	pros::delay(400);
	//grab 2nd mogo
	mogochassis.moveToPose(-48, 0, 240, 1000, {.forwards=false, .lead=.3}, false);
	chassis.moveToPose(0, 0, 90, 2300, {}, false);
	clampready=true;
	chassis.moveToPose(28, 0, 90, 2000, {}, false);
	//2nd two rings
	spintake=1;
	mogochassis.moveToPose(24, 24, 180, 2000, {.forwards=false, .lead=.3}, false);
	mogochassis.moveToPose(48, 75, 180, 2000, {.forwards=false, .lead=.5}, false);
	//right ladybrown
	mogochassis.moveToPoint(44, 48,  2000, {}, false);
	armLatch.state=1;
	spintake=1;
	mogochassis.moveToPose(62, 48, 270, 2000, {.forwards=false}, false);
	pros::delay(50);
	spintake=0;
	armLatch.state=2;
	mogochassis.moveToPose(64, 48, 270, 1500, {.forwards=false});
	pros::delay(400);
	mogochassis.moveToPose(44, 48, 270, 2000, {}, false);
	armLatch.state=0;
		//corner rings
		spintake=1;
		mogochassis.moveToPose(46, 24, 0, 2000, {.forwards=false, .lead=.3}, false);
		mogochassis.moveToPose(46, 0, 0, 2000, {.forwards=false, .lead=.3}, false);
		mogochassis.moveToPose(46, -16, 0, 2000, {.forwards=false, .lead=.3}, false);
		mogochassis.moveToPose(46, 0, 0, 2000, {.lead=.3}, false);
		mogochassis.moveToPose(60, -3, 270, 2000, {.forwards=false, .lead=.3}, false);
		//deposit
		mogochassis.moveToPose(60, -18, 120, 2000, {.lead=.3}, false);
		clampready=false;
		spintake=0;
		pros::delay(400);
		//grab 2nd mogo
		mogochassis.moveToPose(48, 0, 120, 1000, {.forwards=false, .lead=.3}, false);
	
}


bool AUTON = false;
void autonomous() {
	AUTON = true;
	// 0 = off
	// 1 = forward
	// 2 = reverse
	pros::Task intakeFilter([&]() {
		int ringType=0;
		if(autonum != 0 && autonum%2 == 0) targetType = 2;
        while (AUTON) {
			double value = optical.get_hue();
			// 0 is none, 1 is red, 2 is blue;
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
	
            pros::delay(6);
        }
    });

	pros::Task autoclamp([&]() {
		bool latch = false;
        while (AUTON) {
			if (clampready && distanceSensor.get() < 50){
				pros::delay(50);
				latch = true;
				clamp.set_value(true);
			}
			else if (clampready && latch){
				clamp.set_value(true);
			}else{
				clamp.set_value(false);
				latch = false;
			}
	
            pros::delay(6);
        }
    });

	pros::Task arm([&]() {
		bool latch = false;
		int desiredPos=0;
        while (AUTON) {
			if (armLatch.state == 0) desiredPos = 0;
			else if (armLatch.state == 1) desiredPos = 120;
			else if (armLatch.state == 2) desiredPos = 666;
			double output = armPID.update(desiredPos - armEncoder.get_value());
			armMotor.move_velocity(output);	
            pros::delay(10);
        }
    });
	//skills();
	switch (autonum) {
		case 0:
			skills();
			break;
		case 1:
			redRingSide();
			break;
		case 2:
			blueRingSide();
			break;
		case 3:
			redStakeSide();
			break;
		case 4:
			blueStakeSide();
			break;
		case 5:
			redRingFinals();
			break;
		case 6:
			blueRingFinals();
			break;
		case 7:
			redStakeFinals();
			break;
		case 8:
			blueStakeFinals();
			break;
	}
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

	// drive record?
	FILE* filewrite = fopen("/usd/rerun.txt", "w");
	fprintf(filewrite, "");
	fclose(filewrite);

	bool record = false;
	double desiredPos = 0;
	int oldTime=0;
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
		if (armLatch.state == 0) desiredPos = 0;
		else if (armLatch.state == 1) desiredPos = 120;
		else if (armLatch.state == 2) desiredPos = 666;
		double output = armPID.update(desiredPos - armEncoder.get_value());
		armMotor.move_velocity(output);
		
		pros::delay(10);
		if(!record) continue;
		filewrite = fopen("/usd/rerun.txt", "a");
		//pros::lcd::print(1, "Battery %: %d", pros::battery::get_capacity());
		fprintf(filewrite, "tL.move_voltage(%d);\n", tL.get_voltage());
		fprintf(filewrite, "mL.move_voltage(%d);\n", mL.get_voltage());
		fprintf(filewrite, "bL.move_voltage(%d);\n", bL.get_voltage());
		fprintf(filewrite, "tR.move_voltage(%d);\n", tR.get_voltage());
		fprintf(filewrite, "mR.move_voltage(%d);\n", mR.get_voltage());
		fprintf(filewrite, "bR.move_voltage(%d);\n", bR.get_voltage());
		fprintf(filewrite, "firstStageIntake.move_voltage(%d);\n", firstStageIntake.get_voltage());
		fprintf(filewrite, "secondStageIntake.move_voltage(%d);\n", secondStageIntake.get_voltage());
		fprintf(filewrite, "armMotor1.move_voltage(%d);\n", armMotor1.get_voltage());
		fprintf(filewrite, "armMotor2.move_voltage(%d);\n", armMotor2.get_voltage());
		fprintf(filewrite, "clamp.set_value(%i);\n", clampLatch.state);

		fprintf(filewrite, "pros::delay(%i);\n", pros::millis()-oldTime);
		oldTime = pros::millis();
		fclose(filewrite);
	}
}