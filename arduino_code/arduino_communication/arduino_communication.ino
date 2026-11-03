#include <Stepper.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

const int stepsPerRevolution = 400;

// Define pins
const int dirPin_AP = 4, stepPin_AP = 5;
const int dirPin_ML = 2, stepPin_ML = 3;
const int dirPin_Linear_out = 8, stepPin_Linear_out = 10;
const int dirPin_Linear_in = 12, stepPin_Linear_in = 11;

// Initialize steppers
Stepper Stepper_ML(stepsPerRevolution, dirPin_ML, stepPin_ML);
Stepper Stepper_AP(stepsPerRevolution, dirPin_AP, stepPin_AP);
Stepper Linear_Catheter(stepsPerRevolution, dirPin_Linear_out, stepPin_Linear_out);
Stepper Linear_Delivery(stepsPerRevolution, dirPin_Linear_in, stepPin_Linear_in);

// ROS node and publishers
ros::NodeHandle nh; // Arduino node initialization
std_msgs::Int32 act, dati_step;
ros::Publisher pub("Finish_Actuation", &act);         
ros::Publisher pub_step("dati_step", &dati_step);

// Limit and counter variables
int step_ML = 0, step_AP = 0, step_LIN = 0;
const int offset_LIN = 0, offset_ML = 0, offset_AP = 0;
int count_LIN = 0, count_LIN_old = 0, count_ML = 0, count_ML_old = 0, count_AP = 0, count_AP_old = 0;
const int MAX_LIN = 30000, MAX_ML = 10000, MAX_AP = 10000;
const int MIN_LIN = -10000, MIN_ML = -10000, MIN_AP = -10000; // Aggiungi limiti minimi espliciti

// Helper function to enforce limits
void check_and_correct_limits(int &step, int &count, int &count_old, const int max_limit, const int min_limit, const char* motor_name) {
    count += step;
    if (count >= max_limit) {
        String log_message = "MAX " + String(motor_name);
        nh.loginfo(log_message.c_str());
        step = max_limit - count_old;
        count = max_limit;
    } else if (count <= min_limit) {
        String log_message = "MIN " + String(motor_name);
        nh.loginfo(log_message.c_str());
        step = min_limit - count_old;
        count = min_limit;
    } else {
        count_old = count;
    }
}

// Reset function
void reset_to_starting_positions() {
    int reverse_steps_LIN = -count_LIN_old + offset_AP;
    int reverse_steps_ML = -count_ML_old + offset_ML;
    int reverse_steps_AP = -count_AP_old + offset_LIN;

    // Actuate Linear Motors
   
    Stepper_ML.step(reverse_steps_ML);
    Stepper_AP.step(reverse_steps_AP);
    Linear_Catheter.step(reverse_steps_LIN);
    Linear_Delivery.step(reverse_steps_LIN);

    // Reset counters
    count_LIN_old = offset_LIN;
    count_ML_old = offset_ML;
    count_AP_old = offset_AP;
    count_LIN = offset_LIN;
    count_ML = offset_ML;
    count_AP = offset_AP;
}

// Subscriber callback function
void actuation(const std_msgs::Int32MultiArray &command) {
    nh.loginfo("Command received");
    step_LIN = command.data[0];
    step_ML = command.data[1];
    step_AP = command.data[2];

    // Perform limit checks and correction
    check_and_correct_limits(step_LIN, count_LIN, count_LIN_old, MAX_LIN, MIN_LIN, "LIN");
    check_and_correct_limits(step_ML, count_ML, count_ML_old, MAX_ML, MIN_ML, "ML");
    check_and_correct_limits(step_AP, count_AP, count_AP_old, MAX_AP, MIN_AP, "AP");

    // Publish updated step data
    dati_step.data = step_LIN;
    pub_step.publish(&dati_step);
    dati_step.data = step_ML;
    pub_step.publish(&dati_step);
    dati_step.data = step_AP;
    pub_step.publish(&dati_step);

    // Actuate Linear Motor
    Linear_Catheter.step(step_LIN);
    Linear_Delivery.step(step_LIN);

    // Actuate other steppers
    Stepper_ML.step(step_ML);
    Stepper_AP.step(step_AP);

    nh.loginfo("Actuation Finished");
    act.data = 1;
    pub.publish(&act);  // Publish actuation finished
}

void reset_all_motors(const std_msgs::Int32 &msg) {
    if (msg.data == 1) {
        reset_to_starting_positions();
        msg.data == 0;
    }
}

// Subscriber node to topic of displacement
ros::Subscriber<std_msgs::Int32MultiArray> sub("displacements_VC", &actuation);
ros::Subscriber<std_msgs::Int32> sub_reset("reset_motors", &reset_all_motors);

void setup() {
    // Set up pin modes
    pinMode(stepPin_AP, OUTPUT);
    pinMode(dirPin_AP, OUTPUT);
    pinMode(stepPin_ML, OUTPUT);
    pinMode(dirPin_ML, OUTPUT);
    pinMode(stepPin_Linear_out, OUTPUT);
    pinMode(dirPin_Linear_out, OUTPUT);
    pinMode(stepPin_Linear_in, OUTPUT);
    pinMode(dirPin_Linear_in, OUTPUT);

    // Ensure motor pins are set to LOW initially to prevent actuation
    digitalWrite(stepPin_AP, LOW);
    digitalWrite(dirPin_AP, LOW);
    digitalWrite(stepPin_ML, LOW);
    digitalWrite(dirPin_ML, LOW);
    digitalWrite(stepPin_Linear_out, LOW);
    digitalWrite(dirPin_Linear_out, LOW);
    digitalWrite(stepPin_Linear_in, LOW);
    digitalWrite(dirPin_Linear_in, LOW);

    // Set default motor speeds
    Stepper_ML.setSpeed(30);
    Stepper_AP.setSpeed(30);
    Linear_Catheter.setSpeed(150);
    Linear_Delivery.setSpeed(150);

    // Initialize ROS node and communication
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub);
    nh.advertise(pub_step);
    nh.subscribe(sub);
    nh.subscribe(sub_reset);
}

void loop() {
    nh.spinOnce();
}
