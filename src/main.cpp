
#include <Arduino.h>
#include <Joint.h>
#include <JointGoal.h>
#include <Interpolations.h>
#include <Comm.h>

#define mySSID "VenusWifi"
#define myPASSWORD "bisgatafuta"
#include <OTA.h>

#define LED_PIN 2
#define PUSHBUTTON 23
#define VERBOSE false

Joint *joints[6];
WiFiClient client;
WiFiServer wifiServer(1337);
Comm comm(&wifiServer, 2u);
// float joint_positions[6], joint_velocities[6];
float *joint_positions, *joint_velocities;
float joint_positions_t[100][6], joint_velocities_t[100][6];

void set_joint_positions( float *joint_positions, float *joint_velocities, bool angles_in_rad){
    int tdelay = 1;
    JointGoal **jg;
    bool all_reached = false;

    jg = new JointGoal*[6];
    for (int i = 0; i < 6; i++) {
        if(!angles_in_rad)
            joint_positions[i] = deg2rad(joint_positions[i]);
        jg[i] = new JointGoal(joints[i], joint_positions[i], joint_velocities[i], Interpolations::linear);
    }
    while(!all_reached){
        all_reached = true;
        for (int i = 0; i < 6; i++) {
            jg[i]->step();
            all_reached = all_reached && jg[i]->reached();
        }
        comm.publish_joint_info(joints);
        delay(tdelay);
    }

    // Cleanning up
    for (int i = 0; i < 6; i++) {
        delete jg[i];
    }
    delete jg;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PUSHBUTTON,INPUT_PULLUP);

    joints[0] = new Joint(18,   0);  // Joint 1 MG966R
    joints[1] = new Joint(4,    0);  // Joint 2 MG966R
    joints[2] = new Joint(19,   6);  // Joint 3 MG966R
    joints[3] = new Joint(2,    0);  // Joint 4 MG966R
    joints[4] = new Joint(21,   -8);  // Joint 5 TG90
    joints[5] = new Joint(22,   0);  // Joint 6 TG90
    // joints[0]->write(0);
    // joints[1]->write(0);
    // joints[2]->write(0);
    // joints[3]->write(0);
    // joints[4]->write(0);
    // joints[5]->write(0);
    joints[0]->write(0);
    joints[1]->write(90);
    joints[2]->write(-90);
    joints[3]->write(0);
    joints[4]->write(90);
    joints[5]->write(0);

    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }
    setupOTA("yara");
    wifiServer.begin();
}

void loop(){

    // ArduinoOTA.handle();
    unsigned long start, elapsed;

    // // Manual button override:
    // if (digitalRead(PUSHBUTTON) == LOW) {
    //     // Setting arm to home position
    //     joint_positions[0] = 0;
    //     joint_positions[1] = 90;
    //     joint_positions[2] = -90;
    //     joint_positions[3] = 0;
    //     joint_positions[4] = 90;
    //     joint_positions[5] = 0;
    //     joint_velocities[0] = 0.0872665;
    //     joint_velocities[1] = 0.0872665;
    //     joint_velocities[2] = 0.0872665;
    //     joint_velocities[3] = 0.0872665;
    //     joint_velocities[4] = 0.0872665;
    //     joint_velocities[5] = 0.0872665;
    //     set_joint_positions(joint_positions,joint_velocities, false);
    // }
    if(comm.client_available()){
        if(VERBOSE)
            Serial.println("Client conected! Waiting for incomming data...");
        while(comm.client_connected()){
            comm.publish_joint_info(joints);
            if(comm.check_incomming_data()){
                if(VERBOSE){
                    Serial.println("Received data! Processing...");
                }

                // comm.process_incomming_data(joint_positions, joint_velocities);
                unsigned int no_of_points = comm.process_incomming_trajectory_data(joint_positions_t, joint_velocities_t);
                if(VERBOSE){
                    Serial.print("Data processed. No of points: "); Serial.println(no_of_points);
                }
                for(int i = 0; i < no_of_points; i++){
                    joint_positions = joint_positions_t[i];
                    joint_velocities = joint_velocities_t[i];
                    if(VERBOSE){
                        Serial.print("Data processed. Target joint pos.: (");
                        Serial.print(joint_positions[0]); Serial.print(", ");
                        Serial.print(joint_positions[1]); Serial.print(", ");
                        Serial.print(joint_positions[2]); Serial.print(", ");
                        Serial.print(joint_positions[3]); Serial.print(", ");
                        Serial.print(joint_positions[4]); Serial.print(", ");
                        Serial.print(joint_positions[5]);
                        Serial.print("), with target vel.: (");
                        Serial.print(joint_velocities[0]); Serial.print(", ");
                        Serial.print(joint_velocities[1]); Serial.print(", ");
                        Serial.print(joint_velocities[2]); Serial.print(", ");
                        Serial.print(joint_velocities[3]); Serial.print(", ");
                        Serial.print(joint_velocities[4]); Serial.print(", ");
                        Serial.print(joint_velocities[5]); Serial.println(").");
                        Serial.println("Setting target pos and velocity...");
                    }
                    // elapsed = millis() - start;
                    // Serial.print(elapsed); Serial.println("ms");
                    set_joint_positions(joint_positions, joint_velocities, true);
                    start = millis();
                    if(VERBOSE){
                        Serial.println("Done! Sending 'done' message...");
                    }

                    comm.send_message(comm.JOINT_POSITION_SET);
                    if(VERBOSE){
                        Serial.println("Message sent. Waiting for incomming data...");
                        Serial.println();
                    }
                }
            }
        }
        comm.stop_client();
        if(VERBOSE)
            Serial.println("Client disconected...");
    }
}