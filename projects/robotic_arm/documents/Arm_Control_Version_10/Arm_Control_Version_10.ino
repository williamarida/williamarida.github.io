// Version 9 - April 15th, 2025 - 1:00 p.m.

// ME 5751
// Preliminary Arm Code

// paper placement note

      //4 7/8 inches in the y 



// Current Version Notes: Pickup task works fine, but circle tracing does not work well at all.



// Include needed libraries

// Define which task you want to execute - pickup == true for block pickup, pickup == false for circle tracing

 bool pickup = false; 

 // Specify the number of tasks

 const int no_tasks = 4;

#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


 // Assign servos

 Servo servo_grip;    // gripper servo
 Servo servo_rhs;    // Right-hand side of arm servo (from front view) - Corresponds to theta 1
 Servo servo_lhs;    // Left-hand side of arm  servo (from front view) - Corresponds to theta 2

 // define angles which will be used to close and open the gripper

 float grip_close_ang = 63;
 float grip_open_ang = 50;

 // Assign pins to the servos 

 const int servo_grip_pin = 3; 
 const int servo_rhs_pin = 6;
 const int servo_lhs_pin = 5;

 // Assign pins to stepper

 const int step_pin = 8;
 const int drive_pin = 9;

 // Initialize the stepper

 AccelStepper stepper_a(1, step_pin, drive_pin);    //connect driver to pins 10 and 11

 // Find the conversion limit for actual angles used by servos

 float th1_convLim = 180*0.65;
 float th2_convLim = 180*0.675;


 // Next, define a home position (need to update once suitable home position is found)

  float EOx_3d_home = 0;
  float EOy_3d_home = 240;
  float EOz_3d_home = 30; 


  // Next, create vectors for the deired 

  //float EOx_3d_des = 210;
  //float EOy_3d_des = 360;
  //float EOz_3d_des = -70;


  // Next, define the drop off position

  float EOx_3d_do = -80;
  float EOy_3d_do = 348;
  float EOz_3d_do = 0;

    // Next, create vectors for the task position coordinates, these will include drop off coordinates

    // Start with the x-coordinate vector

  const float EOx_3d_des_1 = 40;
  const float EOx_3d_des_2 = 120;
  const float EOx_3d_des_3 = 42;
  const float EOx_3d_des_4 = 122;

  float EOx_3d_des[no_tasks] = {EOx_3d_des_1, EOx_3d_des_2, EOx_3d_des_3, EOx_3d_des_4};


   // Next, the y-coordinate vector

  const float EOy_3d_des_1 = 300;
  const float EOy_3d_des_2 = 300;
  const float EOy_3d_des_3 = 400;
  const float EOy_3d_des_4 = 400;

  float EOy_3d_des[no_tasks] = {EOy_3d_des_1, EOy_3d_des_2, EOy_3d_des_3, EOy_3d_des_4};

  // Next, the z-coordinate vector

  // Note, for the Z coordinate vector, create a consistent vertical height value, should be same for all the tasks

  const float EOz_3d_des_const = -86;

  float EOz_3d_des[no_tasks] = {EOz_3d_des_const, EOz_3d_des_const, EOz_3d_des_const, EOz_3d_des_const};

  
  
  // Now create the point for the pen pickup

  const float EOx_3d_pen = -135;
  const float EOy_3d_pen = 395;
  const float EOz_3d_pen = -30;


  // Now choose the desired height at which the circle will be traced

  const float EOz_3d_circ = -30;


  // Define the desired resolution/smoothness for placement-motion tasks and circle tracing

  const int resolution = 23;      // pickup/placement resolution
  const int resolution_2 = 35;    // circle-tracing resolution

  // Define the time pause between each resolution step
  
  int time_pause_1 = 30;      // miliseconds - delay between forward steps
  int time_pause_2 = 30;      // miliseconds - delay between return steps



// Create a functions to find phi_1, the orientation of the robatic arm in the horizontal with respect to the starting position
// and find the coordinates of point Q in the 2D-arm plane

// Reference your "3D inverse kinematics" diagram to understand the origin and axis system used

// Note, you need to verify that your selected origin is correct

// EOy_3d is the end effector's horizontal component that extends through the arm's initial orientation. 
// EOx_3d is the end effector's horizontal in the x-axis direction component perpedicular to the y-axis and consistent with the right-hand rule.
// EOz_3d is the end effector's vertical component 




// First find phi_1, in radians

float Phi_1_Calc(float EOx_3d, float EOy_3d) {

 // Define geometry and pi

  const float Pi = 3.14159265359;
  const float R1 = 60;
  const float R2 = 141.15;
  const float R3 = R2;
  const float R4 = 190;
  const float QD = R4 - R1;
  const float EQ_h = 185;     // Update this value once you have a more precise measurement
  const float EQ_red_h = EQ_h*0.8;
  const float EQ_v = 23;      // Update this value once you have a more precise measurement


  float phi_1 = atan2(EOy_3d, EOx_3d);       // calculate the end effector orientation with respect to x-axis
 
  return phi_1;

}



// Next find the x prime coordinate in the 2D plane

float Qxp_2d_calc(float phi_1, float EOx_3d, float EOy_3d) {

  const float Pi = 3.14159265359;
  const float R1 = 60;
  const float R2 = 141.15;
  const float R3 = R2;
  const float R4 = 190;
  const float QD = R4 - R1;
  const float EQ_h = 185;     // Update this value once you have a more precise measurement
  const float EQ_red_h = EQ_h*0.8;
  const float EQ_v = 23;      // Update this value once you have a more precise measurement


  // 3D domain

  float Qx_3d = EOx_3d - cos(phi_1)*EQ_red_h;      // find the position of point Q in the 3D horizontal x-direction
  float Qy_3d = EOy_3d - sin(phi_1)*EQ_red_h;      // find the position of point Q in the 3D horizontal y-direction
  

   // 2D domain

  float Qxp_2d = (-1)*sqrt(sq(Qx_3d) + sq(Qy_3d));


  return Qxp_2d;

}



// Next find the y prime coordinate in the 2D plane

float Qyp_2d_calc(float phi_1, float EOz_3d) {

  const float Pi = 3.14159265359;
  const float R1 = 60;
  const float R2 = 141.15;
  const float R3 = R2;
  const float R4 = 190;
  const float QD = R4 - R1;
  const float EQ_h = 185;            // Update this value once you have a more precise measurement
  const float EQ_red_h = EQ_h*0.8;
  const float EQ_v = 23;             // Update this value once you have a more precise measurement

  float Qyp_2d = EOz_3d + EQ_v;                     // vertical position of Q is the same as that of the end effector

  return Qyp_2d;

}




// This function performs the two dimensional inverse kinematics for theta 1 given the 2D coordinates of Q

float inv_kin_th1(float Qxp_2d, float Qyp_2d) {

  // Define geometry and pi

  const float Pi = 3.14159265359;
  const float R1 = 60;
  const float R2 = 141.15;
  const float R3 = R2;
  const float R4 = 190;
  const float QD = R4 - R1;
  const float EQ = 100;     // This value is a placeholder, update later


  float G1 = sq(R2) - sq(Qxp_2d) - sq(QD) - sq(Qyp_2d);  // G1-4 are intermediate values
  float G2 = sq(2*Qxp_2d*QD) + sq(2*Qyp_2d*QD);
  float G3 = 2*Qxp_2d*QD;
  float G4 = 2*Qyp_2d*QD;


  float J1 = sqrt(G2);   // J1-2 are intermediate values
  float J2 = G1/(J1);


  float th_1 = asin(J2) - atan2(G3, G4);
  float th_1_d = th_1*(180/Pi);                        // servos take arguements in degrees, so conversion is necessary 


  return th_1_d;

 }



 // This function performs the two dimensional inverse kinematics for theta 2 given the 2D coordinates of Q


 float inv_kin_th2(float Qxp_2d, float Qyp_2d, float th1_d) {

  // Define geometry and pi

  const float Pi = 3.14159265359;
  const float R1 = 60;
  const float R2 = 141.15;
  const float R3 = R2;
  const float R4 = 190;
  const float QD = R4 - R1;
  const float EQ = 100;     // This value is a placeholder, update later

  float th1_r = th1_d*(Pi/180);

  float V1 = QD*cos(th1_r);
  float V2 = Qxp_2d + V1;
  float V3 = V2/R2;

  float th2_r = acos(V3);

  float th2_d = th2_r*(180/Pi);

  return th2_d;

 }



 // Next, create a function that determines the horizontal, 2d increments between the home position and desired task position

 float incrm_fnct(float des_x_2d, float home_x_2d, float res) {
 
 float diff_Qx = abs(des_x_2d - home_x_2d);
 float incrm_Qx = diff_Qx/res;

  return incrm_Qx;

}




// Now, create a function that calculates the vertical height in the line that corresponds to the x-coordinate position

float Qyp_fnct(float Qxp_val, float des_x_2d, float home_x_2d, float des_y_2d, float home_y_2d) {

float diff_num = des_y_2d - home_y_2d;
float diff_den = des_x_2d - home_x_2d;

float Qyp_calc = (diff_num/diff_den)*(Qxp_val - home_x_2d) + home_y_2d; 

return Qyp_calc;

}




void setup() {


  // Define Pi

  const float Pi = 3.14159265359;

  // Initialize Serial Monitor 
  
  Serial.begin(9600);



   // Attach the servos to their designated pins 

 servo_grip.attach(servo_grip_pin);   
 servo_rhs.attach(servo_rhs_pin);  // Again, corresponds to theta 1
 servo_lhs.attach(servo_lhs_pin);  // Again, corresponds to theta 2



  // Determine angle for phi 1 at home position

 float phi_1_h = Phi_1_Calc(EOx_3d_home, EOy_3d_home);

  // Determine angle for phi 1 at drop-off position

 float phi_1_do = Phi_1_Calc(EOx_3d_do, EOy_3d_do);



 // Convert from 3D end effector position to 2D Q position for home position

 float Qx_home = Qxp_2d_calc(phi_1_h, EOx_3d_home, EOy_3d_home);

 float Qy_home = Qyp_2d_calc(phi_1_h, EOz_3d_home);

 // Do the same for the Q drop off poisition in the 2D plane

 float Qx_do = Qxp_2d_calc(phi_1_do, EOx_3d_do, EOy_3d_do);

 float Qy_do = Qyp_2d_calc(phi_1_do, EOz_3d_do);






  // Find the home position angles 

 float th_1_home = inv_kin_th1(Qx_home, Qy_home);
 float th_2_home = inv_kin_th2(Qx_home, Qy_home, th_1_home);
 float th_grip_home = 0;                                          // can be chosen, no calculation needed



 // Map the home position over to the actual angles range used by servos
 
float th_1_home_conv = map(th_1_home, 0, 180, th1_convLim, 0);

float th_2_home_conv = map(th_2_home, 0, 180, 0, th2_convLim);



 // Give the servos an initial position to avoid them jittering

 servo_grip.write(grip_open_ang);
 servo_rhs.write(th_1_home_conv);  //needs to be mapped onto actual angles
 servo_lhs.write(th_2_home_conv);  //needs to be mapped onto actual angles




// Set speed and acceleration for the stepper

  stepper_a.setMaxSpeed(1000);
  stepper_a.setAcceleration(400);

// Set initial position as zero

  stepper_a.setCurrentPosition(0);


  // Now define a line from our home position to our drop off position

  float increment_hdo = incrm_fnct(Qx_do, Qx_home, resolution);    // find the increment in the linear line

  float Qx_line_hdo[resolution];   // intialize line for Qx values in linear line

  Qx_line_hdo[0] = Qx_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("X coordinate (home to drop off)");
  Serial.println(Qx_line_hdo[0]);

  // for loop serves as a pseudo linspace function, creating a vector for the x-coordinate values in linear line

  for (int i = 1; i < resolution; i++) {

    Qx_line_hdo[i] = Qx_line_hdo[i-1] - increment_hdo;

    Serial.print("X coordinate (home to drop off) ");
    Serial.println(Qx_line_hdo[i]);

    }



    // Next, generate the vertical Qyp_2d values that correspond to the previous x-coordinates for home to drop off


  float Qy_line_hdo[resolution];


  Qy_line_hdo[0] = Qy_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("Y coordinate ");
  Serial.println(Qy_line_hdo[0]);


  for (int i = 1; i < resolution; i++) {

    Qy_line_hdo[i] = Qyp_fnct(Qx_line_hdo[i], Qx_do, Qx_home, Qy_do, Qy_home);
    
    Serial.print("Y coordinate ");
    Serial.println(Qy_line_hdo[i]);

  }


  Serial.println();
  Serial.println();
  
  float th1_line_hdo[resolution];
  float th2_line_hdo[resolution];

  float th1_conv_hdo[resolution];
  float th2_conv_hdo[resolution];




  // Now find the theta 1 and theta 2 values corresponding to each point in the line from home to drop off

  for (int i = 0; i < resolution; i++) {

    th1_line_hdo[i] = inv_kin_th1(Qx_line_hdo[i], Qy_line_hdo[i]);  // Values for theta 2 are correct. Theta 1 keeps overflowing!


  }

  for (int i = 0; i < resolution; i++) {

    
    th2_line_hdo[i] = inv_kin_th2(Qx_line_hdo[i], Qy_line_hdo[i], th1_line_hdo[i]);

  }

  // convert kinematic angles to servo angles

  for (int i =0; i < resolution; i++) {

    th1_conv_hdo[i] = map(th1_line_hdo[i], 0, 180, th1_convLim, 0);
    th1_conv_hdo[i] = constrain(th1_conv_hdo[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted:");
    Serial.println(th1_conv_hdo[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution; i++) {

   th2_conv_hdo[i] = map(th2_line_hdo[i], 0, 180, 0, th2_convLim);
   th2_conv_hdo[i] = constrain(th2_conv_hdo[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted:");
   Serial.println(th2_conv_hdo[i]);


  }







  servo_grip.write(grip_open_ang);





// If pickup is true, then begin picking up blocks, otherwise, circle tracing will begin

  if (pickup == true) {

    for (int k = 0; k < no_tasks; k++) {


 // Determine angle for phi 1 at task position

 float phi_1_t = Phi_1_Calc(EOx_3d_des[k], EOy_3d_des[k]);


 // Find the position of the Q task poisition in the 2D plane

 float Qx_des = Qxp_2d_calc(phi_1_t, EOx_3d_des[k], EOy_3d_des[k]);

 float Qy_des = Qyp_2d_calc(phi_1_t, EOz_3d_des[k]);







  // Now define a line between our home position and desired task position in the 2D plane

  float increment_ht = incrm_fnct(Qx_des, Qx_home, resolution);    // find the increment in the linear line

  float Qx_line_ht[resolution];   // intialize line for Qx values in linear line

  Qx_line_ht[0] = Qx_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("X coordinate (home to desired)");
  Serial.println(Qx_line_ht[0]);

  // for loop serves as a pseudo linspace function, creating a vector for the x-coordinate values in linear line

  for (int i = 1; i < resolution; i++) {

    Qx_line_ht[i] = Qx_line_ht[i-1] - increment_ht;

    Serial.print("X coordinate (home to desired) ");
    Serial.println(Qx_line_ht[i]);

    }


  // Next, generate the vertical Qyp_2d values that correspond to the previous x-coordinates


  float Qy_line_ht[resolution];


  Qy_line_ht[0] = Qy_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("Y coordinate ");
  Serial.println(Qy_line_ht[0]);


  for (int i = 1; i < resolution; i++) {

    Qy_line_ht[i] = Qyp_fnct(Qx_line_ht[i], Qx_des, Qx_home, Qy_des, Qy_home);
    
    Serial.print("Y coordinate ");
    Serial.println(Qy_line_ht[i]);

  }




  
  Serial.println();
  Serial.println();
  
  float th1_line[resolution];
  float th2_line[resolution];

  float th1_conv[resolution];
  float th2_conv[resolution];


  // Now find the theta 1 and theta 2 values corresponding to each point in the line

  for (int i = 0; i < resolution; i++) {

    th1_line[i] = inv_kin_th1(Qx_line_ht[i], Qy_line_ht[i]);  // Values for theta 2 are correct. Theta 1 keeps overflowing!


  }

  for (int i = 0; i < resolution; i++) {

    
    th2_line[i] = inv_kin_th2(Qx_line_ht[i], Qy_line_ht[i], th1_line[i]);

  }

  // convert kinematic angles to servo angles

  for (int i =0; i < resolution; i++) {

    th1_conv[i] = map(th1_line[i], 0, 180, th1_convLim, 0);
    th1_conv[i] = constrain(th1_conv[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted:");
    Serial.println(th1_conv[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution; i++) {

   th2_conv[i] = map(th2_line[i], 0, 180, 0, th2_convLim);
   th2_conv[i] = constrain(th2_conv[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted:");
   Serial.println(th2_conv[i]);


  }

Serial.println();

// Display theta 1 values in serial monitor

for (int i = 0; i < resolution; i++) {

Serial.print("Theta 1: ");
Serial.println(th1_line[i]);

}

// Display theta 2 values in serial monitor

Serial.println();

for (int i = 0; i < resolution; i++) {

Serial.print("Theta 2: ");
Serial.println(th2_line[i]);

}





 


// Orient stepper toward home position

float phi_1_d_h = phi_1_h*(180/Pi);    // convert from radians to degrees
float phi_1_s_h = map(phi_1_d_h, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

stepper_a.moveTo(phi_1_s_h);
stepper_a.runToPosition();
Serial.print("stepper home: ");
Serial.println(phi_1_d_h);

delay(60);

// Orient stepper toward task position

float phi_1_d_t = phi_1_t*(180/Pi);    // convert from radians to degrees
float phi_1_s_t = map(phi_1_d_t, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

stepper_a.moveTo(phi_1_s_t);
stepper_a.runToPosition();
Serial.print("stepper task: ");
Serial.println(phi_1_d_t);

delay(60);


// Command linkage to follow the linear-line trajectory angles to task point


Serial.println();

for (int i = 0; i < resolution; i++) {

servo_rhs.write(th1_conv[i]);
servo_lhs.write(th2_conv[i]);

delay(time_pause_1 + k*2);

}



// Once arm reaches desired task point, command gripper to close or open, depending on current context

delay(250);


  servo_grip.write(grip_close_ang);


delay(150);


// Command linkage to return to home position from task position


Serial.println();

for (int i = resolution - 1; i >= 0; i--) {

servo_rhs.write(th1_conv[i]);
servo_lhs.write(th2_conv[i]);

delay(time_pause_1 + k*2);

}



// Orient stepper toward drop-off position

float phi_1_d_do = phi_1_do*(180/Pi);    // convert from radians to degrees
float phi_1_s_do = map(phi_1_d_do, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

stepper_a.moveTo(phi_1_s_do);
stepper_a.runToPosition();
Serial.print("stepper drop off: ");
Serial.println(phi_1_s_do);

delay(200);



// Command linkage to follow the linear-line trajectory angles to drop-off point


Serial.println();

for (int i = 0; i < resolution; i++) {

servo_rhs.write(th1_conv_hdo[i]);
servo_lhs.write(th2_conv_hdo[i]);

delay(time_pause_1 + k*2);

}


// command gripper to open


delay(100);


  servo_grip.write(grip_open_ang);


delay(150);


// Command linkage to return to home position from drop-off position


Serial.println();

for (int i = resolution - 1; i >= 0; i--) {

servo_rhs.write(th1_conv_hdo[i]);
servo_lhs.write(th2_conv_hdo[i]);

delay(time_pause_1 + k*2);

}


// orient stepper back at home position 

stepper_a.moveTo(phi_1_s_h);
stepper_a.runToPosition();


// command gripper to open

delay(50);

  servo_grip.write(grip_open_ang);

  }

  }













// if pickup is false, then begin circle tracing

  if (pickup == false) {


    // create a vector of angles in radians for a full revolution

    float Pi_2x = 2*Pi;

    float angle_incr = incrm_fnct(0, Pi_2x, resolution_2);

    float angle_vec[resolution_2];

    angle_vec[0] = 0;

    for (int j = 1; j <= resolution_2; j++) {

      angle_vec[j] = angle_vec[j - 1] + angle_incr;
      

    }

    // Create vectors for x and y coordinates of circle

    float x_offset = 20;
    float y_offset = 280;

    float circle_x[resolution_2];
    float circle_y[resolution_2];

    for (int j = 0; j < resolution_2; j++) {

      circle_x[j] = 60*cos(angle_vec[j]) + x_offset;
      circle_y[j] = 60*sin(angle_vec[j]) + y_offset;

      Serial.print("Circle x coord");
      Serial.println(circle_x[j]);
      
      Serial.print("Circle y coord");
      Serial.println(circle_y[j]);
    

    }




    // Now that the circle coordinates have been created, the next task is to solve the kinematics corresponding to each circle coordinate

    float th1_line_circ[resolution_2];
    float th2_line_circ[resolution_2];
    float phi1_line_circ[resolution_2];

    float Qxp_line_circ[resolution_2];
    float Qyp_line_circ[resolution_2];



    // first solve for phi_1

    for (int j = 0; j < resolution_2; j++) {

     phi1_line_circ[j] = Phi_1_Calc(circle_x[j], circle_y[j]);
     Serial.print("Phi circle angle (currently wanted): ");
     Serial.println(phi1_line_circ[j]);

    }



    // Now, solve for Qxp and Qyp coordinates in circle

 
  for (int j = 0; j < resolution_2; j++) {

       Qxp_line_circ[j] = Qxp_2d_calc(phi1_line_circ[j], circle_x[j], circle_y[j]);
       Serial.print("Qxp value circle trace (currently wanted: ");
       Serial.println(Qxp_line_circ[j]);

    }

  for (int j = 0; j < resolution_2; j++) {

       Qyp_line_circ[j] = Qyp_2d_calc(phi1_line_circ[j], EOz_3d_circ);
       Serial.print("Qyp value circle trace (currently wanted: ");
       Serial.println(Qyp_line_circ[j]);

    }


    // Now solve for th1 and th2 

    for (int j = 0; j < resolution_2; j++) {

        th1_line_circ[j] = inv_kin_th1(Qxp_line_circ[j], Qyp_line_circ[j]);
        Serial.print("th1_line_circ: ");
        Serial.println(th1_line_circ[j]);
        
    }

    for (int j = 0; j < resolution_2; j++) {

        th2_line_circ[j] = inv_kin_th2(Qxp_line_circ[j], Qyp_line_circ[j], th1_line_circ[j]);
        Serial.print("th2_line_circ: ");
        Serial.println(th2_line_circ[j]);

    }






// convert kinematic angles to servo angles


  float th1_conv_circ[resolution_2];
  float th2_conv_circ[resolution_2];


  for (int i =0; i < resolution_2; i++) {

    th1_conv_circ[i] = map(th1_line_circ[i], 0, 180, th1_convLim, 0);
    th1_conv_circ[i] = constrain(th1_conv_circ[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted (circle angles):");
    Serial.println(th1_conv_circ[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution_2; i++) {

   th2_conv_circ[i] = map(th2_line_circ[i], 0, 180, 0, th2_convLim);
   th2_conv_circ[i] = constrain(th2_conv_circ[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted (circle angles):");
   Serial.println(th2_conv_circ[i]);


  }





// Solve for the stepper angle of the pen-holder

 float phi_1_p = Phi_1_Calc(EOx_3d_pen, EOy_3d_pen);


// solve for the 2D coordinates at the pen-holder

 float Qx_pen = Qxp_2d_calc(phi_1_p, EOx_3d_pen, EOy_3d_pen);

 float Qy_pen = Qyp_2d_calc(phi_1_p, EOz_3d_pen);







     // Now define a line between our home position and the pen-holder position in the 2D plane

  float increment_hp = incrm_fnct(Qx_pen, Qx_home, resolution);    // find the increment in the linear line

  float Qx_line_hp[resolution];   // intialize line for Qx values in linear line

  Qx_line_hp[0] = Qx_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("X coordinate (home to pen holder)");
  Serial.println(Qx_line_hp[0]);

  // for loop serves as a pseudo linspace function, creating a vector for the x-coordinate values in linear line

  for (int i = 1; i < resolution; i++) {

    Qx_line_hp[i] = Qx_line_hp[i-1] - increment_hp;

    Serial.print("X coordinate (home to pen) ");
    Serial.println(Qx_line_hp[i]);

    }


  // Next, generate the vertical Qyp_2d values that correspond to the previous x-coordinates


  float Qy_line_hp[resolution];


  Qy_line_hp[0] = Qy_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("Y coordinate - home to pen holder");
  Serial.println(Qy_line_hp[0]);


  for (int i = 1; i < resolution; i++) {

    Qy_line_hp[i] = Qyp_fnct(Qx_line_hp[i], Qx_pen, Qx_home, Qy_pen, Qy_home);
    
    Serial.print("Y coordinate - home to pen holder");
    Serial.println(Qy_line_hp[i]);

  }



 // Now find the theta 1 and theta 2 values corresponding to each point in the line from the home position to pen holder

 float th1_line_hp[resolution];
 float th2_line_hp[resolution];

 float th1_conv_hp[resolution];
 float th2_conv_hp[resolution];




  for (int i = 0; i < resolution; i++) {

    th1_line_hp[i] = inv_kin_th1(Qx_line_hp[i], Qy_line_hp[i]);  


  }

  for (int i = 0; i < resolution; i++) {

    
    th2_line_hp[i] = inv_kin_th2(Qx_line_hp[i], Qy_line_hp[i], th1_line_hp[i]);

  }

  // convert kinematic angles to servo angles

  for (int i =0; i < resolution; i++) {

    th1_conv_hp[i] = map(th1_line_hp[i], 0, 180, th1_convLim, 0);
    th1_conv_hp[i] = constrain(th1_conv_hp[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted (home to pen):");
    Serial.println(th1_conv_hp[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution; i++) {

   th2_conv_hp[i] = map(th2_line_hp[i], 0, 180, 0, th2_convLim);
   th2_conv_hp[i] = constrain(th2_conv_hp[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted (home to pen):");
   Serial.println(th2_conv_hp[i]);


  }







// This is where the new code additions begin


     // Now define a line between our pen-holder position and a position vertically above that point in the 2D plane


  float vert_offset = 40;

  float increment_v = incrm_fnct(Qy_pen + vert_offset, Qy_home, resolution/2);    // find the increment in the linear line

  float Qy_line_v[resolution/2];   // intialize line for Qy values in linear line

  Qy_line_v[0] = Qy_home;        // set the starting value of Qy to y-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("Y coordinate (pen holder vertical position)");
  Serial.println(Qy_line_v[0]);

  // for loop serves as a pseudo linspace function, creating a vector for the y-coordinate values in linear line

  for (int i = 1; i < resolution/2; i++) {

    Qy_line_v[i] = Qy_line_v[i-1] - increment_v;

    Serial.print("Y coordinate (pen to vert) ");
    Serial.println(Qy_line_v[i]);

    }


  // Next, generate the vertical Qxp_2d values that correspond to the previous y-coordinates

  float Qx_line_v[resolution/2];


  Qx_line_v[0] = Qx_pen;        // set the starting value of Qx to x-coordinate of pen position

  Serial.println();
  Serial.println();
  Serial.print("X coordinate - pen to vert");
  Serial.println(Qx_line_v[0]);


  for (int i = 1; i < resolution/2; i++) {

    Qx_line_v[i] = Qx_pen;
    
    Serial.print("Y coordinate - home to pen holder");
    Serial.println(Qx_line_v[i]);

  }



 // Now find the theta 1 and theta 2 values corresponding to each point in the line from the home position to pen holder

 float th1_line_v[resolution/2];
 float th2_line_v[resolution/2];

 float th1_conv_v[resolution/2];
 float th2_conv_v[resolution/2];




  for (int i = 0; i < resolution/2; i++) {

    th1_line_v[i] = inv_kin_th1(Qx_line_v[i], Qy_line_v[i]);  

  }


  for (int i = 0; i < resolution/2; i++) {
    
    th2_line_v[i] = inv_kin_th2(Qx_line_v[i], Qy_line_v[i], th1_line_v[i]);

  }

  // convert kinematic angles to servo angles

  for (int i =0; i < resolution/2; i++) {

    th1_conv_v[i] = map(th1_line_v[i], 0, 180, th1_convLim, 0);
    th1_conv_v[i] = constrain(th1_conv_v[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted (pen to vert):");
    Serial.println(th1_conv_v[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution/2; i++) {

   th2_conv_v[i] = map(th2_line_v[i], 0, 180, 0, th2_convLim);
   th2_conv_v[i] = constrain(th2_conv_v[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted (pen to vert):");
   Serial.println(th2_conv_v[i]);


  }




// Next, define a line between the end of our vertical position and our home position

  float increment_hv = incrm_fnct(Qx_pen, Qx_home, resolution);    // find the increment in the linear line

  float Qx_line_hv[resolution];   // intialize line for Qy values in linear line

  Qx_line_hv[0] = Qx_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("X coordinate (vertical position to home)");
  Serial.println(Qx_line_hv[0]);

  // for loop serves as a pseudo linspace function, creating a vector for the y-coordinate values in linear line

  for (int i = 1; i < resolution; i++) {

    Qx_line_hv[i] = Qx_line_hv[i-1] - increment_hv;

    Serial.print("X coordinate (pen to vert) ");
    Serial.println(Qx_line_hv[i]);

    }


  // Next, generate the vertical Qxp_2d values that correspond to the previous y-coordinates

  float Qy_line_hv[resolution];


  Qy_line_hv[0] = Qy_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("Y coordinate - vert to home");
  Serial.println(Qy_line_hv[0]);


  for (int i = 1; i < resolution; i++) {

    Qy_line_hv[i] = Qyp_fnct(Qx_line_hv[i], Qx_pen, Qx_home, Qy_line_v[resolution/2 - 1], Qy_home);
    
    Serial.print("Y coordinate - vert to home");
    Serial.println(Qy_line_hv[i]);

    

  }



 // Now find the theta 1 and theta 2 values corresponding to each point in the line from the home position to pen holder

 float th1_line_hv[resolution];
 float th2_line_hv[resolution];

 float th1_conv_hv[resolution];
 float th2_conv_hv[resolution];




  for (int i = 0; i < resolution; i++) {

    th1_line_hv[i] = inv_kin_th1(Qx_line_hv[i], Qy_line_hv[i]);  


  }

  for (int i = 0; i < resolution; i++) {

    
    th2_line_hv[i] = inv_kin_th2(Qx_line_hv[i], Qy_line_hv[i], th1_line_hv[i]);

  }

  // convert kinematic angles to servo angles

  for (int i =0; i < resolution; i++) {

    th1_conv_hv[i] = map(th1_line_hv[i], 0, 180, th1_convLim, 0);
    th1_conv_hv[i] = constrain(th1_conv_hv[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted (vert to home):");
    Serial.println(th1_conv_hv[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution; i++) {

   th2_conv_hv[i] = map(th2_line_hv[i], 0, 180, 0, th2_convLim);
   th2_conv_hv[i] = constrain(th2_conv_hv[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted (vert to home):");
   Serial.println(th2_conv_hv[i]);


  }





   // Now define a line between our home position and the first point in the circle in the 2D plane

  float increment_hc = incrm_fnct(Qxp_line_circ[0], Qx_home, resolution);    // find the increment in the linear line

  float Qx_line_hc[resolution];   // intialize line for Qx values in linear line

  Qx_line_hc[0] = Qx_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("X coordinate (home to first circle point)");
  Serial.println(Qx_line_hc[0]);

  // for loop serves as a pseudo linspace function, creating a vector for the x-coordinate values in linear line

  for (int i = 1; i < resolution; i++) {

    Qx_line_hc[i] = Qx_line_hc[i-1] - increment_hc;

    Serial.print("X coordinate (home to first circle point) ");
    Serial.println(Qx_line_hc[i]);

    }


  // Next, generate the vertical Qyp_2d values that correspond to the previous x-coordinates


  float Qy_line_hc[resolution];


  Qy_line_hc[0] = Qy_home;        // set the starting value of Qx to x-coordinate of home position

  Serial.println();
  Serial.println();
  Serial.print("Y coordinate - home to first circle point");
  Serial.println(Qy_line_hc[0]);


  for (int i = 1; i < resolution; i++) {

    Qy_line_hc[i] = Qyp_fnct(Qx_line_hc[i], Qxp_line_circ[0], Qx_home, Qyp_line_circ[0], Qy_home);
    
    Serial.print("Y coordinate - home to first circle point");
    Serial.println(Qy_line_hc[i]);

  }




 // Now find the theta 1 and theta 2 values corresponding to each point in the line from the home position to first point in circle

 float th1_line_hc[resolution];
 float th2_line_hc[resolution];

 float th1_conv_hc[resolution];
 float th2_conv_hc[resolution];

  for (int i = 0; i < resolution; i++) {

    th1_line_hc[i] = inv_kin_th1(Qx_line_hc[i], Qy_line_hc[i]);  


  }

  for (int i = 0; i < resolution; i++) {

    
    th2_line_hc[i] = inv_kin_th2(Qx_line_hc[i], Qy_line_hc[i], th1_line_hc[i]);

  }

  // convert kinematic angles to servo angles

  for (int i =0; i < resolution; i++) {

    th1_conv_hc[i] = map(th1_line_hc[i], 0, 180, th1_convLim, 0);
    th1_conv_hc[i] = constrain(th1_conv_hc[i], 0, th1_convLim);    // be careful, may supress identification of values out of workspace
    Serial.print("th1 converted (home to first circle point):");
    Serial.println(th1_conv_hc[i]);

  }

  // convert kinematic angles to servo angles

   for (int i =0; i < resolution; i++) {

   th2_conv_hc[i] = map(th2_line_hc[i], 0, 180, 0, th2_convLim);
   th2_conv_hc[i] = constrain(th2_conv_hc[i], 0, th2_convLim);    // be careful, may supress identification of values out of workspace
   Serial.print("th2 converted (home to first circle point):");
   Serial.println(th2_conv_hc[i]);


  }


  // Orient stepper toward home position

     float phi_1_d_h = phi_1_h*(180/Pi);    // convert from radians to degrees
     float phi_1_s_h = map(phi_1_d_h, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

     stepper_a.moveTo(phi_1_s_h);
     stepper_a.runToPosition();
     Serial.print("stepper home: ");
     Serial.println(phi_1_d_h);

     delay(1000);



  // Orient the stepper from the home position to the pen holder position

    float phi_1_d_p = phi_1_p*(180/Pi);    // convert from radians to degrees
    float phi_1_s_p = map(phi_1_d_p, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

    stepper_a.moveTo(phi_1_s_p);
    stepper_a.runToPosition();
    Serial.print("stepper home: ");
    Serial.println(phi_1_d_p);

    delay(1000);



  // Now, command the linkage to move from the home position to the pen holder position, following the linear line


Serial.println();


for (int i = 0; i < resolution; i++) {

servo_rhs.write(th1_conv_hp[i]);
servo_lhs.write(th2_conv_hp[i]);

delay(time_pause_1);

}



// Command the gripper to close

delay(500);

servo_grip.write(grip_close_ang);

delay(500);




// Now command linkage to go from pen position to vert position

for (int i = resolution/2 - 1; i >  0; i--) {

Serial.print("return angle th1 (vert to home): ");
Serial.println(th1_conv_v[i]);
servo_rhs.write(th1_conv_v[i]);


Serial.print("return angle th2 (vert to home): ");
Serial.println(th2_conv_v[i]);
servo_lhs.write(th2_conv_v[i]);

Serial.println();


delay(time_pause_2);

}


// Now command linkage to go from vert position to home position

for (int i = resolution - 1; i >  0; i--) {

Serial.print("return angle th1 (vert to home): ");
Serial.println(th1_conv_hv[i]);
servo_rhs.write(th1_conv_hv[i]);


Serial.print("return angle th2 (vert to home): ");
Serial.println(th2_conv_hv[i]);
servo_lhs.write(th2_conv_hv[i]);

Serial.println();


delay(time_pause_2);

}

// Orient stepper toward home position


     stepper_a.moveTo(phi_1_s_h);
     stepper_a.runToPosition();
     Serial.print("stepper home: ");
     Serial.println(phi_1_d_h);

     delay(1000);




// Now orient stepper toward the first point in the circle


    float phi1_circ_d = phi1_line_circ[0]*(180/Pi);    // convert from radians to degrees
    float phi1_circ_s = map(phi1_circ_d, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

    stepper_a.moveTo(phi1_circ_s);
    stepper_a.runToPosition();
    Serial.print("stepper home: ");
    Serial.println(phi1_circ_s);

    delay(1000);




// Now, command linkage to go from home position to first point in the circle

for (int i = 0; i < resolution; i++) {

servo_rhs.write(th1_conv_hc[i]);
servo_lhs.write(th2_conv_hc[i]);

delay(time_pause_1);

}

// Now at the circle, command the linkage to trace the circle

for (int i = 0; i < resolution_2; i++) {

    float phi1_line_circ_d = phi1_line_circ[i]*(180/Pi);    // convert from radians to degrees
    float phi1_line_circ_s = map(phi1_line_circ_d, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

    stepper_a.moveTo(phi1_line_circ_s);
    stepper_a.runToPosition();
    Serial.print("stepper home: ");
    Serial.println(phi1_line_circ_s);

    delay(50);

    servo_rhs.write(th1_conv_circ[i]);
    Serial.print("Theta 1 circle: ");
    Serial.println(th1_conv_circ[i]);

    servo_lhs.write(th2_conv_circ[i]);
    Serial.print("Theta 2 circle: ");
    Serial.println(th2_conv_circ[i]);

    delay(50);



}

delay(1500);


// Now, command the linkage to return home in the 2D plane


for (int i = resolution - 1; i >  0; i--) {

Serial.print("return angle th1 (circ to home): ");
Serial.println(th1_conv_hc[i]);
servo_rhs.write(th1_conv_hc[i]);


Serial.print("return angle th2 (circ to home): ");
Serial.println(th2_conv_hc[i]);
servo_lhs.write(th2_conv_hc[i]);

Serial.println();


delay(time_pause_2);

}


// Lastly, have the stepper orient itself back to the home position

    stepper_a.moveTo(phi_1_s_h);
    stepper_a.runToPosition();
    Serial.print("stepper home: ");
    Serial.println(phi_1_d_p);

    delay(1000);




  // Orient the stepper from the home position to the pen holder position


  
    stepper_a.moveTo(phi_1_s_p);
    stepper_a.runToPosition();
    Serial.print("stepper home: ");
    Serial.println(phi_1_d_p);

    delay(1000);




  // Send the linkage from home to vert position in 2D plane

for (int i = 0; i < resolution; i++) {

servo_rhs.write(th1_conv_hv[i]);
servo_lhs.write(th2_conv_hv[i]);

delay(time_pause_1);

}


// Send the linkage from vert position to pen position in 2D plane

for (int i = 0; i < resolution/2; i++) {

servo_rhs.write(th1_conv_v[i]);
servo_lhs.write(th2_conv_v[i]);

delay(time_pause_1);

}

// Command the gripper to open

delay(500);

servo_grip.write(grip_open_ang);

delay(500);


// now repeat the prior steps and return to home 




// Now command linkage to go from pen position to vert position

for (int i = resolution/2 - 1; i >  0; i--) {

Serial.print("return angle th1 (vert to home): ");
Serial.println(th1_conv_v[i]);
servo_rhs.write(th1_conv_v[i]);


Serial.print("return angle th2 (vert to home): ");
Serial.println(th2_conv_v[i]);
servo_lhs.write(th2_conv_v[i]);

Serial.println();


delay(time_pause_2);

}


// Now command linkage to go from vert position to home position

for (int i = resolution - 1; i >  0; i--) {

Serial.print("return angle th1 (vert to home): ");
Serial.println(th1_conv_hv[i]);
servo_rhs.write(th1_conv_hv[i]);


Serial.print("return angle th2 (vert to home): ");
Serial.println(th2_conv_hv[i]);
servo_lhs.write(th2_conv_hv[i]);

Serial.println();


delay(time_pause_2);




// Orient stepper toward home position

     float phi_1_d_h = phi_1_h*(180/Pi);    // convert from radians to degrees
     float phi_1_s_h = map(phi_1_d_h, 0, 180, -1550, 1550);    // convert to stepper pulses (where 6200 pulses is one rev, so 1550 pulses are 90 degrees)

     stepper_a.moveTo(phi_1_s_h);
     stepper_a.runToPosition();
     Serial.print("stepper home: ");
     Serial.println(phi_1_d_h);

     delay(1000);



}







  }


}



void loop() {


}
