#define NOMBER_MI_NEURONS 7
#include <DynamixelSDK.h>

#define ADDR_AX_ID 3

#define ADDR_AX_TORQUE_ENABLE 24 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION 30
#define ADDR_AX_PRESENT_POSITION 36
#define MOVING 46
#define ANGLE_CW_LIMIT                6
#define ANGLE_CCW_LIMIT               8

// Protocol version
#define PROTOCOL_VERSION 1.0 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID 1     // Dynamixel ID: 1
#define DXL_NEW_ID 1 // Dynamixel ID: 2

#define BAUDRATE 1000000
#define DEVICENAME "1" // DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
// DEVICENAME "2" -> Serial2
// DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE 1                 // Value for enabling the torque
#define TORQUE_DISABLE 0                // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE 0    // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE 1000 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD 20  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

//Faster movement, less oscillation:
//const float freq = 2;
//const int amplitude = 110;

//Stable but slower, more oscillation:
//const float freq = 0.5;
//const int amplitude = 130;

// Sensor Pins
//const int sensorPin1 = A1;  // Analog pin for sensor 1
//const int sensorPin2 = A2;  // Analog pin for sensor 2
//float Sensor_1= 786;
//float Sensor_2= 807;
//float value_1= 0;
//float value_2= 0;

const int sensorPin1 = A1;  // Analog pin for sensor 1
const int sensorPin2 = A2;  // Analog pin for sensor 2
const int sensorPin3 = A3;  // Analog pin for sensor 3
const int sensorPin4 = A4;  // Analog pin for sensor 4
const int sensorPin5 = A5;  // Analog pin for sensor 5
const int sensorPin6 = A6;  // Analog pin for sensor 6

// Center Position
float center_pos = 512;
float center_pos_head = 512;
float center_pos_head_ideal = 512;
float diff;
float new_pos;
float neuron1;
float neuron2;

//Optimal for now:
const float freq = 1;
const int amplitude = 130;
//

int16_t dxl_angle_cw = 256;
int16_t dxl_angle_ccw = 767;


float decay = 1;
float wl_turn = 30;
float wr_turn = 30;
float wl_base = 60;
float wr_base = 60;

float w_l = wl_base;
float w_r = wr_base;

bool increaseWeights = false;
float new_wl;
float new_wr;


// Create PortHandler instance
dynamixel::PortHandler *portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler *packetHandler;

//***********Set Global Variables****************
int goalPosition = 0;
int isMoving = 0;
int dxl_comm_result = COMM_TX_FAIL; // Communication result
uint8_t dxl_error = 0;              // Dynamixel error
int16_t dxl_present_position = 0;   // Present position

uint8_t dxl_new_id = DXL_NEW_ID;

unsigned long int myTime;
unsigned long int newTime;
unsigned int mydelay = 10; // ms

const unsigned int BUFFER_SIZE = 30;
bool started = false;
bool initialized = false;
bool move_initialize = false;

bool right_sens = false;
bool left_sens = false;
bool narrow = false;
int counter = 48;

double contact_time = 0;

double MUT_INH [NOMBER_MI_NEURONS] [BUFFER_SIZE];
int buffer_pos [NOMBER_MI_NEURONS] = {0};


/******************************************************/
//struct MIneuron
/******************************************************/
struct MIneuron {
  double T       = 12;
  double b       = 2.5;
  double x_0     = 0;
  double a[6]   = {2.5, 2.5, 2.5, 0.0, 0.0, 0.0};

  double s_j     = 3;
  double y       = 0;
  double x       = x_0;
  double x_hat   = 0;

} mi_neuron[NOMBER_MI_NEURONS];
/******************************************************/
//int myMax(double a, double b) {
//  return (a > b) ? a : b;
//}
/******************************************************/
inline double  fun_x ( double x , double x_hat, double s_j , double b, int curr_neuron)
{
  double part_sigm = 0;
  int a_Size = NOMBER_MI_NEURONS - 1;
  int index;
  int j = curr_neuron;

  for (int i = 0; i < a_Size ; i++)
  {
    if (curr_neuron + i == a_Size) {
      j = 0;
    }
    else
      j += 1;
    part_sigm += (mi_neuron[curr_neuron].a[i]) * (mi_neuron[j].y);

  }

  return (double)(-1 * part_sigm + s_j - b * x_hat - x);
}
/******************************************************/
inline double  fun_xhat ( double x , double x_hat, double y , double T)
{

  return (double)(1 / T) * (y - x_hat);
}
/******************************************************/

void update_MI_neuron(struct MIneuron* mi_n_1, int curr_neuron) {

  int n = 20;
  double h;
  h = ((double)mydelay / 120) / n; // 150 for 2 or 3 neurons
  double x_1[20], xhat_1[20];
  double k1, k2, k3, k4, k, l1, l2, l3, l4, l;

  x_1[0] = mi_n_1->x;
  xhat_1[0] = mi_n_1->x_hat;

  for (int i = 0; i < n - 1 ; i++)
  {
    l1 = h * fun_xhat( x_1[i] , xhat_1[i]  , mi_n_1->y, mi_n_1->T );
    k1 = h * fun_x( x_1[i] , xhat_1[i],  mi_n_1->s_j, mi_n_1->b, curr_neuron);

    l2 = h * fun_xhat( x_1[i] + 2 * k1 / 3, xhat_1[i] + 2 * l1 / 3, mi_n_1->y, mi_n_1->T );
    k2 = h * fun_x( x_1[i] + 2 * k1 / 3, xhat_1[i] + 2 * l1 / 3, mi_n_1->s_j, mi_n_1->b, curr_neuron);

    l = 1 / 4.0 * (l1 + 3 * l2);
    k = 1 / 4.0 * (k1 + 3 * k2);

    x_1[i + 1] = x_1[i] + k;
    xhat_1[i + 1] = xhat_1[i] + l ;
  }
  mi_n_1->x     = x_1[n - 1];
  mi_n_1->x_hat = xhat_1[n - 1];

  return;
}
/******************************************************/
void update_y_vals(struct MIneuron* mi_n)
{
  if ((mi_n->x < 0))
    mi_n->y = 0;
  else
    mi_n->y = mi_n->x;
}
/******************************************************/
void update_locomotion_network(void)
{
  for (int i = 0; i < NOMBER_MI_NEURONS ; i++)
  {
    update_y_vals(&mi_neuron[i]);
    update_MI_neuron(&mi_neuron[i], i);
  }
}

/******************************************************/
/* put your setup code in setup(), to run once */
//void setup() {
//  Serial.begin(115200);
//  delay(2000);
//}
void setup()
{
  // Setup analog inputs for sensors
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);
  pinMode(sensorPin5, INPUT);
  pinMode(sensorPin6, INPUT);

  // put your setup code here, to run once:
  //pinMode(sensorPin1, INPUT);
  //pinMode(sensorPin2, INPUT);

  //Serial.begin(115200);
  //while(!Serial.available())
  //;

  // Initialize portHandler. Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize packetHandler. Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  for (int i = 0; i < NOMBER_MI_NEURONS ; i++)  {
    packetHandler->write2ByteTxRx(portHandler, i + 1, ANGLE_CW_LIMIT, dxl_angle_cw, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, i + 1, ANGLE_CCW_LIMIT, dxl_angle_ccw, &dxl_error);
  }
}


/******************************************************/
/* put your main code here in loop(), to run repeatedly */
void loop() {

  // Read sensors, 1 is rear left, 2 is rear right, 5 is front left, 6 is front right
//  float value1 = analogRead(sensorPin1);//*1000/Sensor_1;
//  float value2 = analogRead(sensorPin2);//*1000/Sensor_2;
  // float value3 = analogRead(sensorPin3);//*1000/Sensor_3;
  // float value4 = analogRead(sensorPin4);//*1000/Sensor_4;
  float value5 = analogRead(sensorPin5);//*1000/Sensor_5;
  float value6 = analogRead(sensorPin6);//*1000/Sensor_6;


  /* Read my program running time in milliseconds */
  myTime = millis();

  // Give a step current

  //Serial.println(myTime);

  // Giving inputs to the neurons
  for (int i = 0; i < NOMBER_MI_NEURONS; i++) {
    mi_neuron[i].s_j = 3;
  }

  /* Update the neurons output*/
  update_locomotion_network();

  // Create anti-phasic oscillations
  for (int i = 0; i < NOMBER_MI_NEURONS ; i++)  {
    MUT_INH [i] [buffer_pos[i]] = mi_neuron[i].y;
    buffer_pos [i] = (buffer_pos [i] + 1) % BUFFER_SIZE;


    if (initialized == false) {
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i + 1, ADDR_AX_GOAL_POSITION, center_pos, &dxl_error);
    }
  }

  //To make the snake straight in the beginning
  if (initialized == false) {
    delay(2000);
  }
  initialized = true;

  // To gradually increase the weights up to the preset of 50
  //  if (myTime >= 10000 && w_l < 50){
  //      w_l+=0.2;
  //      w_r+=0.2;
  //  }

  // Sensors reach a threshold (rising edge), either of them raises the counter and will drop as soon released (falling edge)
  // Countdown will restart if a sensor was pressed again
  double value5_base = 910;
  double value5_max = 960;
  double value6_base = 850;
  double value6_min = 915;
  double range = 45;

  // If either side is pressed, start the countdown (length of a whole cycle) when sensor is released (falling edge)
  if (left_sens || right_sens) {
  w_r = wr_turn;
  w_l = wl_turn;
  increaseWeights = false;

  if (myTime - contact_time >= 3000) {
    // w_r = wr_base;
    // w_l = wl_base;
    increaseWeights = true;
    center_pos_head_ideal = 512;
    left_sens = false;
    right_sens = false;
  }
}

if (myTime - contact_time >= 150) {
  if (abs((value5 - value5_base) / (value5_max - value5_base)) > 0.6) {
    left_sens = true;
    center_pos_head_ideal = center_pos_head_ideal + (abs((value5 - value5_base) / (value5_max - value5_base)) * range - abs((value6 - value6_base) / (value6_base - value6_min)) * range);
    if (center_pos_head_ideal > 662)
    {
      center_pos_head_ideal = 662;
    }

    contact_time = myTime;
    Serial.print("Left Sensor Touched    ");
    Serial.println(center_pos_head_ideal);
  }

  if (abs((value6 - value6_base) / (value6_base - value6_min)) > 0.6) {
    right_sens = true;
    center_pos_head_ideal = center_pos_head_ideal + (abs((value5 - value5_base) / (value5_max - value5_base)) * range - abs((value6 - value6_base) / (value6_base - value6_min)) * range);
    if (center_pos_head_ideal < 362)
    {
      center_pos_head_ideal = 362;
    }
    contact_time = myTime;
    Serial.print("Right Sensor Touched    ");
    Serial.println(center_pos_head_ideal);
  }
}

if (increaseWeights){

      if (w_l != wl_base) {
        // Get the difference between current and desired weights
        diff = wl_base - w_l;

        // Calculate the new weight after current iteration
        new_wl = w_l + diff * 0.2;

        // If the new weight is larger than the desired weight
        if (new_wl > wl_base) {
          // Set the desired weight
          new_wl = wl_base;
        }
        w_l = new_wl;
      }

      if (w_r != wr_base) {
        // Get the difference between current and desired weights
        diff = wr_base - w_r;

        // Calculate the new weight after current iteration
        new_wr = w_r + diff * 0.2;

        // If the new weight is larger than the desired weight
        if (new_wr > wr_base) {
          // Set the desired weight
          new_wr = wr_base;
        }
        w_r = new_wr;
      }

}

if (myTime >= 10000) {

  for (int i = 1; i <= NOMBER_MI_NEURONS ; i++)  {
    //  Maybe this would work as (1 - x) since x is most of the times close to 0 --- x=abs((value6 - 810)/(810-643))
    //    w_r = abs((value6 - 810)/(810-643))*50;
    //    w_l = abs((value5 - 888)/(945-888))*50;

    neuron1 = mi_neuron[i - 1].y;
    neuron2 = MUT_INH[i - 1][(buffer_pos[i - 1] - 21 + BUFFER_SIZE) % BUFFER_SIZE];   //24

    // For the first motors, we set the direction, so baseline (from the oscillatory movements) will move
    if (i == 1 || i == 2) {

      if (center_pos_head != center_pos_head_ideal) {
        // Get the difference between current and desired positions
        diff = center_pos_head_ideal - center_pos_head;

        // Calculate the new position after current iteration
        new_pos = center_pos_head + diff * 0.1;

        // If the new position is inside of the change range
        if ((diff > 0 && new_pos > center_pos_head_ideal) || (diff < 0 && new_pos < center_pos_head_ideal)) {
          // Set the new position
          new_pos = center_pos_head_ideal;
        }
        center_pos_head = new_pos;
      }
      goalPosition = center_pos_head - (w_l + 5 * i) * neuron1 + (w_r + 5 * i) * neuron2;
    }

    else {
      center_pos = 512;
      goalPosition = center_pos - (w_l + 5 * i) * neuron1 + (w_r + 5 * i) * neuron2;
    }




    if (i == 1 && goalPosition < 520 && goalPosition > 504 && move_initialize == false) {
      move_initialize = true;
    }
    if (move_initialize) {
      packetHandler->read1ByteTxRx(portHandler, i, MOVING, (uint8_t *)&isMoving, &dxl_error);
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
      packetHandler->read2ByteTxRx(portHandler, i, ADDR_AX_PRESENT_POSITION, (uint16_t *)&dxl_present_position, &dxl_error);
    }
  }

}
// 220 ms
//delay(200);
/* delay at the end */
//  delay(mydelay);

}
