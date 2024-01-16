#define NOMBER_MI_NEURONS 20
unsigned long int myTime;
unsigned long int newTime;
unsigned int mydelay = 10; // ms

/******************************************************/ 
//struct MIneuron 
/******************************************************/ 
struct MIneuron {  
   double T       = 12;
   double b       = 2.5;
   double x_0     = 0;
   double a[19]   ={2.5, 2.5, 2.5, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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
  int a_Size = NOMBER_MI_NEURONS-1;
  int index;
  int j = curr_neuron;
  
  for (int i = 0; i< a_Size ; i++)
  {
    if (curr_neuron + i == a_Size){
      j = 0;
    }
    else
      j += 1;
    part_sigm += (mi_neuron[curr_neuron].a[i]) * (mi_neuron[j].y);
       
  }
    
  return (double)(-1*part_sigm+s_j-b*x_hat-x); } 
/******************************************************/
inline double  fun_xhat ( double x , double x_hat, double y , double T)
{ 
  
  return (double)(1/T)*(y-x_hat); } 
/******************************************************/ 

void update_MI_neuron(struct MIneuron* mi_n_1, int curr_neuron){

  int n = 20; 
  double h; 
  h= ((double)mydelay/200)/n;   // 200 for 2 or 3 neurons
  double x_1[20],xhat_1[20]; 
  double k1,k2,k3,k4,k, l1,l2,l3,l4,l; 

  x_1[0] = mi_n_1->x;
  xhat_1[0] = mi_n_1->x_hat;

  for (int i=0; i<n-1 ; i++)
  {
    l1 = h*fun_xhat( x_1[i] , xhat_1[i]  , mi_n_1->y, mi_n_1->T );
    k1= h*fun_x( x_1[i] , xhat_1[i],  mi_n_1->s_j, mi_n_1->b, curr_neuron);
    
    l2 = h*fun_xhat( x_1[i] +2*k1/3, xhat_1[i] +2*l1/3, mi_n_1->y, mi_n_1->T );
    k2= h*fun_x( x_1[i] +2*k1/3, xhat_1[i] +2*l1/3, mi_n_1->s_j, mi_n_1->b, curr_neuron);
    
    l = 1/4.0 * (l1 + 3*l2);
    k = 1/4.0 * (k1 + 3*k2);
   
    x_1[i+1]= x_1[i]+k;
    xhat_1[i+1]= xhat_1[i]+l ;
  } 
  mi_n_1->x     = x_1[n-1]; 
  mi_n_1->x_hat = xhat_1[n-1]; 
  
return; 
}
/******************************************************/ 
void update_y_vals(struct MIneuron* mi_n)
{  
  if((mi_n->x <0))
    mi_n->y = 0;
  else
    mi_n->y = mi_n->x;
}
/******************************************************/ 
void update_locomotion_network(void)
{
  for (int i = 0; i< NOMBER_MI_NEURONS ; i++)
  {
    update_y_vals(&mi_neuron[i]);
    update_MI_neuron(&mi_neuron[i],i);} 
}

/******************************************************/ 
/* put your setup code in setup(), to run once */
void setup() {
  Serial.begin(115200);
  delay(2000);
}

/******************************************************/ 
/* put your main code here in loop(), to run repeatedly */
void loop() {
  
  /* Read my program running time in milliseconds */
  myTime = millis();
  
  // Give a step current
  
  if((myTime>3000))   { 
    for (int i=0; i<NOMBER_MI_NEURONS; i++){
    mi_neuron[i].s_j = 3;}}
  else {
    for (int i=0; i<NOMBER_MI_NEURONS; i++){
    mi_neuron[i].s_j = 0;}}
  
  
  /* Update the neurons output*/
  update_locomotion_network();

  // for (int i = 0; i< NOMBER_MI_NEURONS ; i++)  {    
  //   Serial.print(mi_neuron[i].y);Serial.print(",");}
  // Serial.print("\n");

  newTime = millis();

  Serial.print("Time to obtain a set of values: ");Serial.print(newTime-myTime);
  Serial.print("\n");

  // 220 ms
  
  /* delay at the end */
  delay(mydelay);

}
