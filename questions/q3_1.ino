#define NOMBER_MI_NEURONS 3
unsigned long int myTime;
unsigned int mydelay = 10; // ms
/******************************************************/ 
//struct MIneuron 
/******************************************************/ 
struct MIneuron {  
   double tao_m   = 1;   
   double T       = 12;
   double b       = 0;
   double x_0     = 0;
   double theta   = 0;
   double c_j     = 1;
   double s_j     = 0;
   double y       = 0;
   double q_0     = 0;
   double x       = x_0;
   double x_hat   = 0;
   double q       = q_0;
   
} mi_neuron[NOMBER_MI_NEURONS];
/******************************************************/ 
inline double  fun_x ( double x , double x_hat , double c_j , double s_j , double b, double tao )
{ return (double)(1/tao)*(c_j*s_j-b*x_hat-x); } 
/******************************************************/
inline double  fun_xhat ( double x , double x_hat, double theta , double T )
{ double y;
  if (x >= theta){
    y = x - theta;
  }
  else{
    y=0;
  }

return (double)(1/T)*(y-x_hat); } 
/******************************************************/ 

void update_MI_neuron(struct MIneuron* mi_n){

  int n = 20; 
  double h; 
  h= ((double)mydelay/1000)/n;
  double xa[20],ya[20],yb[20]; 
  double k1,k2,k3,k4,k, l1,l2,l3,l4,l; 

  ya[0] = mi_n->x;
  yb[0] = mi_n->x_hat;

for (int i=0; i<n-1 ; i++)
{
  l1 = h*fun_xhat( ya[i] , yb[i]  , mi_n->theta, mi_n->T );
  k1= h*fun_x( ya[i] , yb[i] , mi_n->c_j, mi_n->s_j, mi_n->b, mi_n->tao_m); 
  
  l2 = h*fun_xhat( ya[i] +k1/2, yb[i] +l1/2, mi_n->theta, mi_n->T );
  k2= h*fun_x( ya[i] +k1/2, yb[i] +l1/2 , mi_n->c_j, mi_n->s_j, mi_n->b, mi_n->tao_m); 
  
  l3 = h*fun_xhat( ya[i]+k2/2 , yb[i] +l2/2, mi_n->theta, mi_n->T );
  k3= h*fun_x( ya[i] +k2/2, yb[i] +l2/2 , mi_n->c_j, mi_n->s_j, mi_n->b, mi_n->tao_m);
   
  l4 = h*fun_xhat( ya[i]+k3 , yb[i] +l3, mi_n->theta, mi_n->T );
  k4= h*fun_x( ya[i] +k3, yb[i] +l3, mi_n->c_j, mi_n->s_j, mi_n->b, mi_n->tao_m); 
  
  l = 1/6.0 * (l1 + 2*l2 + 2*l3 + l4);
  k = 1/6.0 * (k1 + 2*k2 + 2*k3 + k4);
  
  
 
  ya[i+1]= ya[i]+k;
  yb[i+1]= yb[i]+l ;
} 
mi_n->x     = ya[n-1]; 
mi_n->x_hat = yb[n-1];  

return; 
}
/******************************************************/ 


void setup_MI_neurons(struct MIneuron* mi_n, double b)
{
  mi_n->b = b;
  mi_n->T = 12*b/2.5;
}
/******************************************************/ 
void update_locomotion_network(void)
{
for (int i = 0; i< NOMBER_MI_NEURONS ; i++){
  update_MI_neuron(&mi_neuron[i]);
  }

}

/******************************************************/ 
/* put your setup code in setup(), to run once */
void setup() {
delay(3000);

Serial.begin(115200);

//setup_MI_neurons(&mi_neuron[0], 0);
setup_MI_neurons(&mi_neuron[1], 2.5);
setup_MI_neurons(&mi_neuron[2], 1000);

}

/******************************************************/ 
/* put your main code here in loop(), to run repeatedly */
void loop() {



/* Read my program running time in milliseconds */
myTime = millis();

// Give a step current
if((myTime>3000))    
mi_neuron[0].s_j = 1;
else
mi_neuron[0].s_j = 0;

if((myTime>3000))    
mi_neuron[1].s_j = 1;
else
mi_neuron[1].s_j = 0;

if((myTime>3000))    
mi_neuron[2].s_j = 1;
else
mi_neuron[2].s_j = 0;


/* Update the neurons output*/
update_locomotion_network();


for (int i = 0; i< NOMBER_MI_NEURONS ; i++)
{Serial.print(mi_neuron[i].x);Serial.print(" ");}
Serial.print("\n");

/* delay at the end */
delay(mydelay);

}