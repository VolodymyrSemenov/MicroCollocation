// Constants
const double pi = 3.141592653;    
const double g = 9.81;           

// Robot Parameters
const double lz = 0.17;            
const double kz = 1699;            
const double b = 5.0;             
const double m = 1.2;           

// Motor Parameters
const double vmax = 18;
const double Ra = 0.135;
const double R = 33.0625; 
const double kt = 0.0098;        
const double kb = kt;             
const double c = 0.0000016; 
const double J = 0.00000183;       
const double La = 0.0000166; 

// Initial Condition
extern double leg;  
extern double legDot;    
extern double theta;      
extern double thetaDot;   
extern double iaz;  