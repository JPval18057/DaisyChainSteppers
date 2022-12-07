//PID Variables
double kp, kd, ki; //si no se permiten inicializar en valores constantes ponerle los valores en el setup como PID_CONFIG();
double err,err_prev,err_ki,err_kd; ///verificar que se inicializen en 0
double uk;


//prototype
int PID(double kp, double kd, double ki, double ref, double pos_motor);
void PID_CONFIG(void);
//ref is the desired position
//pos_motor is the current position

void setup() {
  // put your setup code here, to run once:
  PID_CONFIG();
}

void loop() {
  // put your main code here, to run repeatedly:

}

int PID(double kp, double kd, double ki, double ref, double pos_motor){
    err = ref - pos_motor;
    err_kd = err - err_prev;
    err_ki = err + err_ki;
    uk = kp*err + kd*err_kd + ki*err_ki;
    //update the previous error
    err_prev = err;
    
    //convert uk to integrer or use map function to output a coherent speed/position result.
    int result = (int) uk;
    return result;
    //Try using the motor speed to control the motor and if it stops at a speed of steps/sec
    //then that is where this controller goes.
}

void PID_CONFIG(void){
  //initial values of constants
  kp = 1; ki = 0; kd = 0;
  //initial values of error
  err_prev = 0; err_kd = 0; err_ki = 0;
}
