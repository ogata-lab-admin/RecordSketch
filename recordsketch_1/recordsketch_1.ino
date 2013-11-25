/**
 *
 * analogIn.ino
 * RTno is RT-middleware and arduino.
 *
 * Using RTno, arduino device can communicate any RT-components 
 *  through the RTno-proxy component which is launched in PC.
 * Connect arduino with USB, and program with RTno library.
 * You do not have to define any protocols to establish communication
 *  between arduino and PC.
 *
 * Using RTno, you must not define the function "setup" and "loop".
 * Those functions are automatically defined in the RTno libarary.
 * You, developers, must define following functions:
 *  int onInitialize(void);
 *  int onActivated(void);
 *  int onDeactivated(void);
 *  int onExecute(void);
 *  int onError(void);
 *  int onReset(void);
 * These functions are spontaneously called by the RTno-proxy
 *  RT-component which is launched in the PC.
 */
#include <UART.h>
#include <RTno.h>
#include <MsTimer2.h>
#include <Math.h>

//setting pin numbers
#define PIN_SW (8)
#define PINA_A (9)  //for plotter
#define PINA_B (10) //for plotter
#define PINB_A (11) //for record
#define PINB_B (12) //for record
#define PIN_SOLENOID (13)
#define STATE (3)

//fixed number
int plotScope = 100; //movable scope of plotter [step] 
int c_timer =  500; //compensation timer[ms]

//variant
int stepState = 1;
int currentPos = 0;
int targetPos = 0;

float m_targetStep = 0.0;
int m_rot = 0;
int m_sol = 0;
int m_gain = 10;
int m_p_speed = 50;
int m_c_speed = 50;


boolean m_state[STATE+1];


void stepBackward(){
//plotter rotatate 1 step forward
	if(stepState == 4){
		digitalWrite(PINA_A, HIGH);
		digitalWrite(PINA_B, LOW);
		stepState = 3;
	}else if(stepState == 3){
		digitalWrite(PINA_A, HIGH);
		digitalWrite(PINA_B, HIGH);
		stepState = 2;	
        }else if(stepState == 2){
		digitalWrite(PINA_A, LOW);
		digitalWrite(PINA_B, HIGH);
		stepState = 1;
        }else{
		digitalWrite(PINA_A, LOW);
		digitalWrite(PINA_B, LOW);
		stepState = 4;
         }
        currentPos++;		
}

void stepForward(){
//plotter rotatate 1 step backward
	if(stepState == 1){
		digitalWrite(PINA_A, LOW);
		digitalWrite(PINA_B, HIGH);
		stepState = 2;
	}else if(stepState == 2){
		digitalWrite(PINA_A, HIGH);
		digitalWrite(PINA_B, HIGH);
		stepState = 3;	
        }else if(stepState == 3){
		digitalWrite(PINA_A, HIGH);
		digitalWrite(PINA_B, LOW);
		stepState = 4;
        }else{
		digitalWrite(PINA_A, LOW);
		digitalWrite(PINA_B, LOW);
		stepState = 1;
        }	
        currentPos--;	
}

void compensation(){
        //turn record
        for(int i = 0; i <=8 ; i++){
    	  digitalWrite(PINB_A, LOW);
  	  digitalWrite(PINB_B, HIGH);
  	  delay(10);
  	  // P1:off, P2:ON, P1B:ON, P2B:off
  	  digitalWrite(PINB_A, HIGH);
  	  digitalWrite(PINB_B, HIGH);
  	  delay(10);
  	  // P1:off, P2:off, P1B:ON, P2B:ON
  	  digitalWrite(PINB_A, HIGH);
  	  digitalWrite(PINB_B, LOW);
  	  delay(10);
  	  // P1:ON, P2:off, P1B:off, P2B:ON
  	  digitalWrite(PINB_A, LOW);
  	  digitalWrite(PINB_B, LOW);
  	  delay(10);        
        }
  
	int m_div  = abs(targetPos - currentPos);
	
        if (abs(currentPos) + m_div >= plotScope/2){
		m_div = plotScope/2 - abs(currentPos);
	}
        if (m_div > 10){
		if ( targetPos  > currentPos){
			for(int i = 0; i <=m_div; i ++){
				stepForward();
                                delay(m_c_speed);
			}
		}else{
			for(int i  = 0; i <= m_div; i++){
				stepBackward();
                                delay(m_c_speed);
			}
		}
        }	
}



/**
 * This function is called at first.
 * conf._default.baudrate: baudrate of serial communication
 * exec_cxt.periodic.type: reserved but not used.
 */
void rtcconf(config_str& conf, exec_cxt_str& exec_cxt) {
  conf._default.connection_type = ConnectionTypeSerial1;
  conf._default.baudrate = 57600;
  exec_cxt.periodic.type = ProxySynchronousExecutionContext;
}

/** 
 * Declaration Division:
 *
 * DataPort and Data Buffer should be placed here.
 *
 * Currently, following 6 types are available.
 * TimedLong:
 * TimedDouble:
 * TimedFloat:
 * TimedLongSeq:
 * TimedDoubleSeq:
 * TimedFloatSeq:
 *
 * Please refer following comments. If you need to use some ports,
 * uncomment the line you want to declare.
 **/
 
TimedDouble targetStep;
InPort<TimedDouble> targetStepIn("targetStep", targetStep);
TimedLong rot;
InPort<TimedLong> rotIn("rot", rot);
TimedLong sol;
InPort<TimedLong> solIn("sol", sol);
TimedLong gain;
InPort<TimedLong> gainIn("gain", gain);
TimedLong p_speed;
InPort<TimedLong> p_speedIn("p_speed", p_speed);
TimedLong c_speed;
InPort<TimedLong> c_speedIn("c_speed", c_speed);
TimedBooleanSeq state;
OutPort<TimedBooleanSeq> stateOut("state", state);

//////////////////////////////////////////
// on_initialize
//
// This function is called in the initialization
// sequence. The sequence is triggered by the
// PC. When the RTnoRTC is launched in the PC,
// then, this function is remotely called
// through the USB cable.
// In on_initialize, usually DataPorts are added.
//
//////////////////////////////////////////
int RTno::onInitialize() {
  /* Data Ports are added in this section.
  */
  addInPort(targetStepIn);
  addInPort(rotIn);
  addInPort(solIn);
  addInPort(gainIn);
  addInPort(p_speedIn);
  addInPort(c_speedIn);
  addOutPort(stateOut);
  
  // Some initialization (like port direction setting)
  //setting timer
  MsTimer2::set(c_timer, compensation);
  //setting pinMode
  pinMode(PIN_SW, INPUT);
  pinMode(PINA_A, OUTPUT);
  pinMode(PINA_B, OUTPUT);
  pinMode(PINB_A, OUTPUT);
  pinMode(PINB_B, OUTPUT);
  pinMode(PIN_SOLENOID, OUTPUT);

  return RTC_OK; 
}

////////////////////////////////////////////
// on_activated
// This function is called when the RTnoRTC
// is activated. When the activation, the RTnoRTC
// sends message to call this function remotely.
// If this function is failed (return value 
// is RTC_ERROR), RTno will enter ERROR condition.
////////////////////////////////////////////
int RTno::onActivated() {
  // Write here initialization code.
  //initialize plotter position and reset position-counter
	while ( digitalRead(PIN_SW) == LOW ) {
		stepBackward();
                delay(100);
	}
	
	for (int i =  0; i <= plotScope/2 ; i ++){
		stepForward();
                delay(100);
        }
        currentPos = 0;
        targetPos = 0;	
	
	//start compensation <- timer
	MsTimer2::start();
	
	//set state
	for (int i = 1; i <= STATE ; i++){
		m_state[i]  = false;
	}
	m_state[0] = true;

  return RTC_OK; 
}

/////////////////////////////////////////////
// on_deactivated
// This function is called when the RTnoRTC
// is deactivated.
/////////////////////////////////////////////
int RTno::onDeactivated()
{
  // Write here finalization code.
  MsTimer2::stop();
  return RTC_OK;
}

//////////////////////////////////////////////
// This function is repeatedly called when the 
// RTno is in the ACTIVE condition.
// If this function is failed (return value is
// RTC_ERROR), RTno immediately enter into the 
// ERROR condition.r
//////////////////////////////////////////////
int RTno::onExecute() {
    //read data from dataPorts
    if(targetStepIn.isNew()) {
        targetStepIn.read();
        m_targetStep = targetStep.data;
    }
    if(rotIn.isNew()){
        rotIn.read();
        m_rot = (int)rot.data;
    }
    if(solIn.isNew()){
        solIn.read();
        m_sol = (int)sol.data;
    }
    if(gainIn.isNew()){
        gainIn.read();
        m_gain = (int)gain.data;
    }
    if(p_speedIn.isNew()){
        p_speedIn.read();
        m_p_speed = (int)p_speed.data;
    }
    if(c_speedIn.isNew()){
        c_speedIn.read();
        m_c_speed = (int)c_speed.data;
    }
    
    //move plotter
    if(m_targetStep > 0){
         if(currentPos + m_targetStep * m_gain > plotScope/2){
             m_state[1] = true;
             m_targetStep = m_targetStep - (currentPos + m_targetStep * m_gain - plotScope/2)/m_gain;
         }
         for(int i = 0; i < (int)(m_targetStep * m_gain); i++){
             stepForward();
             delay(m_p_speed);
         }
    }else if(m_targetStep < 0){
         if(abs(currentPos + m_targetStep * m_gain) > plotScope/2){
             m_state[2] = true;
             m_targetStep = m_targetStep + (abs(currentPos + m_targetStep * m_gain) - plotScope/2)/m_gain;
         }
         for(int i = 0; i < (int)(m_targetStep * -1 * m_gain); i++){
             stepBackward();
             delay(m_p_speed);
        
         }  
    }
    //on/off solenoid
    if(m_sol > 0){
        digitalWrite(PIN_SOLENOID, HIGH);
    }
    
    //write state for dataPorts
    state.data.length(4);
    for(int i = 0;i < 4; i++){
        state.data[i] = m_state[i];
    }
    stateOut.write();

    
  return RTC_OK; 
}


//////////////////////////////////////
// on_error
// This function is repeatedly called when
// the RTno is in the ERROR condition.
// The ERROR condition can be recovered,
// when the RTno is reset.
///////////////////////////////////////
int RTno::onError()
{
  return RTC_OK;
}

////////////////////////////////////////
// This function is called when 
// the RTno is reset. If on_reset is
// succeeded, the RTno will enter into
// the INACTIVE condition. If failed 
// (return value is RTC_ERROR), RTno
// will stay in ERROR condition.ec
///////////////////////////////////////
int RTno::onReset()
{
  return RTC_OK;
}


