#include <stdio.h>
#include <stdbool.h>
#include "MACROS.h"


#include "FlightStateDetector.h"


#define TIME_ELAPSED(start_ms, now_ms, wait_ms) (start_ms <= (now_ms - wait_ms))

//----- Our defines ----------
#define APO 			0
#define MAIN 			1
#define SECOND_STAGE	2
#define AUX 			3
 
//----- Private functions ----------
static void FlightState_STARTUP					(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_PREFLIGHT				(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_BOOST					(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_SECOND_STAGE_DELAY		(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_SECOND_STAGE_IGNITION	(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_SECOND_STAGE_BOOST		(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_FREEFLIGHT				(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_FREEFALL				(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_DRAGCHUTE_FALL			(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_DRAGCHUTE_FAILURE		(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_MAINSHUTE_FALL			(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_RECOVERY_FAILURE		(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);
static void FlightState_LANDING					(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs);

//----- Private variables ----------
static FlightState_t flightState_d;
static AHRS_t * 	 AHRS_ptr;
static FSD_settings_t FSD_settings_d;
static const char *TAG = "FSD";

static uint64_t stateChangeTime = 0;
static armingstatus_t armstatus_d = DISARMED;

void FSD_arming(){
	armstatus_d = ARMED;
}

void FSD_disarming(){
	armstatus_d = DISARMED;
}

armingstatus_t FSD_checkArmed(){
	return armstatus_d;
}

flightstate_t FSD_getState(){
	return flightState_d.state;
}

void FSD_forceState(flightstate_t new_state){
	flightState_d.state = new_state;
}

esp_err_t FSD_init(AHRS_t * ahrs){
	if(ahrs == NULL){
		return ESP_FAIL;
	}

	
	
		FSD_settings_d.drouge_alt 		= 0;
		FSD_settings_d.main_alt 		= 150;
		FSD_settings_d.max_tilt 		= 10;
		FSD_settings_d.rail_height 		=4000;
		FSD_settings_d.staging_delay_ms	= 500;
		FSD_settings_d.staging_max_tilt = 10;
	

	AHRS_ptr = ahrs;

	flightState_d.state = FLIGHTSTATE_STARTUP;
	FSD_disarming();

	return ESP_OK;
}

esp_err_t FSD_detect(uint64_t time_ms){
	if(FSD_checkArmed() == DISARMED){
		flightState_d.state = FLIGHTSTATE_STARTUP;
		
		return ESP_OK;
	}

	FlightState_t * currentState = &flightState_d;
	AHRS_t * 		ahrs 		 = AHRS_ptr;

	switch(flightState_d.state){
	case FLIGHTSTATE_STARTUP:
		FlightState_STARTUP(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_PREFLIGHT:
		FlightState_PREFLIGHT(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_BOOST:
		FlightState_BOOST(time_ms, currentState, ahrs);
		break;
	
	case FLIGHTSTATE_SECOND_STAGE_DELAY:
		FlightState_SECOND_STAGE_DELAY(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_SECOND_STAGE_IGNITION:
		FlightState_SECOND_STAGE_IGNITION(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_SECOND_STAGE_BOOST:
		FlightState_SECOND_STAGE_BOOST(time_ms, currentState, ahrs);
		break;
	
	case FLIGHTSTATE_FREEFLIGHT:
		FlightState_FREEFLIGHT(time_ms, currentState, ahrs);
		break;
	
	case FLIGHTSTATE_FREEFALL:
		FlightState_FREEFALL(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_DRAGCHUTE_FALL:
		FlightState_DRAGCHUTE_FALL(time_ms, currentState, ahrs);
		break;
	
	case FLIGHTSTATE_DRAGCHUTE_FAILURE:
		FlightState_DRAGCHUTE_FAILURE(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_MAINSHUTE_FALL:
		FlightState_MAINSHUTE_FALL(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_RECOVERY_FAILURE:
		FlightState_RECOVERY_FAILURE(time_ms, currentState, ahrs);
		break;

	case FLIGHTSTATE_LANDING:
		FlightState_LANDING(time_ms, currentState, ahrs);
		break;

	default:
		
		FSD_disarming();
	}

	return ESP_OK;
}


static void FlightState_STARTUP	(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop
	


	//State change conditions
	if (TIME_ELAPSED(stateChangeTime, time_ms, 500)) {
		currentState->state = FLIGHTSTATE_PREFLIGHT;
		currentState->state_ready = false;
	}
}

static void FlightState_PREFLIGHT (uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop
	

	//State change conditions
	if ((TIME_ELAPSED(stateChangeTime, time_ms, 100)) && (ahrs->acc_axis_lowpass >= (1.6f * 9.81f)) ) {

		flightState_d.ignition_time_ms = time_ms;

		currentState->state = FLIGHTSTATE_BOOST;
		currentState->state_ready = false;
	}
}

static void FlightState_BOOST(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;

		

	}

	//Executed every loop


	//State change conditions
	if((TIME_ELAPSED(stateChangeTime, time_ms, 200))  && (ahrs->acc_axis_lowpass < 0.0f) ) {
			currentState->state = FLIGHTSTATE_SECOND_STAGE_DELAY;
			currentState->state_ready = false;
	}
}

static void FlightState_SECOND_STAGE_DELAY(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
    //Executed only once
    if(!(currentState->state_ready)) {
        currentState->state_ready = true;
        stateChangeTime = time_ms;
    }

    //Executed every loop


    //State change conditions (second stage ignition)
    if((TIME_ELAPSED(stateChangeTime, time_ms, FSD_settings_d.staging_delay_ms))) { 

       

        currentState->state = FLIGHTSTATE_SECOND_STAGE_IGNITION;
        currentState->state_ready = false;
    }

    //State change conditions (altitude)
    if ((TIME_ELAPSED(stateChangeTime, time_ms, 200))  && ((ahrs->max_altitude - ahrs->altitudeP) > 10) ) {
        currentState->state = FLIGHTSTATE_FREEFALL;
        currentState->state_ready = false;
    }
}

static void FlightState_SECOND_STAGE_IGNITION(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
    //Executed only once
    if(!(currentState->state_ready)) {
        currentState->state_ready = true;
        stateChangeTime = time_ms;
    }

    //Executed every loop


    //State change conditions (second stage burn)
    if((TIME_ELAPSED(stateChangeTime, time_ms, 100)) && (ahrs->acc_axis_lowpass >= (1.6f * 9.81f)) ) { 
        currentState->state = FLIGHTSTATE_SECOND_STAGE_BOOST;
        currentState->state_ready = false;
    }

    //State change conditions (second stage ignition failed)
    if(TIME_ELAPSED(stateChangeTime, time_ms, 3000))
    {
        currentState->state = FLIGHTSTATE_FREEFLIGHT;
        currentState->state_ready = false;
    }

    //State change conditions (altitude)
    if ((TIME_ELAPSED(stateChangeTime, time_ms, 200))  && ((ahrs->max_altitude - ahrs->altitudeP) > 10) ) {
        currentState->state = FLIGHTSTATE_FREEFALL;
        currentState->state_ready = false;
    }
}

static void FlightState_SECOND_STAGE_BOOST(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop


	//State change conditions // do poprawy
	if ((TIME_ELAPSED(stateChangeTime, time_ms, 200))  && (ahrs->acc_axis_lowpass < 0.0f) ) {
			currentState->state = FLIGHTSTATE_FREEFLIGHT;
			currentState->state_ready = false;
	}
}

static void FlightState_FREEFLIGHT(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop


	//State change conditions
	if ((TIME_ELAPSED(stateChangeTime, time_ms, 200))  && ((ahrs->max_altitude - ahrs->altitudeP) > 10.0f) ) {
			currentState->state = FLIGHTSTATE_FREEFALL;
			currentState->state_ready = false;
	}
}

static void FlightState_FREEFALL (uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
    //Executed only once
    if(!(currentState->state_ready)) {
        currentState->state_ready = true;
        stateChangeTime = time_ms;
    }

    //Executed every loop


    //State change conditions
    if (/*(TIME_ELAPSED(stateChangeTime, time_ms, 100))*/ 1) {

      

        currentState->state = FLIGHTSTATE_DRAGCHUTE_FALL;
        currentState->state_ready = false;
    }
}

static void FlightState_DRAGCHUTE_FALL (uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop


	//State change conditions
	if ((TIME_ELAPSED(stateChangeTime, time_ms, 100) && (ahrs->altitudeP <= FSD_settings_d.main_alt))) {

	

		currentState->state = FLIGHTSTATE_MAINSHUTE_FALL;
		currentState->state_ready = false;
	}
	else if((TIME_ELAPSED(stateChangeTime, time_ms, 2000) && (ahrs->ascent_rate < -60.0f))){
		currentState->state = FLIGHTSTATE_DRAGCHUTE_FAILURE;
		currentState->state_ready = false;
	}
}

static void FlightState_DRAGCHUTE_FAILURE (uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop


	//State change conditions
	if(TIME_ELAPSED(stateChangeTime, time_ms, 100)){

		

		currentState->state = FLIGHTSTATE_MAINSHUTE_FALL;
		currentState->state_ready = false;
	}
}

static void FlightState_MAINSHUTE_FALL (uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop


	//State change conditions
	if(TIME_ELAPSED(stateChangeTime, time_ms, 5000) && (ahrs->ascent_rate < -60.0f) ){
		currentState->state = FLIGHTSTATE_RECOVERY_FAILURE;
		currentState->state_ready = false;
	}
	else if(TIME_ELAPSED(stateChangeTime, time_ms, 30000)  && (ahrs->altitudeP < 200.0f) && (ahrs->velocityP > -2.0f)){
		currentState->state = FLIGHTSTATE_LANDING;
		currentState->state_ready = false;
	}
}

static void FlightState_RECOVERY_FAILURE (uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}

	//Executed every loop

	//State change conditions
	if(TIME_ELAPSED(stateChangeTime, time_ms, 30000)  && (ahrs->altitudeP < 200.0f) && (ahrs->velocityP > -2.0f)){
		currentState->state = FLIGHTSTATE_LANDING;
		currentState->state_ready = false;
	}
}

static void FlightState_LANDING	(uint64_t time_ms, FlightState_t * currentState, AHRS_t * ahrs){
	//Executed only once
	if(!(currentState->state_ready)) {
		currentState->state_ready = true;
		stateChangeTime = time_ms;
	}


	//Executed every loop



	//State change conditions
	if (TIME_ELAPSED(stateChangeTime, time_ms, 30000)){
		currentState->state = FLIGHTSTATE_SHUTDOWN;
		currentState->state_ready = false;
	}
}


