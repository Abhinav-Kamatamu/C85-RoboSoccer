/***************************************************
 CSC C85 RoboSoccer; tunable constants

 Every tunable and hardware constant lives here

 NUMBERS BELOW ARE PLACEHOLDERS, not TRUE values.
***************************************************/

#ifndef ROBOSOCCER_AI_CONFIG_H
#define ROBOSOCCER_AI_CONFIG_H

// **** hardware ****
#define WHEEL_TRACK_CM        12.8 
#define POWER_TO_CMPS         0.38
#define POWER_TO_RADPS        0.060
#define MIN_MOVE_POWER        15

#define MOTOR_TRIM_L          0
#define MOTOR_TRIM_R          0
#define MAX_POWER             100
#define MAX_POWER_STEP_PER_FRAME 25

// **** perception: pixel -> cm (bench test 1 , NOT uniform in x/y) ****
#define PX_PER_CM_X           7.5
#define PX_PER_CM_Y           6.3

// **** field (defaults only; field.cfg loading is a later TODO, we need an actual board repr model FIRST ) ****
#define DEFAULT_FIELD_W_CM    170.0
#define DEFAULT_FIELD_H_CM    115.0
#define DEFAULT_GOAL_WIDTH_CM 40.0
#define DEFAULT_WALL_MARGIN_CM 12.0

// **** vision / lost handling ****
#define LOST_FRAMES           6
#define BALL_LOST_FRAMES      12
#define READY_FRAMES          5

// **** staging / align / strike ****
#define STAGE_TOL_CM          4.0  
#define MOVE_TOL_CM           5.0  
#define ALIGN_TOL_DEG         4.0  
#define LAT_TOL_CM            2.0  
#define ALIGN_STABLE_FRAMES   5    
#define ALIGN_TIMEOUT_S       4.0  
#define ABORT_TOL_DEG         20.0 
#define STRIKE_TIMEOUT_S      3.0  
#define STRIKE_V_CMPS         30.0 
#define STRIKE_V_MIN_CMPS     8.0  
#define FT_DIST_CM            15.0 
#define FT_T_S                1.0  
#define FT_V_CMPS             20.0 
#define REPO_DIST_CM          15.0 
#define REPO_V_CMPS           -20.0
#define RETRY_WAIT_S          2.0  

// **** wall / unstick ****
#define WALL_RECOVER_V_CMPS   20.0
#define UNSTICK_STALL_S       1.5 
#define UNSTICK_REVERSE_S     0.4 
#define UNSTICK_TURN_S        0.4
#define UNSTICK_V_CMPS        -20.0
#define UNSTICK_W_RADPS       1.5

// **** control gains (proportional: error -> commanded turn rate) ****
#define HEADING_TURN_GAIN     2.0 
#define LATERAL_TURN_GAIN     0.05
#define GOTO_V_CMPS           25.0
#define GOTO_SLOWDOWN_CM      10.0
#define MAX_W_RADPS           3.0 
#define CLOSE_AHEAD_CM        20.0
#define STAGE_DIST_CM         20.0
#define STALL_EPS_CMPS        2.0 
#define MIN_WALL_STAGE_CM     6.0 
                            

#endif
