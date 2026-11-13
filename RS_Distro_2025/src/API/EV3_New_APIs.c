/***********************************************************************************************************************
 *
 *      EV3_New_APIs.c - Every API our team added on top of the CSC C85 EV3 starter code, collected in one file.
 *
 *      Nothing in this file was part of the starter code. Each part is copied unchanged from the place it lives in
 *      our project, which is named in the banner above that part:
 *
 *        Part 1 - Extended motor API (20 BT_motor_* functions)        from EV3_RobotControl/btcomm.c
 *        Part 2 - Drive and gyro-turn helpers                         from EV3_Localization.c
 *        Part 3 - Colour sensor calibration and classification        from EV3_Localization.c
 *        Part 4 - Sensor rail positioning and colour sweeps           from EV3_Localization.c
 *        Part 5 - Windows Bluetooth transport (bt_win_*)              from EV3_RobotControl/bt_win.c
 *
 *      API_Documentation.md, next to this file, describes every function.
 *
 *      This file is a collection for reading and marking. The project itself builds from the original files (see the
 *      Makefile), and this file must not be added to that build: Part 1 would then be defined twice. The short
 *      blocks marked "Glue for this collection file" are the only lines that do not come from the project; they
 *      stand in for the surrounding code of the original files (includes, constants, the socket variable).
 *
 *      Part 5 uses <winsock2.h>, whose names clash with the Lego headers used by Parts 1 to 4, so in the project it
 *      is a separate source file. Here it is selected with -DEV3_NEW_APIS_BT_WIN, which compiles Part 5 alone.
 *
 * ********************************************************************************************************************/

#ifndef EV3_NEW_APIS_BT_WIN

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Glue for this collection file: includes, and the Bluetooth socket that btcomm.c owns.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "btcomm.h"

#ifdef _WIN32
// Same redirection btcomm.c does on Windows: socket I/O goes through the Part 5 wrappers.
int bt_win_write(const void *buf, int n);
int bt_win_read(void *buf, int n);
#define read(s,buf,n)   bt_win_read((buf),(int)(n))
#define write(s,buf,n)  bt_win_write((buf),(int)(n))
#else
extern int *socket_id;			// Defined in btcomm.c
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// New constant added to btcomm.h: port NUMBERS of the four motor ports (the existing MOTOR_A..MOTOR_D are bit masks).
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef MOTOR_NO_A
#define MOTOR_NO_A 0
#define MOTOR_NO_B 1
#define MOTOR_NO_C 2
#define MOTOR_NO_D 3
#endif

/**********************************************************************************************************************
 *
 *      PART 1 - Extended motor API                                              (from EV3_RobotControl/btcomm.c)
 *
 *********************************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Extended motor API: wrappers for the EV3 output opcodes in bytecodes.h.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static int BT_motor_valid_ports(char port_ids)
{
 if (port_ids<=0 || port_ids>0x0F)
 {
  fprintf(stderr,"Motor command: port mask must be in [1, 15]\n");
  return(0);
 }
 return(1);
}

// opOUTPUT_SET_TYPE, opOUTPUT_READ and opOUTPUT_GET_COUNT take a port *number*
// (A=0 ... D=3), unlike every other output opcode, which takes a bit mask.
static int BT_motor_valid_port_no(int port_no)
{
 if (port_no<0 || port_no>3)
 {
  fprintf(stderr,"Motor command: port number must be in [0, 3] (A=0, B=1, C=2, D=3)\n");
  return(0);
 }
 return(1);
}

static int BT_motor_port_count(char port_ids)
{
 int count=0;
 int bit;
 for (bit=0; bit<4; bit++)
  if (port_ids&(1<<bit)) count++;
 return(count);
}

static void BT_motor_append_lc0(unsigned char *payload, int *length, int value)
{
 payload[(*length)++]=(unsigned char)LC0(value);
}

static void BT_motor_append_lc1(unsigned char *payload, int *length, int value)
{
 payload[(*length)++]=(unsigned char)LC1_byte0();
 payload[(*length)++]=(unsigned char)value;
}

static void BT_motor_append_lc2(unsigned char *payload, int *length, int value)
{
 unsigned int encoded=(unsigned int)value;
 payload[(*length)++]=(unsigned char)LC2_byte0();
 payload[(*length)++]=(unsigned char)(encoded&0xFF);
 payload[(*length)++]=(unsigned char)((encoded>>8)&0xFF);
}

static void BT_motor_append_lc4(unsigned char *payload, int *length, int value)
{
 unsigned int encoded=(unsigned int)value;
 payload[(*length)++]=(unsigned char)(PRIMPAR_LONG|PRIMPAR_CONST|PRIMPAR_4_BYTES);
 payload[(*length)++]=(unsigned char)(encoded&0xFF);
 payload[(*length)++]=(unsigned char)((encoded>>8)&0xFF);
 payload[(*length)++]=(unsigned char)((encoded>>16)&0xFF);
 payload[(*length)++]=(unsigned char)((encoded>>24)&0xFF);
}

static void BT_motor_append_gv1(unsigned char *payload, int *length, int offset)
{
 payload[(*length)++]=(unsigned char)(PRIMPAR_LONG|PRIMPAR_VARIABEL|PRIMPAR_GLOBAL|PRIMPAR_1_BYTE);
 payload[(*length)++]=(unsigned char)(offset&0xFF);
}

static void BT_motor_append_gv4(unsigned char *payload, int *length, int offset)
{
 payload[(*length)++]=(unsigned char)(PRIMPAR_LONG|PRIMPAR_VARIABEL|PRIMPAR_GLOBAL|PRIMPAR_4_BYTES);
 payload[(*length)++]=(unsigned char)(offset&0xFF);
 payload[(*length)++]=(unsigned char)((offset>>8)&0xFF);
 payload[(*length)++]=(unsigned char)((offset>>16)&0xFF);
 payload[(*length)++]=(unsigned char)((offset>>24)&0xFF);
}

static int BT_motor_send_command(const unsigned char *payload, int payload_length,
                                 int global_memory_size, unsigned char *reply)
{
 unsigned char command[64];
 int command_length=payload_length+7;
 int bytes_written=0;
 int bytes_read=0;
 int reply_length;
 int sent_id=message_id_counter&0xFFFF;
 int stale_replies=0;
 unsigned char discarded_reply[1024];
 unsigned char *reply_buffer=reply;

 // The firmware allows at most 1021 bytes of global (reply) memory per direct command
 if (payload_length<1 || command_length>(int)sizeof(command) ||
     global_memory_size<0 || global_memory_size>1021)
 {
  fprintf(stderr,"Motor command: invalid command size\n");
  return(-1);
 }

 memset(command,0,sizeof(command));
 command[0]=(unsigned char)((command_length-2)&0xFF);
 command[1]=(unsigned char)((command_length-2)>>8);
 command[2]=(unsigned char)(message_id_counter&0xFF);
 command[3]=(unsigned char)((message_id_counter>>8)&0xFF);
 command[4]=0x00;
 command[5]=(unsigned char)(global_memory_size&0xFF);
 command[6]=(unsigned char)((global_memory_size>>8)&0xFF);
 memcpy(&command[7],payload,payload_length);

 while (bytes_written<command_length)
 {
  int written=(int)write(*socket_id,&command[bytes_written],command_length-bytes_written);
  if (written<=0)
  {
   perror("Motor command write failed");
   message_id_counter++;
   return(-1);
  }
  bytes_written+=written;
 }
 message_id_counter++;

 if (reply_buffer==NULL) reply_buffer=discarded_reply;
 for (;;)
 {
  bytes_read=0;
  memset(reply_buffer,0,1024);
  while (bytes_read<2)
  {
   int received=(int)read(*socket_id,&reply_buffer[bytes_read],2-bytes_read);
   if (received<=0)
   {
    perror("Motor command reply read failed");
    return(-1);
   }
   bytes_read+=received;
  }
  reply_length=(int)reply_buffer[0]+((int)reply_buffer[1]<<8)+2;
  if (reply_length<5 || reply_length>1024)
  {
   fprintf(stderr,"Motor command: invalid reply length\n");
   return(-1);
  }
  while (bytes_read<reply_length)
  {
   int received=(int)read(*socket_id,&reply_buffer[bytes_read],reply_length-bytes_read);
   if (received<=0)
   {
    perror("Motor command reply read failed");
    return(-1);
   }
   bytes_read+=received;
  }
  // The EV3 echoes the message id. A reply carrying another id belongs to an
  // earlier command, so its payload must not be handed back as this one's.
  if (((int)reply_buffer[2]+((int)reply_buffer[3]<<8))==sent_id) break;
  if (++stale_replies>8)
  {
   fprintf(stderr,"Motor command: no reply with matching message id\n");
   return(-1);
  }
 }
 if (reply_buffer[4]!=0x02)
 {
  fprintf(stderr,"Motor command failed (reply type 0x%02X)\n",reply_buffer[4]);
  return(-1);
 }
 return(0);
}

static int BT_motor_send_ports(unsigned char opcode, char port_ids,
                               unsigned char *reply)
{
 unsigned char payload[8];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 payload[length++]=opcode;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 return BT_motor_send_command(payload,length,0,reply);
}

////////////////////////////////////////////////////////////////////////////////
// Selects the motor type that the EV3 associates with one output port.
// Inputs: port_no - port NUMBER, not a mask (MOTOR_NO_A=0 through MOTOR_NO_D=3)
//         motor_type - 7 for a Large Motor or 8 for a Medium Motor
// Returns: 0 on success; -1 for invalid input or a communication error
// Note: the EV3 detects motor types by itself and overwrites this value, so
//       this call is rarely needed.
////////////////////////////////////////////////////////////////////////////////
int BT_motor_set_type(char port_no, int motor_type)
{
 unsigned char payload[8];
 int length=0;
 if (!BT_motor_valid_port_no(port_no)) return(-1);
 if (motor_type!=7 && motor_type!=8)
 {
  fprintf(stderr,"BT_motor_set_type: type must be 7 (large) or 8 (medium)\n");
  return(-1);
 }
 payload[length++]=opOUTPUT_SET_TYPE;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,port_no);
 BT_motor_append_lc0(payload,&length,motor_type);
 return BT_motor_send_command(payload,length,0,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Resets the selected motor output(s): zeroes the step count that the
// step/time moves and BT_motor_get_reading use. It does not change the count
// read by BT_motor_get_count / BT_motor_get_position (see BT_motor_clear_count).
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_reset(char port_ids)
{
 return BT_motor_send_ports(opOUTPUT_RESET,port_ids,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Sets the open-loop power value for the selected motor output(s).
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         power - power percentage in [-100, 100]
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_set_power(char port_ids, int power)
{
 unsigned char payload[8];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 if (power < -100 || power > 100)
 {
  fprintf(stderr,"BT_motor_set_power: power must be in [-100, 100]\n");
  return(-1);
 }
 payload[length++]=opOUTPUT_POWER;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 BT_motor_append_lc1(payload,&length,power);
 return BT_motor_send_command(payload,length,0,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Sets the regulated speed percentage for the selected motor output(s).
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         speed - speed percentage in [-100, 100]
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_set_speed(char port_ids, int speed)
{
 unsigned char payload[8];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 if (speed < -100 || speed > 100)
 {
  fprintf(stderr,"BT_motor_set_speed: speed must be in [-100, 100]\n");
  return(-1);
 }
 payload[length++]=opOUTPUT_SPEED;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 BT_motor_append_lc1(payload,&length,speed);
 return BT_motor_send_command(payload,length,0,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Starts the selected motor output(s) using the previously configured values.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_start(char port_ids)
{
 return BT_motor_send_ports(opOUTPUT_START,port_ids,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Sets the direction convention for the selected motor output(s).
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         polarity - -1 for backward, 0 for opposite, or 1 for forward
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_set_polarity(char port_ids, int polarity)
{
 unsigned char payload[8];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 if (polarity < -1 || polarity > 1)
 {
  fprintf(stderr,"BT_motor_set_polarity: polarity must be -1, 0, or 1\n");
  return(-1);
 }
 payload[length++]=opOUTPUT_POLARITY;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 BT_motor_append_lc0(payload,&length,polarity);
 return BT_motor_send_command(payload,length,0,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Reads the measured speed and the move step count of one motor output.
// Inputs: port_no - port NUMBER, not a mask (MOTOR_NO_A=0 through MOTOR_NO_D=3)
//         speed - non-NULL output pointer for the signed speed percentage
//         tacho_count - non-NULL output pointer for the signed step count
// Returns: 0 on success; -1 for invalid input or a communication error
// Note: this step count is the EV3's per-move counter. The firmware zeroes it
//       whenever the motor is stopped (and on BT_motor_reset), so it is not an
//       odometer - use BT_motor_get_count / BT_motor_get_position for that.
////////////////////////////////////////////////////////////////////////////////
int BT_motor_get_reading(char port_no, int *speed, int *tacho_count)
{
 unsigned char payload[16];
 unsigned char reply[1024];
 int length=0;
 if (!BT_motor_valid_port_no(port_no)) return(-1);
 if (speed==NULL || tacho_count==NULL)
 {
  fprintf(stderr,"BT_motor_get_reading: output pointers must not be NULL\n");
  return(-1);
 }
 // Reply memory: 4-byte tacho at offset 0, 1-byte speed at offset 4. The EV3
 // writes the tacho with a 32-bit store, so it is kept on a 4-byte boundary.
 payload[length++]=opOUTPUT_READ;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,port_no);
 BT_motor_append_gv1(payload,&length,4);
 BT_motor_append_gv4(payload,&length,0);
 if (BT_motor_send_command(payload,length,5,reply)<0) return(-1);
 *tacho_count=(int)((unsigned int)reply[5] |
                    ((unsigned int)reply[6]<<8) |
                    ((unsigned int)reply[7]<<16) |
                    ((unsigned int)reply[8]<<24));
 *speed=(int)(signed char)reply[9];
 return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Checks whether any selected motor output is currently busy.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
// Returns: 1 if any selected output is busy, 0 if idle, or -1 on error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_is_busy(char port_ids)
{
 unsigned char payload[8];
 unsigned char reply[1024];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 payload[length++]=opOUTPUT_TEST;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 BT_motor_append_gv1(payload,&length,0);
 if (BT_motor_send_command(payload,length,1,reply)<0) return(-1);
 return reply[5]!=0;
}

////////////////////////////////////////////////////////////////////////////////
// Blocks the EV3 command until the selected motor output(s) are ready.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
// Returns: 0 on success; -1 for invalid input or a communication error
// Note: this call does not return until the step/time/sync move finishes, and
//       the EV3 ignores other commands meanwhile. A motor that is blocked and
//       never reaches its target keeps it waiting - poll BT_motor_is_busy
//       instead when a timeout is needed.
////////////////////////////////////////////////////////////////////////////////
int BT_motor_wait(char port_ids)
{
 return BT_motor_send_ports(opOUTPUT_READY,port_ids,NULL);
}

static int BT_motor_step_or_time(unsigned char opcode, char port_ids, int power_or_speed,
                                 int ramp_up, int duration, int ramp_down, int brake)
{
 unsigned char payload[32];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 if (power_or_speed < -100 || power_or_speed > 100 ||
     ramp_up<0 || duration<0 || ramp_down<0 ||
     (brake!=0 && brake!=1))
 {
  fprintf(stderr,"Motor move: invalid power, duration, ramp, or brake value\n");
  return(-1);
 }
 payload[length++]=opcode;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 BT_motor_append_lc1(payload,&length,power_or_speed);
 BT_motor_append_lc4(payload,&length,ramp_up);
 BT_motor_append_lc4(payload,&length,duration);
 BT_motor_append_lc4(payload,&length,ramp_down);
 BT_motor_append_lc0(payload,&length,brake);
 return BT_motor_send_command(payload,length,0,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Runs an open-loop power move for a tacho distance with acceleration ramps.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         power - power percentage in [-100, 100]
//         ramp_up_degrees/run_degrees/ramp_down_degrees - tacho degrees
//         brake - 1 to brake at the end, 0 to coast
// Returns: 0 on command success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_step_power(char port_ids, int power, int ramp_up_degrees,
                        int run_degrees, int ramp_down_degrees, int brake)
{
 return BT_motor_step_or_time(opOUTPUT_STEP_POWER,port_ids,power,
                              ramp_up_degrees,run_degrees,ramp_down_degrees,brake);
}

////////////////////////////////////////////////////////////////////////////////
// Runs an open-loop power move for a duration with acceleration ramps.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         power - power percentage in [-100, 100]
//         ramp_up_ms/run_ms/ramp_down_ms - durations in milliseconds
//         brake - 1 to brake at the end, 0 to coast
// Returns: 0 on command success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_time_power(char port_ids, int power, int ramp_up_ms,
                        int run_ms, int ramp_down_ms, int brake)
{
 return BT_motor_step_or_time(opOUTPUT_TIME_POWER,port_ids,power,
                              ramp_up_ms,run_ms,ramp_down_ms,brake);
}

////////////////////////////////////////////////////////////////////////////////
// Runs a regulated-speed move for a tacho distance with acceleration ramps.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         speed - speed percentage in [-100, 100]
//         ramp_up_degrees/run_degrees/ramp_down_degrees - tacho degrees
//         brake - 1 to brake at the end, 0 to coast
// Returns: 0 on command success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_step_speed(char port_ids, int speed, int ramp_up_degrees,
                        int run_degrees, int ramp_down_degrees, int brake)
{
 return BT_motor_step_or_time(opOUTPUT_STEP_SPEED,port_ids,speed,
                              ramp_up_degrees,run_degrees,ramp_down_degrees,brake);
}

////////////////////////////////////////////////////////////////////////////////
// Runs a regulated-speed move for a duration with acceleration ramps.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
//         speed - speed percentage in [-100, 100]
//         ramp_up_ms/run_ms/ramp_down_ms - durations in milliseconds
//         brake - 1 to brake at the end, 0 to coast
// Returns: 0 on command success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_time_speed(char port_ids, int speed, int ramp_up_ms,
                        int run_ms, int ramp_down_ms, int brake)
{
 return BT_motor_step_or_time(opOUTPUT_TIME_SPEED,port_ids,speed,
                              ramp_up_ms,run_ms,ramp_down_ms,brake);
}

static int BT_motor_sync(unsigned char opcode, char port_ids, int speed_or_power,
                         int turn_ratio, int distance_or_time, int brake)
{
 unsigned char payload[24];
 int length=0;
 if (!BT_motor_valid_ports(port_ids)) return(-1);
 if (BT_motor_port_count(port_ids)!=2)
 {
  // The EV3 motor driver keeps exactly two entries for a synchronized pair
  fprintf(stderr,"Motor sync: exactly two motor ports must be selected\n");
  return(-1);
 }
 if (speed_or_power < -100 || speed_or_power > 100 ||
     turn_ratio < -200 || turn_ratio > 200 || distance_or_time<0 ||
     (brake!=0 && brake!=1))
 {
  fprintf(stderr,"Motor sync: invalid speed, turn, duration, or brake value\n");
  return(-1);
 }
 payload[length++]=opcode;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,(unsigned char)port_ids);
 BT_motor_append_lc1(payload,&length,speed_or_power);
 BT_motor_append_lc2(payload,&length,turn_ratio);
 BT_motor_append_lc4(payload,&length,distance_or_time);
 BT_motor_append_lc0(payload,&length,brake);
 return BT_motor_send_command(payload,length,0,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Drives a pair of motors in sync for a tacho distance with a steering ratio.
// Inputs: port_ids - mask selecting exactly two motor outputs (MOTOR_A|MOTOR_D)
//         speed - speed percentage in [-100, 100]
//         turn_ratio - steering ratio in [-200, 200]
//         degrees - tacho degrees to move; 0 runs until stopped
//         brake - 1 to brake at the end, 0 to coast
// Returns: 0 on command success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_step_sync(char port_ids, int speed, int turn_ratio, int degrees, int brake)
{
 return BT_motor_sync(opOUTPUT_STEP_SYNC,port_ids,speed,turn_ratio,degrees,brake);
}

////////////////////////////////////////////////////////////////////////////////
// Drives a pair of motors in sync for a duration with a steering ratio.
// Inputs: port_ids - mask selecting exactly two motor outputs (MOTOR_A|MOTOR_D)
//         speed - speed percentage in [-100, 100]
//         turn_ratio - steering ratio in [-200, 200]
//         time_ms - duration in milliseconds; 0 runs until stopped
//         brake - 1 to brake at the end, 0 to coast
// Returns: 0 on command success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_time_sync(char port_ids, int speed, int turn_ratio, int time_ms, int brake)
{
 return BT_motor_sync(opOUTPUT_TIME_SYNC,port_ids,speed,turn_ratio,time_ms,brake);
}

////////////////////////////////////////////////////////////////////////////////
// Sets to 0 the tacho count read by BT_motor_get_count / BT_motor_get_position.
// Inputs: port_ids - one or more output port bits (MOTOR_A through MOTOR_D)
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_clear_count(char port_ids)
{
 return BT_motor_send_ports(opOUTPUT_CLR_COUNT,port_ids,NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Reads the tacho sensor count (degrees) of one motor output. This count keeps
// running across moves and stops; only BT_motor_clear_count sets it to 0.
// Inputs: port_no - port NUMBER, not a mask (MOTOR_NO_A=0 through MOTOR_NO_D=3)
//         tacho_count - non-NULL output pointer for the signed tacho count
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_get_count(char port_no, int *tacho_count)
{
 unsigned char payload[16];
 unsigned char reply[1024];
 int length=0;
 if (!BT_motor_valid_port_no(port_no)) return(-1);
 if (tacho_count==NULL)
 {
  fprintf(stderr,"BT_motor_get_count: output pointer must not be NULL\n");
  return(-1);
 }
 payload[length++]=opOUTPUT_GET_COUNT;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,port_no);
 BT_motor_append_gv4(payload,&length,0);
 if (BT_motor_send_command(payload,length,4,reply)<0) return(-1);
 *tacho_count=(int)((unsigned int)reply[5] |
                    ((unsigned int)reply[6]<<8) |
                    ((unsigned int)reply[7]<<16) |
                    ((unsigned int)reply[8]<<24));
 return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Reads the tacho sensor counts of two motors in a single Bluetooth round trip
// (half the time of two BT_motor_get_count calls - useful inside a control loop).
// Inputs: port_no_a, port_no_b - port NUMBERS, not masks (MOTOR_NO_A=0 through MOTOR_NO_D=3)
//         count_a, count_b - non-NULL output pointers for the signed tacho counts
// Returns: 0 on success; -1 for invalid input or a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_get_count_pair(char port_no_a, char port_no_b, int *count_a, int *count_b)
{
 unsigned char payload[24];
 unsigned char reply[1024];
 int length=0;
 if (!BT_motor_valid_port_no(port_no_a) || !BT_motor_valid_port_no(port_no_b)) return(-1);
 if (count_a==NULL || count_b==NULL)
 {
  fprintf(stderr,"BT_motor_get_count_pair: output pointers must not be NULL\n");
  return(-1);
 }
 // Two opOUTPUT_GET_COUNT opcodes in one command; counts land at reply offsets 0 and 4
 payload[length++]=opOUTPUT_GET_COUNT;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,port_no_a);
 BT_motor_append_gv4(payload,&length,0);
 payload[length++]=opOUTPUT_GET_COUNT;
 BT_motor_append_lc0(payload,&length,0);
 BT_motor_append_lc0(payload,&length,port_no_b);
 BT_motor_append_gv4(payload,&length,4);
 if (BT_motor_send_command(payload,length,8,reply)<0) return(-1);
 *count_a=(int)((unsigned int)reply[5] |
                ((unsigned int)reply[6]<<8) |
                ((unsigned int)reply[7]<<16) |
                ((unsigned int)reply[8]<<24));
 *count_b=(int)((unsigned int)reply[9] |
                ((unsigned int)reply[10]<<8) |
                ((unsigned int)reply[11]<<16) |
                ((unsigned int)reply[12]<<24));
 return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Reads the current position of one motor in tacho degrees.
// Inputs: port_id - exactly one output port bit (MOTOR_A, MOTOR_B, MOTOR_C or MOTOR_D)
//         position - non-NULL output pointer for the signed position
// Returns: 0 on success; -1 for invalid input or a communication error
// Note: same value as BT_motor_get_count, selected by port bit instead of port
//       number. opOUTPUT_POSITION cannot be used for this: it is a "set
//       position" opcode that the EV3 firmware never implemented, and the
//       brick answers it with an error reply.
////////////////////////////////////////////////////////////////////////////////
int BT_motor_get_position(char port_id, int *position)
{
 int port_no=0;
 if (!BT_motor_valid_ports(port_id)) return(-1);
 if (BT_motor_port_count(port_id)!=1)
 {
  fprintf(stderr,"BT_motor_get_position: exactly one motor port must be selected\n");
  return(-1);
 }
 if (position==NULL)
 {
  fprintf(stderr,"BT_motor_get_position: output pointer must not be NULL\n");
  return(-1);
 }
 while (!(port_id&(1<<port_no))) port_no++;
 return BT_motor_get_count((char)port_no,position);
}

////////////////////////////////////////////////////////////////////////////////
// Sends the EV3 program-stop motor opcode to stop program output activity.
// Inputs: none
// Returns: 0 on command success; -1 on a communication error
////////////////////////////////////////////////////////////////////////////////
int BT_motor_program_stop(void)
{
 unsigned char payload[1]={opOUTPUT_PRG_STOP};
 return BT_motor_send_command(payload,1,0,NULL);
}


/**********************************************************************************************************************
 *
 *      Robot configuration used by Parts 2 to 4                                       (from EV3_Localization.h)
 *
 *********************************************************************************************************************/

#ifndef HEXKEY
	#define HEXKEY "00:16:53:56:4c:53"	// <--- SET UP YOUR EV3's HEX ID here
#endif

#define DRIVE_WHEELS    (MOTOR_A | MOTOR_D)  // Left wheel on port A, right wheel on port D
#define GYRO_PORT       PORT_1               // Port the gyro sensor is plugged into
#define GYRO_DIR        1                    // 1 if the gyro angle grows turning right (CW), -1 if it grows turning left
#define WHEEL_DIAM_MM   48.2                 // Effective drive-wheel diameter, measured: a commanded 100mm drive went 88.5mm at 56.0
#define TRACK_MM        120.0                // Distance between the two drive-wheel centres - MEASURE THIS on the robot
#define TURN_SPEED      50                   // Sync speed (percent) for the main part of a turn
#define TURN_FIX_SPEED  10                   // Slower sync speed for the gyro corrections
#define TURN_LEFT_RATIO (-200)               // turn_ratio that spins LEFT (A back, D fwd). Flip the sign if a positive angle turns RIGHT
#define TURN_MAX_FIXES  4                    // Gyro corrections allowed after the main move before giving up
#define TURN_STILL      2                    // Robot counts as stopped below this many degrees/second...
#define TURN_SETTLE     3                    // ...for this many gyro readings in a row
#define TURN_TIMEOUT    10.0                 // Give up on a single sync move after this many seconds

#define CALIB_FILE "calibration.txt"   // Calibration data file (read at startup)
#define CAL_READ_SAMPLES 10            // Amount of samples taken for denoising
#define COLOUR_SENSOR_PORT PORT_2      // Port the NXT colour sensor is plugged into

#define RAIL_MOTOR MOTOR_B
#define RAIL_TACHO_NO MOTOR_NO_B // Same motor as a port number, for reading the tacho count
#define RAIL_LEFT_DEG (-140)     // Furthest LEFT the sensor may travel, in tacho degrees from centre (0)
#define RAIL_RIGHT_DEG 140       // Furthest RIGHT the sensor may travel, in tacho degrees from centre (0)
#define RAIL_SPEED 60            // Regulated speed (percent) used for every rail move
#define RAIL_RAMP_DEG 10         // Longest speed-up / slow-down ramp of a rail move, in tacho degrees
#define RAIL_TOL_DEG 3           // On target once the tacho is within this many degrees of where it was sent
#define RAIL_MAX_TRIES 1         // Moves allowed per target (first move plus nudges) before settling
#define RAIL_SETTLE 0.25         // Stopped once the tacho has not changed for this many seconds
#define RAIL_TIMEOUT 2.0         // Give up on a single rail move after this many seconds

#define DRIVE_SPEED        80     // Sync speed (percent) for straight moves
#define FS_MOVE_SPEED      15     // Sync speed for positioning moves (fast short moves overshoot)

typedef enum colour_t
{
 COLOUR_INVALID = -1,
 COLOUR_BLACK   = 1,
 COLOUR_BLUE    = 2,
 COLOUR_GREEN   = 3,
 COLOUR_YELLOW  = 4,
 COLOUR_RED     = 5,
 COLOUR_WHITE   = 6
} colour_t;

#define NUM_COLOURS 6                  // Number of reference colours (BLACK..WHITE)

// Reference entry for a single colour. 'valid' tells us whether this reference holds
// usable data.
typedef struct
{
 int R, G, B;
 int valid;
} colour_ref;

// Glue for this collection file: prototypes of the Part 3 and Part 4 functions (they are in EV3_Localization.h).
void calibrate_sensor(void);
int load_calibration(void);
int read_colour_med(int *R, int *G, int *B, int *A);
colour_t colour_assign(void);
int rail_goto(int target);
int read_rail_colours(int count, colour_t *res);
void P_gyro_turn(int ang);

/**********************************************************************************************************************
 *
 *      PART 2 - Drive and gyro-turn helpers                                            (from EV3_Localization.c)
 *
 *********************************************************************************************************************/

// Polls until the drive wheels finish their current move. Returns 0 on success,
// -1 on error or timeout.
static int wait_wheels(void)
{
 struct timespec start, cur;
 int busy;

 // Poll BT_motor_is_busy rather than BT_motor_wait, so a stuck wheel can't hang the program forever
 clock_gettime(CLOCK_REALTIME, &start);
 do {
  busy = BT_motor_is_busy(DRIVE_WHEELS);
  clock_gettime(CLOCK_REALTIME, &cur);
  if ((cur.tv_sec - start.tv_sec) + (cur.tv_nsec - start.tv_nsec) / 1e9 > TURN_TIMEOUT)
  {
   fprintf(stderr, "wait_wheels: timed out waiting for the move to finish\n");
   return(-1);
  }
 } while (busy == 1);
 return(busy < 0 ? -1 : 0);
}

// Spins in place by 'deg' (positive = left) with a regulated sync move, then
// polls until the move finishes. Returns 0 on success, -1 on error or timeout.
static int sync_spin(double deg, int speed)
{
 int tacho = (int)(fabs(deg) * TRACK_MM / WHEEL_DIAM_MM + 0.5);  // wheel degrees for an in-place spin

 if (tacho == 0) return(0);
 if (BT_motor_step_sync(DRIVE_WHEELS, speed, deg > 0 ? TURN_LEFT_RATIO : -TURN_LEFT_RATIO, tacho, 1) < 0) return(-1);
 return(wait_wheels());
}

// Drives straight by 'mm' millimetres (negative = reverse) at 'speed' percent with a
// regulated sync move, then polls until the move finishes. Returns 0 on success, -1 on error or timeout.
static int sync_drive_speed(double mm, int speed)
{
 int tacho = (int)(fabs(mm) * 360.0 / (M_PI * WHEEL_DIAM_MM) + 0.5);  // wheel degrees for that distance

 if (tacho == 0) return(0);
 if (BT_motor_step_sync(DRIVE_WHEELS, mm > 0 ? speed : -speed, 0, tacho, 1) < 0) return(-1);
 return(wait_wheels());
}

// Straight move at the normal driving speed.
static int sync_drive(double mm)
{
 return(sync_drive_speed(mm, DRIVE_SPEED));
}

// Straight move at the slow positioning speed find_street() uses, so short moves don't overshoot.
static int sync_drive_slow(double mm)
{
 return(sync_drive_speed(mm, FS_MOVE_SPEED));
}

// Waits until the robot has stopped rotating, then returns how far it has
// turned to the LEFT (degrees) since the last gyro reset.
static int settled_heading(void)
{
 int angle = 0, rate = 0, still = 0, reads = 0;
 while (still < TURN_SETTLE && reads < 50)
 {
  BT_read_gyro(GYRO_PORT, 0, &angle, &rate);
  still = (abs(rate) <= TURN_STILL) ? still + 1 : 0;
  reads++;
 }
 return(-GYRO_DIR * angle);
}

// Turns the EV3 in place by 'ang' degrees (positive = left, negative = right).
// A regulated sync move does the turn, then the gyro measures the result and
// small sync moves close the remaining error to the gyro's 1-degree resolution.
void P_gyro_turn(int ang)
{
 int a = 0, r = 0, heading, err, fix;
 double scale = 1.0;   // gyro degrees per commanded degree; usually < 1 because the tyres scrub

 if (ang > 360 || ang < -360)
 {
  fprintf(stderr, "P_gyro_turn: the angle must be between -360 and 360 degrees\n");
  return;
 }

 BT_read_gyro(GYRO_PORT, 1, &a, &r);            // reset the gyro to zero

 if (sync_spin(ang, TURN_SPEED) < 0) { BT_all_stop(1); return; }
 heading = settled_heading();

 // Learn this floor's slip from the main move, so the corrections land on target
 if (abs(heading) > abs(ang) / 2)
 {
  scale = (double)ang / heading;
  if (scale < 0.7) scale = 0.7;
  if (scale > 1.5) scale = 1.5;
 }

 for (fix = 0; fix < TURN_MAX_FIXES; fix++)
 {
  err = ang - heading;
  if (err == 0) break;
  if (sync_spin(err * scale, TURN_FIX_SPEED) < 0) break;
  heading = settled_heading();
 }

 BT_all_stop(1);
}

/**********************************************************************************************************************
 *
 *      PART 3 - Colour sensor calibration and classification                           (from EV3_Localization.c)
 *
 *      calibrate_sensor() was an empty stub in the starter code; the body is ours.
 *
 *********************************************************************************************************************/

/* Colour reference table starts at these default values below load_calibration()
 * overwrites them with real sensor values if a valid CALIB_FILE exists. */
static colour_ref fallback_references[NUM_COLOURS + 1] = {
    [COLOUR_BLACK]  = { 133, 115,  88, 1 },
    [COLOUR_BLUE]   = {  93, 122, 212, 1 },
    [COLOUR_GREEN]  = { 101, 136,  46, 1 },
    [COLOUR_YELLOW] = { 432, 412, 119, 1 },
    [COLOUR_RED]    = { 458, 133,  86, 1 },
    [COLOUR_WHITE]  = { 465, 476, 480, 1 }
};

void calibrate_sensor(void)
{
 // Hold the sensor on each colour in turn, store the per-channel median as that colour's
 // reference, and write the results to CALIB_FILE for later runs.
 int order[NUM_COLOURS] = { COLOUR_BLACK, COLOUR_BLUE, COLOUR_GREEN, COLOUR_YELLOW, COLOUR_RED, COLOUR_WHITE };
 const char *names[NUM_COLOURS] = { "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE" };

 FILE *f;
 int c;

 fprintf(stderr,"Calibration function called!\n");

 // calibrate_sensor() is called from main() BEFORE the BT socket is opened, so we open
 // our own connection here and close it when we are done.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"calibrate_sensor(): Unable to open comm socket to the EV3, cannot calibrate\n");
  return;
 }

 // Any colour we fail to read keeps its existing default value.
 for (int k=0; k<NUM_COLOURS; k++)
 {
  int R, G, B, A;

  c = order[k];
  fprintf(stderr,"\nPlace the sensor on the %s colour, then press ENTER to sample...",names[k]);
  // Wait for the user to press ENTER before sampling.
  while (getchar()!='\n');

  if (read_colour_med(&R, &G, &B, &A)==1)
  {
   fallback_references[c].R = R;
   fallback_references[c].G = G;
   fallback_references[c].B = B;
   fallback_references[c].valid = 1;
   fprintf(stderr,"  %s reference set to R=%d, G=%d, B=%d\n",names[k],R,G,B);
  }
  else
  {
   fprintf(stderr,"  Sensor read failed for %s, keeping previous/default value\n",names[k]);
  }
 }

 BT_close();

 // Save to CALIB_FILE: one "R G B" line per colour, enum order (1..6).
 f = fopen(CALIB_FILE,"w");
 if (f==NULL)
 {
  fprintf(stderr,"calibrate_sensor(): Unable to open %s for writing\n",CALIB_FILE);
  return;
 }
 for (int col=1; col<=NUM_COLOURS; col++)
  fprintf(f,"%d %d %d\n",fallback_references[col].R,fallback_references[col].G,fallback_references[col].B);
 fclose(f);

 fprintf(stderr,"\nCalibration complete, data written to %s\n",CALIB_FILE);
}

int load_calibration(void)
{
 // Read CALIB_FILE ("R G B" per line, enum order 1..6) into the reference table. Parse into
 // a temp first and commit only if all six lines are valid, so a missing or malformed file
 // leaves the defaults untouched. Returns 1 on success, 0 otherwise.
 FILE *f;
 colour_ref tmp[NUM_COLOURS + 1];
 int R, G, B;

 f = fopen(CALIB_FILE,"r");
 if (f==NULL) return(0);

 for (int col=1; col<=NUM_COLOURS; col++)
 {
  if (fscanf(f,"%d %d %d",&R,&G,&B)!=3) { fclose(f); return(0); }
  tmp[col].R = R;
  tmp[col].G = G;
  tmp[col].B = B;
  tmp[col].valid = 1;
 }
 fclose(f);

 for (int col=1; col<=NUM_COLOURS; col++) fallback_references[col] = tmp[col];
 return(1);
}

static int cmp_int(const void *a, const void *b)
{
 int ia = *((const int *)a);
 int ib = *((const int *)b);
 if (ia<ib) return(-1);
 if (ia>ib) return(1);
 return(0);
}

int read_colour_med(int *R, int *G, int *B, int *A)
{
 // Take CAL_READ_SAMPLES readings and return the per-channel median (caller holds the rail
 // paused over the target). Returns 1 on success, -1 if any read fails.
 //
 // The firmware already removes ambient from R,G,B, then the API subtracts A again (which
 // can go negative), so we add the median A back per channel; *A returns that median.
 int rbuf[CAL_READ_SAMPLES];
 int gbuf[CAL_READ_SAMPLES];
 int bbuf[CAL_READ_SAMPLES];
 int abuf[CAL_READ_SAMPLES];
 int r, g, b, a;
 int med_r, med_g, med_b, med_a;

 for (int i=0; i<CAL_READ_SAMPLES; i++)
 {
  if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT, &r, &g, &b, &a)!=1)
   return(-1);
  rbuf[i] = r;
  gbuf[i] = g;
  bbuf[i] = b;
  abuf[i] = a;
 }

 qsort(rbuf, CAL_READ_SAMPLES, sizeof(int), cmp_int);
 qsort(gbuf, CAL_READ_SAMPLES, sizeof(int), cmp_int);
 qsort(bbuf, CAL_READ_SAMPLES, sizeof(int), cmp_int);
 qsort(abuf, CAL_READ_SAMPLES, sizeof(int), cmp_int);

 // Median (even count -> lower-middle element, fine for noise rejection).
 med_r = rbuf[CAL_READ_SAMPLES/2];
 med_g = gbuf[CAL_READ_SAMPLES/2];
 med_b = bbuf[CAL_READ_SAMPLES/2];
 med_a = abuf[CAL_READ_SAMPLES/2];

 // Add the median ambient back into each channel.
 *R = med_r + med_a;
 *G = med_g + med_a;
 *B = med_b + med_a;
 *A = med_a;
 return(1);
}

// White-balances a reading against the WHITE reference and puts it on the HSV cylinder as
// (s*cos h, s*sin h, v). Hue is scaled by saturation so it stops mattering for black/white.
static void colour_features(double R, double G, double B, double f[3])
{
 colour_ref *w = &fallback_references[COLOUR_WHITE];
 double r, g, b, mx, mn, h, s;

 r = fmax(R, 0) / (w->R > 0 ? w->R : 1);
 g = fmax(G, 0) / (w->G > 0 ? w->G : 1);
 b = fmax(B, 0) / (w->B > 0 ? w->B : 1);

 mx = fmax(r, fmax(g, b));
 mn = fmin(r, fmin(g, b));
 if (mx <= 0) { f[0] = f[1] = f[2] = 0; return; }

 if (mx == mn) h = 0;
 else if (mx == r) h = (g - b) / (mx - mn);
 else if (mx == g) h = 2 + (b - r) / (mx - mn);
 else h = 4 + (r - g) / (mx - mn);
 h *= M_PI / 3;
 s = (mx - mn) / mx;

 f[0] = s * cos(h);
 f[1] = s * sin(h);
 f[2] = mx;
}

// Reads the sensor and returns the nearest calibrated colour in HSV space, or -1 if the read fails.
colour_t colour_assign(void)
{
 int R, G, B, A, best = -1;
 double f[3], ref[3], d, best_d = 0;
 colour_ref *c;

 if (read_colour_med(&R, &G, &B, &A) != 1) return(-1);
 colour_features(R, G, B, f);

 for (int i = 1; i <= NUM_COLOURS; i++) {
  c = &fallback_references[i];
  if (!c->valid) continue;
  colour_features(c->R, c->G, c->B, ref);
  d = (f[0]-ref[0])*(f[0]-ref[0]) + (f[1]-ref[1])*(f[1]-ref[1]) + (f[2]-ref[2])*(f[2]-ref[2]);
  if (best < 0 || d < best_d) { best = i; best_d = d; }
 }
 return(best);
}

/**********************************************************************************************************************
 *
 *      PART 4 - Sensor rail positioning and colour sweeps                              (from EV3_Localization.c)
 *
 *********************************************************************************************************************/

// Move the rail sensor to an absolute tacho position (degrees from centre, the
// tacho is zeroed at the centre). Positions come from the tacho, never from
// timing, so errors don't build up.
int rail_goto(int target)
{
  struct timespec t0,tc,now;
  int count,pos,err,last,way,dist,ramp;

  for (int tries=0; tries<RAIL_MAX_TRIES; tries++)
  {
    // A stop wipes the EV3's internal move count, so measure the move from the
    // position read right here.
    if (BT_motor_port_stop(RAIL_MOTOR,1)<0||BT_motor_get_count(RAIL_TACHO_NO,&count)<0)
      return(-1);
    pos=count;
    err=target-pos;
    if (abs(err)<=RAIL_TOL_DEG) break;

    way=err>0?1:-1;
    dist=abs(err);
    ramp=dist/3<RAIL_RAMP_DEG?dist/3:RAIL_RAMP_DEG;
    if (BT_motor_step_speed(RAIL_MOTOR,way*RAIL_SPEED,ramp,dist-2*ramp,ramp,1)<0)
    {
      BT_motor_port_stop(RAIL_MOTOR,0);
      return(-1);
    }

    // Wait until the sensor has come to rest (tacho stops changing) or we time out.
    last=pos;
    clock_gettime(CLOCK_REALTIME,&t0);
    tc=t0;
    while (1)
    {
      if (BT_motor_get_count(RAIL_TACHO_NO,&count)<0)
      {
        BT_motor_port_stop(RAIL_MOTOR,0);
        return(-1);
      }
      pos=count;
      clock_gettime(CLOCK_REALTIME,&now);
      if (pos!=last) { last=pos; tc=now; }
      if ((((double)(now.tv_sec-tc.tv_sec))+((double)(now.tv_nsec-tc.tv_nsec))/1000000000.0)>=RAIL_SETTLE) break;
      if ((((double)(now.tv_sec-t0.tv_sec))+((double)(now.tv_nsec-t0.tv_nsec))/1000000000.0)>=RAIL_TIMEOUT) break;
    }
  }

  BT_motor_port_stop(RAIL_MOTOR,1);
  return(0);
}

// Read 'count' colours in one full movement of the rail, left to right, into res[].
// res[0] is the leftmost reading and res[count-1] the rightmost; with an odd count
// the middle reading lands on the centre (0). Positions are evenly spaced tacho
// degrees between RAIL_LEFT_DEG and RAIL_RIGHT_DEG.
int read_rail_colours(int count, colour_t *res)
{
  static int zeroed=0;
  int target,div;

  if (res==NULL||count<1) return(0);

  // The sensor must be at the centre of the rail the first time this is called,
  // that position becomes tacho 0 for the rest of the run.
  if (!zeroed)
  {
    if (BT_motor_clear_count(RAIL_MOTOR)<0) return(0);
    zeroed=1;
  }

  // phase 1: move to the leftmost position (most negative position)
  if (rail_goto(RAIL_LEFT_DEG)<0) { BT_motor_port_stop(RAIL_MOTOR,0); return(0); }

  // phase 2: move to the rightmost position (most positive position), reading colours
  //          at count evenly spaced stops along the way and storing them in res
  div=count>1?count-1:1;
  for (int i=0; i<count; i++)
  {
    target=RAIL_LEFT_DEG+((RAIL_RIGHT_DEG-RAIL_LEFT_DEG)*i)/div;
    if (rail_goto(target)<0) { BT_motor_port_stop(RAIL_MOTOR,0); return(0); }
    res[i]=colour_assign();
  }

  // phase 3: move back to the zero position (center of the rail)
  if (rail_goto(0)<0) { BT_motor_port_stop(RAIL_MOTOR,0); return(0); }

  return(1);
}

#else  /* EV3_NEW_APIS_BT_WIN */

/**********************************************************************************************************************
 *
 *      PART 5 - Windows Bluetooth transport                                      (from EV3_RobotControl/bt_win.c)
 *
 *      Compiled only with -DEV3_NEW_APIS_BT_WIN (see the note at the top of this file).
 *
 *********************************************************************************************************************/

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>   // must come before windows.h
#include <ws2bth.h>     // AF_BTH, BTHPROTO_RFCOMM, SOCKADDR_BTH, BTH_ADDR
#include <windows.h>
#include <stdio.h>

// Glue for this collection file: the prototypes from bt_win.h
int bt_win_open(const char *device_id);
int bt_win_close(void);
int bt_win_write(const void *buf, int n);
int bt_win_read(void *buf, int n);

static SOCKET ev3_socket = INVALID_SOCKET;   // the single RFCOMM socket to the EV3
static int    wsa_started = 0;               // WSAStartup only needs to run once per process

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Windows replacement for BlueZ's str2ba(): parse "xx:xx:xx:xx:xx:xx" into a BTH_ADDR. The leftmost hex pair is the
// most significant byte of the 48-bit address, which matches both the address Windows shows for the paired EV3 and
// the HEXKEY string the Linux build already uses. Returns 0 on success, -1 if the string is not a valid hex address.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int str2ba_win(const char *str, BTH_ADDR *addr)
{
 unsigned int bytes[6];
 BTH_ADDR result = 0;
 int i;

 if (str == NULL || addr == NULL) return(-1);
 if (sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x",
            &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5]) != 6) return(-1);
 for (i = 0; i < 6; i++)
 {
  if (bytes[i] > 0xFF) return(-1);
  result = (result << 8) | (BTH_ADDR)(bytes[i] & 0xFF);
 }
 *addr = result;
 return(0);
}

int bt_win_open(const char *device_id)
{
 WSADATA wsaData;
 SOCKADDR_BTH addr;
 SOCKET s;

 if (ev3_socket != INVALID_SOCKET)
 {
  // Already connected - close the previous socket before opening a new one
  closesocket(ev3_socket);
  ev3_socket = INVALID_SOCKET;
 }

 if (!wsa_started)
 {
  int wsa_rv = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (wsa_rv != 0)
  {
   fprintf(stderr, "bt_win_open: WSAStartup failed (error %d)\n", wsa_rv);
   return(-1);
  }
  wsa_started = 1;
 }

 fprintf(stderr, "Request to connect to device %s\n", device_id);

 s = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
 if (s == INVALID_SOCKET)
 {
  fprintf(stderr, "bt_win_open: could not create Bluetooth socket (Winsock error %d)\n", WSAGetLastError());
  return(-1);
 }

 memset(&addr, 0, sizeof(addr));
 addr.addressFamily = AF_BTH;
 addr.port = 1;                 // RFCOMM channel 1 (matches the Linux rc_channel = 1)
 if (str2ba_win(device_id, &addr.btAddr) != 0)
 {
  fprintf(stderr, "bt_win_open: invalid device id \"%s\" (expected form 00:16:53:56:4c:53)\n", device_id);
  closesocket(s);
  return(-1);
 }

 if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
 {
  fprintf(stderr, "bt_win_open: connection to %s failed (Winsock error %d). "
                  "Is the EV3 powered on and paired in Windows?\n", device_id, WSAGetLastError());
  closesocket(s);
  return(-1);
 }

 ev3_socket = s;
 return(0);
}

int bt_win_close(void)
{
 if (ev3_socket != INVALID_SOCKET)
 {
  closesocket(ev3_socket);
  ev3_socket = INVALID_SOCKET;
 }
 // WSACleanup() is intentionally left for process exit so a later bt_win_open() in the same run still works.
 return(0);
}

int bt_win_write(const void *buf, int n)
{
 return send(ev3_socket, (const char *)buf, n, 0);
}

int bt_win_read(void *buf, int n)
{
 return recv(ev3_socket, (char *)buf, n, 0);
}

#endif /* _WIN32 */

#endif /* EV3_NEW_APIS_BT_WIN */
