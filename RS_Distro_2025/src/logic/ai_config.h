/***************************************************
 CSC C85 RoboSoccer; tunable constants

 Every tunable and hardware constant lives here

 NUMBERS BELOW ARE PLACEHOLDERS, not TRUE values.
***************************************************/

#ifndef ROBOSOCCER_AI_CONFIG_H
#define ROBOSOCCER_AI_CONFIG_H

// **** hardware ****
#define WHEEL_TRACK_CM        20.5   // wheel centre to wheel centre, cm
// Un-geared bench value (0.38) scaled x3 for the 36T:12T gear train. Turn rate
// is 2 * POWER_TO_CMPS / WHEEL_TRACK_CM.
// NOT re-measured; the gears also cut wheel torque, so re-bench.
#define POWER_TO_CMPS         1.14   // open-loop power -> forward speed, cm/s per unit
#define POWER_TO_RADPS        0.111  // open-loop power -> turn rate, rad/s per unit
#define MIN_MOVE_POWER        6      // below this, open-loop commands don't move the bot at all

#define MOTOR_TRIM_L          0      // per-wheel power bias, if one side pulls
#define MOTOR_TRIM_R          0
#define MAX_POWER             100

// (Measured on the actual robot) In-place spin has a static friction dead zone
// below this power. Below it, drive_shape_power() pulses at MIN_SPIN_POWER
// on a cycle instead of sending a command that is too weak to even begin running. 0 disables.
#define MIN_SPIN_POWER        85
#define SPIN_TRANSLATE_MAX_POWER 4   // forward-power cap while spin-pulsing
#define MAX_POWER_STEP_PER_FRAME 25  // open-loop rate-of-change cap per frame (anti-skid). NOT TESTED!

// Regulated-speed API (BT_turn_speed) calibration. Bench numbers were taken
// on the un-geared drivetrain; scaled x3 for the 36T(motor):12T(wheel) gear
// train so the wheel runs 3x the motor shaft speed. The turn gain is also
// scaled for the wheel track (12.8 -> 20.5cm). Re-bench instead of
// re-scaling if the real behaviour doesn't match.
#define SPEED_API_TO_CMPS_GAIN   1.65   // 0.55 bench x3 gear ratio
#define SPEED_API_TO_DEGPS_GAIN  2.1  // 5.133 bench x3 gear ratio x 12.8/20.5

// Bot footprint relative to the TRACKED POINT (uniform blob centroid).
// The jersey is 17.3 (front-to-back) x 5.5, centred side to side, with its
// back edge on the back of the bot, so its centre is 8.65cm from the back.
// The bot is a full width block under the jersey (wheel centres 20.5cm apart
// with ~4cm outside each, so 28.5cm wide) plus a narrower prong box that
// sticks out 7.3cm past the front of the jersey and is 12.5cm wide.
#define BOT_FRONT_CM          15.95  // tracked point -> prong tips
#define BOT_BODY_FRONT_CM     8.65   // tracked point -> front of the full width block
#define BOT_REAR_CM           8.65   // tracked point -> back of bot
#define BOT_HALF_WIDTH_CM     14.25  // half the body width
#define BOT_PRONG_HALF_WIDTH_CM 6.25 // half the prong box width

// **** perception; pixel -> cm (bench test 1, NOT uniform in x/y) ****
#define PX_PER_CM_X           7.5
#define PX_PER_CM_Y           6.3

// **** field dimensions ****
// Today's (different, non-home) field, measured in jerseys (no ruler on
// hand): 9.3 long x 6 short, jersey front-to-back = 17.3cm (already
// bench-measured, see BOT_FRONT_CM's comment below) -> 160.9 x 103.8.
// Rough (jersey-count, not ruler) but real: the old 170x115 was producing
// a heavily lopsided false wall-recovery rate (bottom wall firing on 84%
// of all WALL_RECOVER frames in one real run, top wall 0%), which a
// uniformly-wrong guess wouldn't do - that asymmetry is what a boundary
// that's actually too big in the direction that never triggered looks
// like. Re-measure properly (not by jersey count) if time allows; this
// replaces a worse number, not a final one.
#define DEFAULT_FIELD_W_CM    160.9
#define DEFAULT_FIELD_H_CM    103.8
#define DEFAULT_GOAL_WIDTH_CM 40.0
#define DEFAULT_WALL_MARGIN_CM 20.0
// Radius of the ball
#define BALL_RADIUS_CM        3.5

// **** vision / lost handling ****
#define LOST_FRAMES           18   // consecutive missed frames before self counts as lost
#define BALL_LOST_FRAMES      36   // same, for the ball
// Consecutive valid frames {penalty,chase}_next_ready()/next_lost() require before
// leaving P_READY/P_LOST.
#define READY_FRAMES          5

// **** staging / align / strike ****
#define STAGE_TOL_CM          3.5   // skill_goto "arrived" radius. Must be <= LAT_TOL_CM otherwize P_GOTO_STAGING/P_ALIGN cycle FOREVER!
#define PASSBY_CM             5.0   // also "arrived" if within this and moving away (overshoot); must be <= LAT_TOL_CM
// Inside ORBIT_ZONE_CM, a heading error over ORBIT_ERR_DEG should urge us to stop what we're doing and
// and turn first. Driving this close while badly aimed could makes us orbit near
// the target.
#define ORBIT_ZONE_CM         22.0
#define ORBIT_ERR_DEG         45.0
#define MOVE_TOL_CM           5.0   // ball-moved-since-plan threshold that triggers a replan
#define ALIGN_TOL_DEG         9.0   // heading error considered "aligned"
#define LAT_TOL_CM            5.0   // lateral offset considered "on the aim line"
#define ALIGN_STABLE_FRAMES   5     // consecutive in-tolerance frames before P_ALIGN is done
#define ALIGN_TIMEOUT_S       4.0
#define GOTO_STAGING_TIMEOUT_S 8.0
#define ALIGN_ATTEMPT_LIMIT   8     // after this many failed aligns, back off and replan
// After this many wall-recoveries with no
// progress, back off farther instead of retrying the same staging point.
#define WALL_RECOVER_ATTEMPT_LIMIT 5

#define ABORT_TOL_DEG         20.0  // strike aborts if heading error exceeds this
#define STRIKE_TIMEOUT_S      3.0
#define STRIKE_V_CMPS         100.0
// Ball displacement since strike began that counts as "struck".
// Usually robust to the ~0.5cm blob jitter that makes per-frame velocity
// utterly useless at 30fps.
#define STRIKE_MOVED_CM       4.0
#define FT_EDGE_STOP_CM       4.0   // follow-through stops within this of the field edge
#define STRIKE_V_MIN_CMPS     8.0
// Generous follow-through distance/time/speed budget. In practice
// ball_in_goal()/near_wall() stop it well before this.
#define FT_DIST_CM            180.0
#define FT_T_S                10.0
#define FT_V_CMPS             20.0
#define REPO_DIST_CM          15.0
#define REPO_V_CMPS           -20.0
#define BACKOFF_TIMEOUT_S     2.5   // give up backing off after this long (stuck/jammed)
#define NO_SHOT_MOVED_CM      6.0   // ball moved less than this since planning => no shot happened
// Ball "at rest" = stayed within REST_SPREAD_CM for REST_WINDOW_S,
// sampled at most REST_MAX_SAMPLES times.
#define REST_WINDOW_S         0.7
#define REST_SPREAD_CM        3.0
#define REST_MAX_SAMPLES      64
#define RETRY_WAIT_S          1.0

// **** wall / unstick ****
// Wall recovery only fires when a corner of the body is within
// WALL_CONTACT_CM of a wall, we are driving into it, and we haven't moved for
// WALL_STALL_S. It ends once every corner is WALL_CLEAR_CM off the wall.
#define WALL_CONTACT_CM       3.0
#define WALL_STALL_S          0.5
#define WALL_CLEAR_CM         5.0
#define WALL_RECOVER_V_CMPS   20.0
// Only drive (fwd/rev) within this many degrees of straight at/away
// from the wall. Beyond that, turn in place first.
#define WALL_DRIVE_CONE_DEG   60.0
#define UNSTICK_STALL_S       1.5
#define UNSTICK_REVERSE_S     0.4
#define UNSTICK_TURN_S        0.4
#define UNSTICK_V_CMPS        -20.0
#define UNSTICK_W_RADPS       1.5

// **** control gains (proportional; error -> commanded turn rate) ****
#define HEADING_TURN_GAIN     2.0   // heading error -> turn rate, while driving
// Same, but for in-place turns (skill_face). It needs to be much higher
// to defeat static friction (see MIN_SPIN_POWER).
#define FACE_TURN_GAIN        9.0
// In-place turns use pulses instead of a continuous loop. The
// bot reacts to commands ~200ms late then coasts, which makes a
// continuous gain limit-cycle (the stable repeating oscillation
// that the bot was doing instead of converging to the target).
// Instead, we send a pulse pulse proportional to the size of the
// error, then stand still and re-measure before the next one.
#define TURN_PULSE_W_RADPS    9.6  // pulse strength (>= MIN_SPIN_POWER*POWER_TO_RADPS)
#define TURN_SETTLE_S         0.45   // min settle time after a pulse before re-measuring
#define TURN_MIN_PULSE_S      0.03   // shortest pulse (~1 frame)
#define TURN_MAX_PULSE_S      0.7    // longest pulse
#define TURN_RATE_INIT_RADPS  7.5  // initial guess for pulse turn rate
#define TURN_UNDERSHOOT       0.75   // aim each pulse at this fraction of the remaining error
#define TURN_RATE_MAX_RADPS   30.0   // turn-rate ceiling
#define TURN_STILL_RADPS      0.6    // "stopped turning" threshold before measuring the next pulse
#define TURN_SETTLE_MAX_S     1.0    // settle timeout even if still turning
#define LATERAL_TURN_GAIN     0.05
#define GOTO_V_CMPS           45.0
#define GOTO_SLOWDOWN_CM      10.0
#define MAX_W_RADPS           4.75
// "Bruh the ball vanished mid-strike but was already this close" will be deemed a
// hit for now, instead of a tracking glitch, like maybe we are just hitting
// it in a way that makes the camera unable to detect it.
#define CLOSE_AHEAD_CM        26.0
// How far past the goal line ball_in_goal() must read before
// trusting it (used in-tandem with BALL_RADIUS_CM).
#define GOAL_CONFIRM_MARGIN_CM 0.5
// Consecutive not-in-goal indications needed before actually leaving
// P_SHOT_DONE. A ball resting right at the confirm boundary flickers
// in/out every tick from position noise.
#define GOAL_LEAVE_CONFIRM_FRAMES 5
#define STAGE_DIST_CM         30.0  // standoff distance behind the ball when staging
#define STALL_EPS_CMPS        2.0
// stall.h: stalled = commanded >= STALL_CMD_V_CMPS on average yet moved
// less than STALL_SPREAD_CM over UNSTICK_STALL_S.
#define STALL_CMD_V_CMPS      8.0
#define STALL_SPREAD_CM       3.0
#define MIN_WALL_STAGE_CM     16.0
// "Driving into the wall" means the pushing end points clearly OUT of the
// field: dot(heading, inward normal) below -this (~20deg past parallel).
#define WALL_OUTWARD_DOT       0.35
// Wall-push staging (get on the wall side of the ball, push it off) only
// applies to a ball actually within this of the wall.
#define WALL_PUSH_BAND_CM      12.0
// Staging-point clamp margins, by which end of the bot faces the wall.
// Nose (BOT_FRONT_CM) must stay fully on the table. The tail margin also
// applies to side walls when facing along them, where it is currently 2.25cm
// short of BOT_HALF_WIDTH_CM: raising it scored worse in simulation.
#define WALL_NOSE_MARGIN_CM    20.0
#define WALL_TAIL_MARGIN_CM    12.0
// Strike aims at a point this far beyond the ball on the aim line
// (not the goal itself), so it converges onto the line by contact
// instead of running parallel to it.
#define STRIKE_CARROT_CM      18.0
// Route around the ball to a staging point. If the straight path passes
// within AVOID_BLOCK_CM of the ball, detour via a point AVOID_CLEAR_CM
// to the side.
#define AVOID_BLOCK_CM        18.5
#define AVOID_CLEAR_CM        36.0
// Min speed to trust velocity for per-frame heading-sign disambiguation
// (blob shape axis can flip 180). Below this, trust last-known-good.
#define HEADING_SIGN_TRUST_CMPS 5.0
// Consecutive frames of vel disagreeing with the committed heading
// needed before actually flipping it. A single noisy frame isn't
// reliable enough in direction alone, even though it might pass
// HEADING_SIGN_TRUST_CMPS somehow.
#define HEADING_FLIP_CONFIRM_FRAMES 5
// Within this many degrees of 180, commit to the last confidently-chosen
// turn direction instead of re-deciding each frame (Don't redecide zone).
#define ANTIPODAL_DEADZONE_DEG 30.0
// Implied speed above this in one frame looks like a bad detection, and probably not
// actual motion. Reject the reading instead of trusting it.
#define MAX_PLAUSIBLE_SPEED_CMPS 150.0
// fix_heading_sign() may only use measured velocity to pick a sign while
// we're actually commanding motion above this speed. While spinning in
// place the blob centroid swings around the axle and reports a fake
// velocity in an arbitrary direction, which used to flip heading 180deg
// every few frames.
#define HEADING_CMD_V_MIN_CMPS 8.0
#define CHASE_EDGE_MARGIN_CM   15.0  // chase never steers closer to a wall than this
// In chase, don't wall-recover while the ball is within ENGAGE_BALL_CM and
// ENGAGE_BALL_DEG of dead ahead unless within HARD_WALL_MARGIN_CM of the edge.
#define ENGAGE_BALL_CM         26.0
// Ball in the pouch. Control gently (CARRY_GAIN_SCALE of normal gain, and no
// correction inside CARRY_DEADBAND_DEG). full-gain steering on the
// pouch kept swinging the ball sideways instead of carrying it.
#define CARRY_DIST_CM          22.0
#define CARRY_GAIN_SCALE       0.5
#define CARRY_DEADBAND_DEG     8.0
#define ENGAGE_BALL_DEG        60.0
#define HARD_WALL_MARGIN_CM    12.0
// Ball "with us" = its centre is in the prong basket in the bot's own frame.
// Forward from the front of the block to BALL_WITH_US_SLACK_CM past where a
// ball just touching the prong tips would sit, and sideways no wider than the
// prong box. Held through BALL_WITH_US_HOLD_FRAMES of dropout since the prongs
// can hide the ball from the camera.
#define BALL_WITH_US_SLACK_CM    2.0
#define BALL_WITH_US_HOLD_FRAMES 6
// Stuck on a wall with the ball: turn off the wall with it instead of backing
// away. A little forward speed keeps the ball pressed in and keeps
// drive_shape_power() from spin pulsing at MIN_SPIN_POWER. It stays under
// STALL_CMD_V_CMPS so the turn can't trip the general stall detector.
#define WALL_CARRY_V_CMPS     6.0
#define WALL_CARRY_W_RADPS    3.0
#define WALL_CARRY_DONE_DOT   0.5   // done once dot(heading, inward normal) reaches this (~30deg into the field)
#define WALL_CARRY_STALL_S    0.5   // heading moved less than WALL_CARRY_MOVED_DEG for this long => give up and reverse
#define WALL_CARRY_MOVED_DEG  5.0

// **** soccer mode (intercept/clear/mark) ****** no bench data
// behind any of these yet, legit js guesses to be tuned once wired up ****

// How far ahead in time skill_intercept extrapolates the ball's position
// (lead = ball.pos + ball.vel * this), instead of chasing where it
// already was. Don't make it too long or a fast ball's lead point will be
// off the field.
#define INTERCEPT_LEAD_S       0.4
// skill_clear aborts like skill_strike does, but much more forgiving.
// clearing prioritizes getting the ball away NOW over aiming it well, so
// it shouldn't bail just because it isn't lined up perfectly.
#define CLEAR_ABORT_TOL_DEG    60.0
// How far in front of own_goal skill_mark stands while shadowing the
// opponent
#define MARK_STAND_DIST_CM     40.0
// mode_soccer: ball within this of own_goal => skill_clear (urgent),
// regardless of anything else.
#define DEFEND_DANGER_DIST_CM  40.0
// mode_soccer: ball moving toward own_goal faster than this (component of
// ball.vel along ball->own_goal) => skill_intercept instead of skill_mark.
#define BALL_APPROACH_SPEED_CMPS 15.0

// **** opponent avoidance (mode_soccer) ****
// Path-detour trigger/clearance for routing S_ATTACK's steering target
// around the opponent, same plan_avoid_point() already used to route
// around the ball. Sized off BOT_HALF_WIDTH_CM since the obstacle here is
// a whole second robot body, not the ball.
#define OPP_AVOID_BLOCK_CM    (BOT_HALF_WIDTH_CM * 2.0 + 6.0)
#define OPP_AVOID_CLEAR_CM    50.0
// Unconditional safety net, applied after EVERY state's run() regardless
// of what it decided to do (see .local/notes/06-opponent-and-testing.md:
// "safety rules do not depend on the model being right"). Inside this
// distance, with the opponent roughly ahead, cap forward speed instead of
// trusting the steering/detour above to have already handled it.
#define OPP_DANGER_CM         (BOT_HALF_WIDTH_CM * 2.0 + 10.0)
#define OPP_DANGER_DEG        45.0
#define OPP_BACKOFF_V_CMPS    8.0

// S_ATTACK-only capture taper. skill_chase() has no distance-based
// slowdown at all - its only speed reduction is cos(heading error), so
// once reasonably aimed it drives at full GOTO_V_CMPS right through
// contact (confirmed in a real log: v stayed pinned at ~45 cm/s all the
// way from 23cm to 19cm from the ball) - a kick, not a controlled touch
// into the pouch. Not fixed inside skill_chase() itself: mode_chase's own
// doc comment says "we consider contact to be the kick", so a hard hit is
// that mode's actual intent, not a bug to share a fix with. This tapers
// S_ATTACK's own speed only, once within BALL_CAPTURE_DIST_CM of the
// BALL itself - not of whatever aim point it's steering at, which can be
// STRIKE_CARROT_CM beyond the ball.
#define BALL_CAPTURE_DIST_CM    25.0
#define BALL_CAPTURE_MIN_V_FRAC 0.45

// **** logging ****
// 1: every adapter frame appends a line to positions_<col>_<mode>.log (all
// agents, raw blob data, motion, wheel powers, plan). 0: no log file is opened
// or written, and none of the formatting work is done. You can override from the
// command line with -DENABLE_FRAME_LOG=0, or edit the constant here.
#ifndef ENABLE_FRAME_LOG
#define ENABLE_FRAME_LOG      1
#endif

#endif
