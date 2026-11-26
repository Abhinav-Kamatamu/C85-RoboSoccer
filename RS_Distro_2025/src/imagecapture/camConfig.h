#ifndef __camConfig_header
#define __camConfig_header

// Camera tunables. We're keeping threm here instead of imageCapture.c so
// that we can easily fine and retune them whenever we want
//
// Camera auto-white-balance/auto-exposure/continuous-autofocus are all on
// by default on most UVC webcams, and all three continuously re-interpret
// the scene, meaning that the hue a pixel reports can drift between
// whenever we run colour calibration ('c') and whenever we're actually
// playing, or even mid-match as the scene's content/brightness changes.
// initCam() locks all three to the fixed values below right after opening
// the camera.
//
// These indicate which camera controls are locked to fixed values at startup. Anything not
// locked is left in the AUTO state the colour calibration (colours.dat) was
// made in. Examples include auto white balance, continuous autofocus, Aperture Priority
// auto exposure.
//
// CAM_LOCK_EXPOSURE 1: manual exposure of CAM_EXPOSURE_TIME. Auto exposure
// slowly drifts between ~15 ms and ~60 ms as the scene brightness changes,
// and since frame rate = min(30, 1000/exposure_ms), the logged fps slid
// between 30 and 17  because of this. A fixed exposure <= 30 ms
// maintains 30 fps and also reduces motion blur.
//
// CAM_LOCK_WB_FOCUS 1: also lock white balance (CAM_WB_TEMP_K) and focus
// (CAM_FOCUS_POS). This is off by default, because the focus value we currently
// have is an unverified placeholder and auto WB already matches the existing calibration.
#define CAM_LOCK_EXPOSURE  1
#define CAM_LOCK_WB_FOCUS  0

// TODO: MEASURE IN LAB
#define CAM_WB_TEMP_K      5500  // V4L2_CID_WHITE_BALANCE_TEMPERATURE, Kelvin, range ~2800-6500
// This was measured on the HP Webcam HD 3310. The frame
// rate = min(30, 1000/exposure_ms). 150 (15ms) gave the same mean brightness (139/255)
// as Aperture Priority auto exposure did in the lab at the time (143/255).
// NOTE the auto mode picks longer exposures in dim light, which is also
// what costs us frame rate. WE NEED 30 !!!!
#define CAM_EXPOSURE_TIME  150   // V4L2_CID_EXPOSURE_ABSOLUTE, 100us units (range 5-10000)
#define CAM_FOCUS_POS      10    // V4L2_CID_FOCUS_ABSOLUTE, driver-specific units. HAS NOT BEEN VERIFIED

// Colour calibration ('c') used to sample the exact single pixel under the
// crosshair, meaning that one noisy/compressed/slightly-off pixel is all it
// takes for the whole reference colour for that object to be
// off for as long as that callibration is active. Averaging a small patch around the
// crosshair instead helps to reduce the noise. Radius 2 = 5x5 = 25 pixels averaged.
#define CAL_SAMPLE_RADIUS  2

// Pixels at/above this on any channel are treated as clipped/saturated
// (a highlight, for example, the ball's top bright spot) and excluded
// from the calibration average instead of dragging it toward white.
#define CAL_CLIP_THRESH    2500

#endif