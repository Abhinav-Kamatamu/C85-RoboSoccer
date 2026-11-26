/***************************************************
 CSC C85 RoboSoccer frame recorder (images half of the run log)

 Saves what the window shows, so we (or an AI) can see frame by frame
 what the robot saw. Text logs are logic/runlog.c; this file only does
 pictures, and does all PNG encoding + disk writes on its own thread so
 the 30 fps loop only pays for a memcpy.

   frames/f000123.png    5 fps snapshots of the display (1024x768)
   raw/f000123.png       1 fps raw camera frames            (logItems bit2)
   events/E003_f000456_P_UNSTICK/
     event.json          what triggered it + every frame in the window
     display/f*.png      every frame from 2 s before to 1 s after (512x384)
     field/f*.png        unwarped field = blob detector input (bit2, 15 fps)

 An event is any transition into a wall-recover/unstick/lost/strike/
 shot-done state, any pre-empt, the 'b' bookmark key, and quitting.
***************************************************/

#ifndef IMAGECAPTURE_FRAMELOG_H
#define IMAGECAPTURE_FRAMELOG_H

struct image;
struct blob;

// items = the bits runlog_start() returned. field_w/h = camera size (sx, sy).
// Registers framelog_stop() with atexit().
void framelog_start(int items, const char *run_dir, int field_w, int field_h);

// Once per frame, after the display buffer is filled. display = the 1024x768
// RGB image the window shows; field/raw = sx*sy RGB (unwarped field, camera).
// kind: 0 = raw camera on screen (not calibrated), 1 = field, 2 = blobs.
void framelog_frame(int frame_no, double t, const unsigned char *display, int kind,
                    const unsigned char *field, const unsigned char *raw);

// Hand this frame's blob list to the text log (frames.jsonl).
void framelog_log_blobs(struct blob *list);

// Draw the robot's intent (heading, staging point, aim line) onto the blob
// image, so the pictures show what it was TRYING to do. Display logging only.
void framelog_overlay(struct image *blob_im);

// Dumps any pending event, waits for the writer to finish. Safe to call twice.
void framelog_stop(void);

#endif
