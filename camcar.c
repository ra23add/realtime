//======================================================================
//
// Test program to test the infrared sensors (and motors) of the
// 4tronix initio robot car. One can run this program within an
// ssh session.
//
// author: Raimund Kirner, University of Hertfordshire
//         initial version: Dec.2016
//
// license: GNU LESSER GENERAL PUBLIC LICENSE
//          Version 2.1, February 1999
//          (for details see LICENSE file)
//
// Compilation: 
// gcc -o camcar -Wall -Werror -lcurses -lwiringPi -lpthread -linitio camcar.c
//
//======================================================================

#include <stdlib.h>
#include <initio.h>
#include <curses.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <pthread.h>
#include <assert.h>

#include "detect_blob.h"

#include <stdbool.h>
#include <stdint.h>

//======================================================================
// Coursework ESD, general instructions
// This file (camcar.c) is the major file to be modified in order to 
// complete the coursework.  There are a few locations marked with "TODO",
// which indicate places where the code might need changes or completion.
// This directory also contains two other source files:
// quickblob.c ... this is a library for searching blobs in images
// detect_blob.c ... this is a wrapper for quickblob.c, providing a 
//                   direct interface to the RaspberryPI camera.
// Normally, quickblob.c and detect_blob.c don't need changes. However,
// studying detect_blob.c a bit is still advisable.
//
// The implementation of the nested state machine in camcar() follows
// the implementation proposal given in the Lecture slides. You may
// want to change the FSM implementation to add extra or refined
// behaviour.
//======================================================================

#define MIN_DISTANCE_US_SENSOR_CM 60
#define MAX_DISTANCE_US_SENSOR_CM 100

/* TODO: Experiment with the following macro value. */
#define BLOB_MIN_SIZE_THRESHOLD 20

/* TODO: Experiment with the following macro value. Must change from 0.15 to a "useful" value. */
/*   Tolerance/Leeway for the alignment between the car and found blob */
#define CAR_BLOB_ALIGNED_TOLERANCE 0.15
#define CAR_BLOB_ALIGNED_MIN_TOLERANCE -CAR_BLOB_ALIGNED_TOLERANCE
#define CAR_BLOB_ALIGNED_MAX_TOLERANCE CAR_BLOB_ALIGNED_TOLERANCE

/* The speed that the car should spin when in the "searching for blob" state */
#define CAR_BLOB_SEARCH_SPIN_SPEED 10
#define CAR_BLOB_REALIGN_SPINS_SPEED 5

#define MAX_DRIVE_SPEED 100

// data structure to communicate between main thread and camera thread
typedef struct ThreadCommunicationData {
  TBlobSearch blob;	// blob object from camera
  int recorded_blob_counter;		// record blob number (to know that a new image has been produced)
  bool should_exit; /* Boolean variable to keep track of thread termination state */
  /* TODO: decide whether Ultra Sonic sensor should be multi-threaded (like parSens) or left like the template has given */
  /* TODO: Based on the above decision, remove the Ultra Sonic threaded related code */
  int ahead_obstacle_distance_cm; /* Distance read from the ultra sonic sensor */
} ThreadCommunicationData_t;

pthread_mutex_t mutex_thread_data_camera;
typedef struct CamThreadCommuncationData {
    TBlobSearch search_blob;
    int recorded_blob_counter;
    bool should_exit;
} CamThreadCommuncationData_t;

pthread_mutex_t mutex_thread_data_us_sensor;
typedef struct USSensorThreadCommuncationData {
    bool should_exit;
    int ahead_obstacle_distance_cm;
} USSensorThreadCommuncationData_t;

typedef struct CarConfiguration {
    void (*spin_left_fn)(int8_t);
    void (*spin_right_fn)(int8_t);
    int8_t (*ir_left_censor_fn)(void);
    int8_t (*ir_right_censor_fn)(void);
    void (*drive_forward_fn)(int8_t speed);
    void (*drive_reverse_fn)(int8_t speed);
    double camera_rotation_radians;
} CarConfiguration_t;

CarConfiguration_t gen_default_car_config() {
    /* Generates a configuration for a perfect robot car */
    CarConfiguration_t config = {0};
    config.spin_left_fn = initio_SpinLeft;
    config.spin_right_fn = initio_SpinRight;
    config.ir_left_censor_fn = initio_IrLeft;
    config.ir_right_censor_fn = initio_IrRight;
    config.drive_forward_fn = initio_DriveForward;
    config.drive_reverse_fn = initio_DriveReverse;
    config.camera_rotation_radians = 0;
    return config;
}

void stop_car_now() {
  initio_DriveForward(0);
  // Or use 
  //initio_Stop()
}

pthread_mutex_t thread_data_mutex; // mutex to protect thread communication
int read_mutexed_distance(struct ThreadCommunicationData *data) {
  pthread_mutex_lock(&thread_data_mutex);
  int distance_cm = data->ahead_obstacle_distance_cm;
  pthread_mutex_unlock(&thread_data_mutex);
  return distance_cm;
}
void assign_mutexed_distance(struct ThreadCommunicationData *data, int distance_cm) {
  pthread_mutex_lock(&thread_data_mutex);
  data->ahead_obstacle_distance_cm = distance_cm;
  pthread_mutex_unlock(&thread_data_mutex);
}
TBlobSearch read_mutexed_blob(struct ThreadCommunicationData *data) {
    pthread_mutex_lock(&thread_data_mutex);
    TBlobSearch blob_data = data->blob;
    pthread_mutex_unlock(&thread_data_mutex);
    return blob_data;
}
void assign_mutexed_blob(struct ThreadCommunicationData *data, TBlobSearch blob) {
    /* Does a shallow copy of the `blob` aggregate to the thread `data` aggregate */
    pthread_mutex_lock(&thread_data_mutex);
    data->blob = blob;
    pthread_mutex_unlock(&thread_data_mutex);
}

/* Checks if the given distance is within the allowed distance */
bool is_US_distance_in_range(int distance_cm) {
  if (distance_cm > MIN_DISTANCE_US_SENSOR_CM && distance_cm < MAX_DISTANCE_US_SENSOR_CM) {
    return true;
  }
  return false;
}

// Thread function to measure distance with ultrasonic sensor
void *us_sensor_worker(void *p_thread_data_raw)
{
  struct ThreadCommunicationData *pthread_data = (struct ThreadCommunicationData *) p_thread_data_raw;
  while (!pthread_data->should_exit) {
    unsigned int distance_to_obj_cm = initio_UsGetDistance();
    assign_mutexed_distance(pthread_data, distance_to_obj_cm);
  }
  return NULL;
}

void ui_clear_remaining_cursor_line() {
    clrtoeol(); /* Curses: Clears everything from the right of the cursor
                   (inclusive) to blank characters */
}

/* TODO: Reason the validity of this code (maybe use `delay()` as commented out in the template) */
/* TODO: Reason the necessity of the function */
void wait_usec(long timer_usec) {
    /* Blocks the thread execution for at least `timer_usec` microseconds */
    usleep(timer_usec);
}

//======================================================================
// camcar():
// Skeleton code for the ESD coursework.
// The implementation uses hierarchical finite state machines (FSM), in order
// to reduce the size of the state transition graph.
// To be done: Fill the actions of the individual states of the FSMs
// with meaningful behaviour.
//======================================================================
void camcar(int argc, char *argv[], struct ThreadCommunicationData *ptdat) 
{
    int ch = 0;
    int recorded_blob_counter = 0;	// record blob nr of last movement
    TBlobSearch blob;	// blob object from camera thread
    CarConfiguration_t car_config = gen_default_car_config();

    // main control loop:  
    while (ch != 'q') {
        int obstacle_L, obstacle_R, obstacle; // FSM-OA
        int blobSufficient; // FSM-SB
        int carBlobAligned; // FSM-AB
        int us_sensor_distance_cm;
        enum { 
            FSM_US_SENSOR_DIST_OK, 
            FSM_US_SENSOR_DIST_TOO_CLOSE, 
            FSM_US_SENSOR_DIS_TOO_FAR
        } fsm_us_sensor_distance_state; /* Finite State Machine for the ultra sonic sensor distance */

        mvprintw(1, 1,"%s: Press 'q' to end program", argv[0]);
        mvprintw(10, 1,"Status: blob(size=%d, halign=%f, recorded_blob_counter=%u)  ", blob.size, blob.halign, ptdat->recorded_blob_counter);

        obstacle_L = ( car_config.ir_left_censor_fn() !=0 );
        obstacle_R = ( car_config.ir_right_censor_fn() !=0 );
        obstacle = obstacle_L || obstacle_R;

        // FSM-OA (Obstacle Avoidance)
        if (obstacle) {
            mvprintw(3, 1,"State OA (stop to avoid obstacle), o-left=%d, o-right=%d", obstacle_L, obstacle_R);
            ui_clear_remaining_cursor_line(); // curses library
            stop_car_now();
        }
        else {
            refresh(); // curses lib: update display

            TBlobSearch blob = read_mutexed_blob(ptdat);

            // writeImageWithBlobAsJPEG() seems to have a bug, do not use right now:
            // writeImageWithBlobAsJPEG(blob, "test_blob.jpg", 70);  // this function is for testing (deactivate if not needed)
            blobSufficient = (blob.size > BLOB_MIN_SIZE_THRESHOLD);

            // FSM-SB (Search Blob)
            if ( ! blobSufficient ) {
                mvprintw(3, 1,"State SB (search blob), blob.size=%d (recorded_blob_counter: %u)", blob.size, ptdat->recorded_blob_counter);
                ui_clear_remaining_cursor_line(); // curses library
                if (recorded_blob_counter < ptdat->recorded_blob_counter) {
                    // TODO: potential actions: turn car or camera platform a few steps around and see if a blob is to be found
                    // TODO: Understand what the above TODO is even talking about / referring to.
                    car_config.spin_left_fn(CAR_BLOB_SEARCH_SPIN_SPEED);
                    const long spin_wait_usec = 500000; /* 0.5 seconds */
                    wait_usec(spin_wait_usec);
                    stop_car_now();
                    recorded_blob_counter = ptdat->recorded_blob_counter;
                }
            } else {
                carBlobAligned = (blob.halign >= CAR_BLOB_ALIGNED_MIN_TOLERANCE
                        && blob.halign <= CAR_BLOB_ALIGNED_MAX_TOLERANCE);

                // FSM-AB (Align to Blob)
                if ( ! carBlobAligned) {
                    mvprintw(3, 1,"State AB (align towards blob), blob.size=%d, halign=%f", blob.size, blob.halign);
                    ui_clear_remaining_cursor_line(); // curses library
                    if (recorded_blob_counter < ptdat->recorded_blob_counter) {
                        /* TODO: Reconsider the following if-else statements, as the TODO asked for a "useful turn duration", which the following code does not use a "turn duration" */
                        if (blob.halign < 0) {
                            car_config.spin_right_fn(CAR_BLOB_REALIGN_SPINS_SPEED);
                        } else {
                            car_config.spin_left_fn(CAR_BLOB_REALIGN_SPINS_SPEED);
                        }
                        recorded_blob_counter = ptdat->recorded_blob_counter;
                    }
                } else {
                    us_sensor_distance_cm = initio_UsGetDistance ();
                    if (us_sensor_distance_cm < MIN_DISTANCE_US_SENSOR_CM) {
                        fsm_us_sensor_distance_state = FSM_US_SENSOR_DIST_TOO_CLOSE;
                    } else if (us_sensor_distance_cm > MAX_DISTANCE_US_SENSOR_CM) { 
                        fsm_us_sensor_distance_state = FSM_US_SENSOR_DIS_TOO_FAR;
                    } else {
                        fsm_us_sensor_distance_state = FSM_US_SENSOR_DIST_OK;
                    }
 
                    // FSM-MB (car at middle of blob, keep distance)
                    switch (fsm_us_sensor_distance_state) {
                    case FSM_US_SENSOR_DIS_TOO_FAR:
                        mvprintw(3, 1,"State FB (drive forward), dist=%d", us_sensor_distance_cm);
                        ui_clear_remaining_cursor_line(); // curses library
                        car_config.drive_forward_fn(MAX_DRIVE_SPEED);
                        break;
                    case FSM_US_SENSOR_DIST_TOO_CLOSE:
                        mvprintw(3, 1,"State RB (drive backwards), dist=%d", us_sensor_distance_cm);
                        ui_clear_remaining_cursor_line(); // curses library
                        car_config.drive_reverse_fn(MAX_DRIVE_SPEED);
                        break;
                    case FSM_US_SENSOR_DIST_OK:
                        mvprintw(3, 1,"State KD (keep us_sensor_distance_cm), dist=%d", us_sensor_distance_cm);
                        ui_clear_remaining_cursor_line(); // curses library
                        stop_car_now();
                    } // switch (FSM-MB)
                } // if (FSM-AB)
            } // if (FSM-SB)
        } // if (FSM-OA)

        ch = getch();
        if (ch != ERR) mvprintw(2, 1,"Key code: '%c' (%d)  ", ch, ch);
        refresh(); // curses lib: update display
        //delay (100); // pause 100ms
  } // while

  return;
}

const uint8_t g_red_color_rgb[] =   { 255, 0  , 0   };
const uint8_t g_green_color_rgb[] = { 0  , 255, 0   };
const uint8_t g_blue_color_rgb[] =  { 0  , 0  , 255 };

//======================================================================
// worker(): Thread function to continuously generate blob objects with camera
// This function will be executed by the explicitly created camera thread,
// to be executed concurrently with the main thread.
//======================================================================
void *worker(void *p_thread_data_raw)
{
  struct ThreadCommunicationData *p_thread_data = (struct ThreadCommunicationData *) p_thread_data_raw;
  const char blob_search_color_rgb[3] = {g_red_color_rgb[0], g_red_color_rgb[1], g_red_color_rgb[2]};  // color to be detected as blob
  TBlobSearch blob;	// blob object from camera
  while (!p_thread_data->should_exit == 0) {
    blob = cameraSearchBlob( blob_search_color_rgb ); // search for sign with colored blob
    assign_mutexed_blob(p_thread_data, blob);

    p_thread_data->recorded_blob_counter++;
  } // while
  return NULL;
}


//======================================================================
// main(): initialisation of libraries, etc
//======================================================================
int main (int argc, char *argv[])
{
  WINDOW *mainwin = initscr();  // curses: init screen
  noecho ();                    // curses: prevent the key being echoed
  cbreak();                     // curses: disable line buffering 
  nodelay(mainwin, TRUE);       // curses: set getch() as non-blocking 
  keypad (mainwin, TRUE);       // curses: enable detection of cursor and other keys

  initio_Init (); // initio: init the library

  pthread_t cam_thread;         // pthread: camera thread handle
  pthread_attr_t pt_attr;       // pthread: thread attributes
  struct ThreadCommunicationData cam_thread_com_data;
  cam_thread_com_data.recorded_blob_counter = 0;
  cam_thread_com_data.should_exit = false;
  pthread_attr_init(&pt_attr);  // pthread: create and init thread attribute
  pthread_mutex_init(&thread_data_mutex, NULL);
  pthread_create(&cam_thread, &pt_attr, worker, &cam_thread_com_data);

  camcar(argc, argv, &cam_thread_com_data);    // start control loop in main thread

  cam_thread_com_data.should_exit = true;               // signal thread to terminate
  pthread_join(cam_thread, NULL);

  pthread_attr_destroy(&pt_attr);

  initio_Cleanup ();  // initio: cleanup the library (reset robot car)
  endwin();           // curses: cleanup the library
  return EXIT_SUCCESS;
}

