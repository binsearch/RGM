/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

//! These are user defined include files
//! Included in double quotes - the path to find these has to be given at compile time
#include "render.hpp"
#include "cs296_base.hpp"
#include "callbacks.hpp"
#include <sys/time.h>
#include <cstdio>
#include <iostream>
using namespace std;
//! GLUI is the library used for drawing the GUI
//! Learn more about GLUI by reading the GLUI documentation
//! Learn to use preprocessor diectives to make your code portable
#ifndef __APPLE__
#include "GL/glui.h"
#else
#include "GL/glui.h"
#endif

//! These are standard include files
//! These are usually available at standard system paths like /usr/include
//! Read about the use of include files in C++
#include <cstdio>


//! Notice the use of extern. Why is it used here?
namespace cs296
{
  extern int32 test_index;
  extern int32 test_selection;
  extern int32 test_count;
  extern cs296::sim_t* entry;
  extern cs296::base_sim_t* test;
  extern cs296::settings_t settings;
  extern const int32 frame_period;
  extern float settings_hz;
  extern int32 width;
  extern int32 height;
  extern int32 main_window;
};

//! This opens up the cs296 namespace
//! What is the consequence of opening up a namespace?
using namespace cs296;


//! This function creates all the GLUI gui elements
void create_glui_ui(void)
{
  GLUI *glui = GLUI_Master.create_glui_subwindow( main_window, GLUI_SUBWINDOW_BOTTOM );
  
  glui->add_statictext("Simulation Timesteps"); 
  GLUI_Spinner* velocityIterationSpinner =
    glui->add_spinner("Velocity Iterations", GLUI_SPINNER_INT, &settings.velocity_iterations);
  velocityIterationSpinner->set_int_limits(1, 500);
  
  GLUI_Spinner* positionIterationSpinner =
    glui->add_spinner("Position Iterations", GLUI_SPINNER_INT, &settings.position_iterations);
  positionIterationSpinner->set_int_limits(0, 100);
  
  GLUI_Spinner* hertzSpinner =
    glui->add_spinner("Sim steps per frame", GLUI_SPINNER_FLOAT, &settings_hz);
  hertzSpinner->set_float_limits(5.0f, 200.0f);


  
  new GLUI_Column( glui, false );
  glui->add_statictext("Simulation Parameters"); 
  glui->add_checkbox("Warm Starting", &settings.enable_warm_starting);
  glui->add_checkbox("Time of Impact", &settings.enable_continuous);
  glui->add_checkbox("Sub-Stepping", &settings.enable_sub_stepping);


  
  new GLUI_Column( glui, false );
  glui->add_statictext("Display Options"); 
  GLUI_Panel* drawPanel =	glui->add_panel("Draw");
  glui->add_checkbox_to_panel(drawPanel, "Shapes", &settings.draw_shapes);
  glui->add_checkbox_to_panel(drawPanel, "Joints", &settings.draw_joints);
  glui->add_checkbox_to_panel(drawPanel, "AABBs", &settings.draw_AABBs);
  glui->add_checkbox_to_panel(drawPanel, "Statistics", &settings.draw_stats);
  glui->add_checkbox_to_panel(drawPanel, "Profile", &settings.draw_profile);
  
  new GLUI_Column( glui, false );
  glui->add_button("Pause", 0, callbacks_t::pause_cb);
  glui->add_button("Single Step", 0, callbacks_t::single_step_cb);
  glui->add_button("Restart", 0, callbacks_t::restart_cb);
  
  glui->add_button("Quit", 0,(GLUI_Update_CB)callbacks_t::exit_cb);
  glui->set_main_gfx_window( main_window );
}


//! This is the main function
int main(int argc, char** argv)
{
  test_count = 1;
  test_index = 0;
  test_selection = test_index;
  
  entry = sim;
  test = entry->create_fcn();

  //! This initializes GLUT
  int numIter = atol (argv[1]);
  b2World* m_world1 = test->get_world( );
  
  float32 time_step = settings.hz > 0.0f ? 1.0f / settings.hz : float32( 0.0f );
  double time_total;
  float32 timeStep = 0, time_collisions = 0, time_velocity_updates = 0,time_position_updates = 0;
  
  struct timeval startTime,endTime;
  gettimeofday(&startTime , NULL);
  
 
  for(int i=0;i<numIter;i++){  	
  	m_world1->Step(time_step, settings.velocity_iterations, settings.position_iterations); 	
  	timeStep += (m_world1->GetProfile()).step;
  	time_collisions += (m_world1->GetProfile()).collide;
  	time_velocity_updates += (m_world1->GetProfile()).solveVelocity;
  	time_position_updates += (m_world1->GetProfile()).solvePosition;
  }
  gettimeofday( &endTime, NULL);
  time_total = ( (endTime.tv_sec - startTime.tv_sec)*1000000.0 + (endTime.tv_usec - startTime.tv_usec))/1000.0;
  timeStep /= numIter;
  time_collisions /= numIter;
  time_velocity_updates /= numIter;
  time_position_updates /= numIter;
  
  cout<<"Total Iterations: "<< numIter <<endl;
  cout<<"Average time per step is "<<timeStep<<" ms"<<endl;
  cout<<"Average time for collisions is "<<time_collisions<<" ms"<<endl;
  cout<<"Average time for velocity updates is "<<time_velocity_updates<<" ms"<< endl;
  cout<<"Average time for position updates is "<<time_position_updates<<" ms"<< endl<<endl;
  cout<<"Total time for loop is "<<time_total <<" ms"<<endl;
  
  //cout<<" ms"<<endl<<"This is from the source code branch made by Tarun Kathuria from Group 19 for CS296 Lab 04."<<endl;
  return 0;
}
