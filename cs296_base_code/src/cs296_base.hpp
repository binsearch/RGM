/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

  // What is the difference between a class and a struct in C++?
  
  class base_sim_t;
  
  struct settings_t;
  
  // Why do we use a typedef
  typedef base_sim_t* sim_create_fcn(); 
  
 /* class Human{
	public:
		b2Body* m_body;
		float m_radius;
		
		Human(b2World* world, float radius, float x, float y,float dens){
			m_body = NULL;
			m_radius = radius;
			
			//set up dynamic body, store in class variable
       			 b2BodyDef myBodyDef;
        		 myBodyDef.type = b2_dynamicBody;
        	  	 myBodyDef.position.Set(x,y);
        		 m_body = world->CreateBody(&myBodyDef);

        		//add circle fixture
       			 b2CircleShape circleShape;
        		circleShape.m_p.Set(0, 0);
        		circleShape.m_radius = m_radius; //use class variable
        		b2FixtureDef myFixtureDef;
        		myFixtureDef.shape = &circleShape;
        		myFixtureDef.density = dens;
        		m_body->CreateFixture(&myFixtureDef);
    		}
   };*/
  
  //! <B> Simulation settings. Some can be controlled in the GUI. </B>
  struct settings_t
  {
    // Notice the initialization of the class members in the constructor
    // How is this happening?
    //! Constructor used to set some initial values of some of the parameters of the GUI
    settings_t() :
      //!view_center(0.0f, 20.0f) - view_center is used to set the center point of the Graphical User Interface to the cartesian value (0.0f,20.0f)<br />
      view_center(0.0f, 20.0f),
      //!hz(60.0f) - Sets the default frames per second of the simulation to 60 fps<br />
      hz(60.0f),
      //! velocity_iterations(8) - Sets the number of velocity iterations(which is the number of times velocity of an objecy is changed in one time step due to collisions or other factors) to have an initial value of 8 <br />
      velocity_iterations(8),
      //! position_iterations(3) - Sets the number of positions iterations(which is the number of times position of an object is changed in one time step due to collisions or other factors) to have an initial value of 3<br />
      position_iterations(3),
      //! Draw the shapes onto the GUI since the value is set to 1. Can be modified from the GUI to 0 if we want all images to be hidden<br/>
      draw_shapes(1),
      //! draw_joints draws the joints onto the GUI if the value is set to 1(which it is). This can be modified from the GUI to 0 if we want all the joints to be hidden<br />
      draw_joints(1),
      //! draw_AABBs shows the AABBs(used for world-querying ,i.e., properties of the current simulated world) on the GUI if the value is set to 1(which it is not,and they are, therefore, hidden). This can be modified from the GUI to 1 if we want all the AABBs to be shown<br />
      draw_AABBs(0),
      //! draw_pairs shows the pairs on the GUI if the value is set to 1(which it is not,and they are, therefore, hidden). <br />
      draw_pairs(0),
      //! draw_contact_points, draw_contact_normals, draw_contact_forces, draw_friction_forces,shows the contact points,contact normals, contact forces and the friction forces respectively on the GUI if the value is set to 1(which it is not,and they are, therefore, hidden). <br />
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      //! draw_COMs shows the center of masses of the bodies if the value is set to 1. It is not going to show the center of masses of the bodies since the value is set to 0 <br />
      draw_COMs(0),
      //! draw_stats shows the statistics in the top left corner if the value is set to 1. It is not going to show the statistics since the value is set to 0 <br />
      draw_stats(0),
      //! draw_stats shows the profile of the simulation in the top left corner if the value is set to 1. It is not going to show the profile of the simulation since the value is set to 0 <br />
      draw_profile(0),
      //! If enable_warm_starting is set to 1, there is automatic velocity and position correction<br /> 
      enable_warm_starting(1),
      //! If enable_continous is set to 1, there is a continous variation in the velocities,positions and motion of the objects. <br />
      enable_continuous(1),
      //! enable_sub_stepping is set to 0, which indicates that it is not enabled. <br />
      enable_sub_stepping(0),
      //! pause is set to 0 to indicate that the simulation should not be paused. Value can be changed from the GUI during execution <br />
      pause(0),
      //! single_step is set to 0 to indicate that the simulation should not be run in a step-by-step fashion. Value can be changed from the GUI during execution <br />
      single_step(0)
    {}
    //! view_center is the position vector of the center of the simulation<br />
    b2Vec2 view_center;
    //! hz is 32 bit float value which denotes the frames per second(fps) of the simulation<br />
    float32 hz;
    //! velocity_iterations is  32 bit integer which denotes the velocity iterations in a time step<br />
    int32 velocity_iterations;
    //! position_iterations is  32 bit integer which denotes the position iterations in a time step<br />
    int32 position_iterations;
    //! draw_shapes - value is 1 if the shapes are to be drawn on the GUI,0 if not <br />
    int32 draw_shapes;
    //! draw_joints - value is 1 if the joints are to be drawn on the GUI,0 if not <br />
    int32 draw_joints;
    //! draw_AABBs - value is 1 if the AABBs are to be drawn on the GUI,0 if not <br />
    int32 draw_AABBs;
    //! draw_pairs - value is 1 if the pairs are to be drawn on the GUI,0 if not <br />
    int32 draw_pairs;
    //! draw_contact_points - value is 1 if the contact points are to be drawn on the GUI,0 if not <br />
    int32 draw_contact_points;
     //! draw_contact_normals - value is 1 if the contact normals are to be drawn on the GUI,0 if not <br />
    int32 draw_contact_normals;
     //! draw_contact_forces - value is 1 if the contact forces are to be drawn on the GUI,0 if not <br />
    int32 draw_contact_forces;
     //! draw_friction_forces - value is 1 if the friction forces are to be drawn on the GUI,0 if not <br />
    int32 draw_friction_forces;
    //! draw_COMs - value is 1 if the center of masses of the bodies are to be drawn on the GUI,0 if not <br />
    int32 draw_COMs;
    //! draw_stats - value is 1 if the statistics of simulation are to be shown on the GUI,0 if not <br />
    int32 draw_stats;
    //! draw_profile - value is 1 if the profile of the simulation are to be shown on the GUI,0 if not <br />
    int32 draw_profile;
    //! enable_warm_starting - value is 1 if warm starting is to be enabled,0 if not <br />
    int32 enable_warm_starting;
    //! enable_continous - value is 1 if there should be continous change in velocities,positions and motion of the bodies, 0 if not <br />
    int32 enable_continuous;
    //! enable_sub_stepping - value is 1 if sub stepping should be enabled, 0 if not <br />
    int32 enable_sub_stepping;
    //! pause - value is 1 if the simulation is to be paused,0 if it should be running. <br />
    int32 pause;
    //! single_step - value is 1 if the simulation is running step-by-step,0 if not,i.e., it should be running continously <br />
    int32 single_step;
  };
  //! This struct has been defined to store the name of the simulation along with certain other attributes like object of class base_sim_t <br />
  struct sim_t
  {
    //! name stores the name of the simlation <br />
    const char *name;
    //! create_fcn is the object of class base_sim_t <br />
    sim_create_fcn *create_fcn;

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}
  };
  
  extern sim_t *sim;
  
  
  const int32 k_max_contact_points = 2048; 
  //! This struct has been defined in order to properly specify attributes of a contact point<br />
   
  struct contact_point_t
  {
    //! fixtureA is one of 2 fixtures which are in contact with each other <br />
    b2Fixture* fixtureA;
    //! fixtureB is the second of 2 fixtures which are in contact with each other <br />
    b2Fixture* fixtureB;
    //! normal is the vector which specifies the normal contact between the 2 fixtures in contact mentioned above <br />
    b2Vec2 normal;
    //! position is the position vector which specifies the position/Cartesian co-ordinates of the contact point of the fixtures <br />
    b2Vec2 position;
    //! state is used to signify the state of the contact point of the 2 fixtures <br />
    b2PointState state;
  };
  //! The base_sim_t class has been defined to define objects related to contact points. This class is a sub-class of a pre-defined, in-built class b2ContactListener<br />
  class base_sim_t : public b2ContactListener
  {
  public:
    //! Default constructor which is used for initialising the properties of the simulation world object. Properties/Parameters include gravitational force,etc. <br />
    base_sim_t();

    // Virtual destructors - amazing objects. Why are these necessary?
    //! This is the virtual destructor. It's a seriously amazing object since it has the power to destroy the virtual world<br />
    virtual ~base_sim_t();
    //! This is just a function used to set a limit on the maximum number of lines which we can see on the GUI when profiles and statistics are being shown <br />
    void set_text_line(int32 line) { m_text_line = line; }
    //! Just used to write the string passes onto the GUI at the Cartesian co-ordinates passes to the function <br />
    void draw_title(int x, int y, const char *string);
    //! Stepping through time steps. <br />
    virtual void step(settings_t* settings);
    //! Some functions for performing some actions based on key presses,etc... <br />
    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }

    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }
    
    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
   
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
     
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    //! This is used to signify if a joint was destroyed or not. If a joint is not passed, no exception/error is encountered since we use B2_NOT_USED makes sure of it.<br />
    // Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    //! Callbacks for derived classes.
    //! Used to signify the beginning of a contact point. B2_NOT_USED is used for exception/error handling <br />
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
   //! Used to signify the release/end of a contact point. B2_NOT_USED is used for exception/error handling <br />
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    //! Precomputes parameters(like state of objects) before contact is established. B2_NOT_USED is used for exception/error handling <br />
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    //! Postcomputes parameter(like state of objects) after release of contact. B2_NOT_USED is used for exception/error handling <br />
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }
   //! An object which points to another object of type b2World. That object will be the simulated world <br />
    b2World* m_world;
  //How are protected members different from private memebers of a class in C++ ?
  protected:
    //! Friend class is a class which can use the private and protected members of it's friend class.<br />
    // What are Friend classes?
    //! In this case, it is to allow access to the contact point methods to contact_listerner_t class object<br />
    friend class contact_listener_t;
    //! This is a simple virtual body declared to act as the ground<br />
    b2Body* m_ground_body;
    //! m_world_AABB is declared which will store the AABB world-querying information of the simulated world <br />
    b2AABB m_world_AABB;
    //! An array of contact points with a max. capacity of k_max_contact_points <br />
    contact_point_t m_points[k_max_contact_points];
    //! A 32 bit integer to store the number of contact points <br />
    int32 m_point_count;
    //! This is used to render the shapes onto the GUI that will be visible during the simulation.<br />
    debug_draw_t m_debug_draw;
    //! A 32 bit integer which stores the number of text lines displayed on the GUI(Profile/Stats) <br />
    int32 m_text_line;
 
    //! The number of steps executed in the simulation <br />
    int32 m_step_count;
    //! Maximum Profile(Used for profile display on the GUI) <br />
    b2Profile m_max_profile;
    //! Total Profile(Used for profile display on the GUI) <br />
    b2Profile m_total_profile;
  };
}

#endif
