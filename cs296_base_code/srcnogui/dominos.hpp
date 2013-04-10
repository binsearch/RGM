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

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_
#include<iostream>
using namespace std;
namespace cs296
{

  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
  public:
    b2Body* rod;
    b2Body *head;
    b2Joint *neckJoint;
    
    dominos_t();
    void keyboard(unsigned char key) {
    //! This is used to change the position of the human
    //! get the current center positions of the human's body and head
    		float x=rod->GetPosition().x;
    		float y = rod->GetPosition().y;
    		float xh=head->GetPosition().x;
    		float yh = head->GetPosition().y;
    		float disp = 0.5;
    		//! destroy the joint joining them
    		m_world->DestroyJoint(neckJoint);
    		//! change their positions depending on the key pressed
    	if(key == 'a'){
    		
    		rod->SetTransform(b2Vec2(x - disp,y),0);
    		head->SetTransform(b2Vec2(xh - disp,yh),0);
    		
    	}	
    	else if(key == 'd'){
    		rod->SetTransform(b2Vec2(x + disp,y),0);
    		head->SetTransform(b2Vec2(xh + disp,yh),0);
    	}
    	//! create another joint using the new positions
    	b2RevoluteJointDef headJointDef;
	 	b2Vec2 neckPos;
	 	
	 	neckPos.Set(head->GetPosition().x,-22.0f+2*2.5);
	 	
	 	
	 	headJointDef.Initialize(rod,head,neckPos);
        	neckJoint=m_world->CreateJoint(&headJointDef);
    }
    static base_sim_t* create()
    {
      return new dominos_t;
    }
  };
}
  
#endif
