
#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;
#include "dominos.hpp"
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

namespace cs296
{
  dominos_t::dominos_t()
  {	//ground
    	  b2Body* b1; 
    	{
    	  b2EdgeShape shape;
      	  shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));	
	
      	  b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }
        // Top horizontal shelf
   
   	 {
    
    	  b2PolygonShape shape;
     	 shape.SetAsBox(3.0f, 0.25f);//breadth,height
	
     	 b2BodyDef bd;
     	 bd.position.Set(-32.0f, 35.0f);
     	 b2Body* ground = m_world->CreateBody(&bd);
     	 ground->CreateFixture(&shape, 0.0f);
   	 }
        
        //Small red ball at the top
        {
        	b2Body* smallSphere;
        	b2CircleShape smallCircle;
        	smallCircle.m_radius = 0.8;
        	b2FixtureDef smallBallFixtureDef;
    		smallBallFixtureDef.shape = &smallCircle;
    		smallBallFixtureDef.density=1.0f;
    		smallBallFixtureDef.friction=0.0f;
    		smallBallFixtureDef.restitution = 0.0f;
    		b2BodyDef smallBallBodyDef;
    		smallBallBodyDef.type = b2_dynamicBody;
    		smallBallBodyDef.position.Set(-32.0f, 35.25f);
    		smallSphere = m_world->CreateBody(&smallBallBodyDef);
    		smallSphere->CreateFixture(&smallBallFixtureDef);
    		
    		b2Body* bigSphere;
    		b2CircleShape bigCircle;
    		bigCircle.m_radius = 1.8;
    	
    		b2FixtureDef bigBallFixtureDef;
    		bigBallFixtureDef.shape = &bigCircle;
    		bigBallFixtureDef.density=2.0f;
    		bigBallFixtureDef.friction=0.0f;
    		bigBallFixtureDef.restitution = 0.0f;
	        
	        b2BodyDef bigBallBodyDef;
	        bigBallBodyDef.type = b2_dynamicBody;
	        bigBallBodyDef.position.Set(-36.0f, 22);
	        bigSphere = m_world->CreateBody(&bigBallBodyDef);
	        bigSphere->CreateFixture(&bigBallFixtureDef);
        	
        	b2PulleyJointDef* fstpulley = new b2PulleyJointDef();
        	b2Vec2 worldAnchorGround1(-36.0f,36.05f);
        	
        	float32 ratio = 1.0f;
        	fstpulley->Initialize(smallSphere, bigSphere, worldAnchorGround1,worldAnchorGround1,  smallSphere->GetWorldCenter(), bigSphere->GetWorldCenter(), ratio);
        	m_world->CreateJoint(fstpulley);
        
        }
        
        
        {
        	b2Body* rod;
        	
        	b2PolygonShape rodShape;
        	rodShape.SetAsBox(0.3f,3);
        	
        	b2FixtureDef rodFixtureDef;
        	rodFixtureDef.shape = &rodShape;
        	
        	b2BodyDef rodBodyDef;
        	rodBodyDef.type= b2_dynamicBody;
     	 	rodBodyDef.position.Set(-36.0f, 33.0f);
     	 	rod  = m_world->CreateBody(&rodBodyDef);
     	 	rod->CreateFixture(&rodFixtureDef);
     	 	
     	 	b2Body* hinge;
     	 	
     	 	b2PolygonShape hingeShape;
     	 	hingeShape.SetAsBox(0.0f,0.0f);
     	 	
     	 	b2FixtureDef hingeFixtureDef;
     	 	hingeFixtureDef.shape = &hingeShape;
     	 	
     	 	b2BodyDef hingeBodyDef;
     	 	hingeBodyDef.position.Set(-36.0f,31.5f);
     	 	hinge = m_world->CreateBody(&hingeBodyDef);
     	 	hinge->CreateFixture(&hingeFixtureDef);
     	 	
     	 	b2RevoluteJointDef hingeJointDef;
     	 	
     	 	
     	 	hingeJointDef.Initialize(rod,hinge,b2Vec2(-36.0f,31.5f));
     	 	m_world->CreateJoint(&hingeJointDef);
        	
        }
        
        
        
        
       /* {
			float32 a = 2.0f;
			float32 b = 4.0f;
			float32 y = 16.0f;
			float32 L = 12.0f;

			b2PolygonShape shape;
			shape.SetAsBox(a, b);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			bd.position.Set(-10.0f, y);
			b2Body* body1 = m_world->CreateBody(&bd);
			body1->CreateFixture(&shape, 5.0f);

			bd.position.Set(10.0f, y);
			b2Body* body2 = m_world->CreateBody(&bd);
			body2->CreateFixture(&shape, 5.0f);

			b2PulleyJointDef pulleyDef;
			b2Vec2 anchor1(-10.0f, y + b);
			b2Vec2 anchor2(10.0f, y + b);
			b2Vec2 groundAnchor1(-10.0f, y + b + L);
			b2Vec2 groundAnchor2(10.0f, y + b + L);
			pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0f);

			m_world->CreateJoint(&pulleyDef);
		}*/
        
        
        
	    {	  
	        float vel;
	        vel = 7;  
		    b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(-10, 26.52); //set the starting position
			myBodyDef.angle = 0; //set the starting angle
			b2Body* dynamicBody = m_world->CreateBody(&myBodyDef);
			b2CircleShape circle;
            circle.m_radius = 0.5;
			
			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &circle;
			boxFixtureDef.density = 1;
			dynamicBody->CreateFixture(&boxFixtureDef);
			dynamicBody->SetLinearVelocity( b2Vec2( vel, 0 ) );
			myBodyDef.position.Set(-9, 26.52);
			b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
			dynamicBody1->CreateFixture(&boxFixtureDef);
			dynamicBody1->SetLinearVelocity( b2Vec2( vel, 0 ) );
			myBodyDef.position.Set(-12, 26.52);
			b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
			dynamicBody2->CreateFixture(&boxFixtureDef);
			dynamicBody2->SetLinearVelocity( b2Vec2( vel+0.5, 0 ) );
			
			myBodyDef.position.Set(-8, 26.52);
			b2Body* dynamicBody3 = m_world->CreateBody(&myBodyDef);
			dynamicBody3->CreateFixture(&boxFixtureDef);
			dynamicBody3->SetLinearVelocity( b2Vec2( vel, 0 ) );
			
			
			//dynamicBody->SetTransform( b2Vec2( 10, 20 ), 1 );
		
			
			//dynamicBody->SetTransform( b2Vec2( 10, 20 ), 45 * DEGTORAD ); //45 degrees counter-clockwise
			//dynamicBody->SetLinearVelocity( b2Vec2( -5, 5 ) ); //moving up and left 5 units per second
		    //dynamicBody->SetAngularVelocity( -90 * DEGTORAD ); //90 degrees per second clockwise
		 }
		
		
		/*	myBodyDef.type = b2_staticBody; //this will be a static body
  		myBodyDef.position.Set(0, 10); //slightly lower position
  		b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
  		staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
		
			myBodyDef.type = b2_kinematicBody; //this will be a kinematic body
  		myBodyDef.position.Set(-18, 11); // start from left side, slightly above the static body
  		b2Body* kinematicBody = m_world->CreateBody(&myBodyDef); //add body to world
  		kinematicBody->CreateFixture(&boxFixtureDef); //add fixture to body
  
  		kinematicBody->SetLinearVelocity( b2Vec2( 8, 0 ) ); //move right 1 unit per second
  		kinematicBody->SetAngularVelocity( 360 * DEGTORAD ); //1 turn per second counter-clockwise*/
  		//m_world->DestroyBody(staticBody);
  		
  		
  	
  	
  	
  	
  	//the dominos part
  		
    {
      float x,y;
      x = 0;
      y = 31.25;
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	  {
	    b2BodyDef bd;
	    bd.type = b2_dynamicBody;
	    bd.position.Set(x + 1.0 * i, y);
	    b2Body* body = m_world->CreateBody(&bd);
	    body->CreateFixture(&fd);
	  }
	  b2BodyDef bd;
	  shape.SetAsBox(0.1f, 3.0f);
	  bd.type = b2_dynamicBody;
	  bd.position.Set( x + 10.0, (y + 1.0));
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	  
	  
      shape.SetAsBox(7.0f, 0.25f);
      bd.position.Set(x + 4.5, y - 1.25);
      bd.type = b2_staticBody;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f); 
    }
  		
  	
  	//The revolving  platforms
    {
    
      float x,y;
      x = 3.0;
      y = 16.0;
      
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      b2BodyDef bd7;
      bd7.position.Set(20.5f, 18.0f);
      bd7.type = b2_dynamicBody;
      b2Body* body7 = m_world->CreateBody(&bd7);
      
      b2BodyDef bd8;
      bd8.position.Set(10.5f, 10.0f);
      bd8.type = b2_dynamicBody;
      b2Body* body8 = m_world->CreateBody(&bd8);
      
      
      b2BodyDef bd9;
      bd9.position.Set(14.0f, 6.0f);
      bd9.type = b2_dynamicBody;
      b2Body* body9 = m_world->CreateBody(&bd9);
      
      
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      body7->CreateFixture(fd);
      body8->CreateFixture(fd);
      body9->CreateFixture(fd);
      
      
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      
      
      
      b2BodyDef bd2;
      bd2.position.Set(x, y);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2BodyDef bd3;
      bd3.position.Set(x+2.5, y+4);
      b2Body* body3 = m_world->CreateBody(&bd3);
        
      b2BodyDef bd4;
      bd4.position.Set(x+2.5, y-4);
      b2Body* body4 = m_world->CreateBody(&bd4);
  
      b2BodyDef bd5;
      bd5.position.Set(x, y-8);
      b2Body* body5 = m_world->CreateBody(&bd5);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    
      b2RevoluteJointDef jointDef1;
      jointDef1.bodyA = body7;
      jointDef1.bodyB = body3;
      jointDef1.localAnchorA.Set(0,0);
      jointDef1.localAnchorB.Set(0,0);
      jointDef1.collideConnected = false;
      m_world->CreateJoint(&jointDef1);
    
      b2RevoluteJointDef jointDef2;
      jointDef2.bodyA = body8;
      jointDef2.bodyB = body4;
      jointDef2.localAnchorA.Set(0,0);
      jointDef2.localAnchorB.Set(0,0);
      jointDef2.collideConnected = false;
      m_world->CreateJoint(&jointDef2);
    
      b2RevoluteJointDef jointDef3;
      jointDef3.bodyA = body9;
      jointDef3.bodyB = body5;
      jointDef3.localAnchorA.Set(0,0);
      jointDef3.localAnchorB.Set(0,0);
      jointDef3.collideConnected = false;
      m_world->CreateJoint(&jointDef3);
    
    }
	
  		
  	//platform on which balls roll
  	
  	{
  	    float r,x,y;
  	    x = -5;
  	    y = 25;
  	    r = 0.5;
  	    {
  	        b2PolygonShape shape;
            shape.SetAsBox(3.0f, 0.01f);
	
            b2BodyDef bd;
            bd.position.Set(x-(r+3)-0.04, y+ 2 * r);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
  	    }
  	    
  	    
  	    {
  	        b2PolygonShape shape;
            shape.SetAsBox(3.0f, 0.01f);
	
            b2BodyDef bd;
            bd.position.Set(x +(r + 3)+0.04, y + 2 * r);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
  	    }
  	    
  	    
  	    {
      	    b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody;
            myBodyDef.position.Set(x, y); //middle
            b2Body* ditch = m_world->CreateBody(&myBodyDef);	
            
            b2PolygonShape polygonShape;
            b2FixtureDef myFixtureDef;
            myFixtureDef.shape = &polygonShape;
            myFixtureDef.density = 1;
      		
      		polygonShape.SetAsBox(r, 0.01f, b2Vec2(0,-2*r), 0 );
      		ditch->CreateFixture(&myFixtureDef);
      		polygonShape.SetAsBox(0.01f,2*r, b2Vec2(r+0.06,0), 0 );
      		ditch->CreateFixture(&myFixtureDef);
      		polygonShape.SetAsBox(0.01f,2 * r, b2Vec2(-r-0.06,0), 0 );
      		ditch->CreateFixture(&myFixtureDef);
        }  	
  	    
  	    {
      	    b2BodyDef myBodyDef;
            myBodyDef.type = b2_staticBody;
            myBodyDef.position.Set(x+6+r+r+0.2, y + r); //middle
            b2Body* ditch = m_world->CreateBody(&myBodyDef);	
            
            b2PolygonShape polygonShape;
            b2FixtureDef myFixtureDef;
            myFixtureDef.shape = &polygonShape;
            myFixtureDef.density = 1;
      		
      		polygonShape.SetAsBox(r, 0.01f, b2Vec2(0,-r), 0 );
      		ditch->CreateFixture(&myFixtureDef);
      		polygonShape.SetAsBox(0.01f, r, b2Vec2(r+0.1,0), 0 );
      		ditch->CreateFixture(&myFixtureDef);
      		polygonShape.SetAsBox(0.01f,r, b2Vec2(-(r+0.03),0), 0 );
      		ditch->CreateFixture(&myFixtureDef);
        }
         	    
  	
  	
  	}	
  		
	}

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
