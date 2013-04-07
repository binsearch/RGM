
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
      	  shape.Set(b2Vec2(-90.0f, -5.0f), b2Vec2(90.0f, -5.0f));	
	
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
        	b2BodyDef hingeBodyDef;
      		hingeBodyDef.position.Set(-36.0f, 33.5f);
     		b2Body* hinge = m_world->CreateBody(&hingeBodyDef);
      		
      		b2PolygonShape rodShape;
      		rodShape.SetAsBox(0.2f, 3.0f);
	
      
      		b2BodyDef rodBodyDef;
      		rodBodyDef.position.Set(-36.0f, 35.0f);
      		rodBodyDef.type = b2_dynamicBody;
      		b2Body* rod = m_world->CreateBody(&rodBodyDef);
      
     	        b2FixtureDef *rodFixtureDef = new b2FixtureDef;
      		rodFixtureDef->density = 2.5f;
      		rodFixtureDef->shape = new b2PolygonShape;
      		rodFixtureDef->shape = &rodShape;
      		rod->CreateFixture(rodFixtureDef);
      
     	        b2RevoluteJointDef jointDef;
      		jointDef.bodyA = rod;
      		jointDef.bodyB = hinge;
      		jointDef.localAnchorA.Set(0,-1.5f);
      		jointDef.localAnchorB.Set(0,0);
      		jointDef.lowerAngle = -1.1f*b2_pi;
      		jointDef.upperAngle = 1* b2_pi;
      		jointDef.enableLimit = true;
     		jointDef.collideConnected = false;
      		m_world->CreateJoint(&jointDef); 
        }
        int tr = 2;
        //big blue boulder
        {
		b2Body* blueBoulder;
		
		b2CircleShape boulderShape;
		boulderShape.m_radius = 1.2f;
		
		b2BodyDef boulderBodyDef;
		boulderBodyDef.type = b2_dynamicBody;
		boulderBodyDef.position.Set(-34.0f,30.0f);
		
		b2FixtureDef boulderFixtureDef;
    		boulderFixtureDef.shape = &boulderShape;
    		boulderFixtureDef.density=1.5f;
    		boulderFixtureDef.friction=80.5f;
    		boulderFixtureDef.restitution = 0.0f;
    		
    		blueBoulder = m_world->CreateBody(&boulderBodyDef);
    		blueBoulder->CreateFixture(&boulderFixtureDef);
        	
        	b2Body* boulderGrdBody;
        	
        	b2PolygonShape grd;
        	grd.SetAsBox(2.0f,0.1f);
        	
        	b2BodyDef grdBodyDef;
        	grdBodyDef.position.Set(-33.60f,28.8f);
        	
        	b2FixtureDef grdFixtureDef;
        	grdFixtureDef.shape=&grd;
        	grdFixtureDef.density=1.0f;
        	grdFixtureDef.friction=81.5f;
        	grdFixtureDef.restitution=0.0;
        	
        	boulderGrdBody = m_world->CreateBody(&grdBodyDef);
        	boulderGrdBody->CreateFixture(&grdFixtureDef); 
        }
        
        
        
	/* {
			float32 a = 2.0f;
			float32 b = 2.0f;
			float32 y = 20.8f;
			float32 L = 12.0f;

			b2PolygonShape shape;
			shape.SetAsBox(a+tr, b);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			bd.position.Set(-28.4f+tr, y);
			b2Body* body1 = m_world->CreateBody(&bd);
			body1->CreateFixture(&shape, 5.0f);

			bd.position.Set(-20.4f+tr, y);
			b2Body* body2 = m_world->CreateBody(&bd);
			body2->CreateFixture(&shape, 5.0f);

			b2PulleyJointDef pulleyDef;
			b2Vec2 anchor1(-28.4f+tr, y + b);
			b2Vec2 anchor2(-20.4f+tr, y + b);
			b2Vec2 groundAnchor1(-28.4f+tr, y + b + L);
			b2Vec2 groundAnchor2(-20.4f+tr, y + b + L);
			pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.0f);

			m_world->CreateJoint(&pulleyDef);
		}*/
		
	int diff=32.6-28.0;	
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(tr+3.2,8.0f);
      bd->fixedRotation = true;
      
      //! The open box is created using 3 rectangles joined to form an open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      //! The open box is set to be shown with certain attributes like density to be 10,friction co-efficient as 0.5
      fd1->density = 10.0;
      fd1->friction = 0.5;
      //! Co-effecient of restitution to be 0
      fd1->restitution = 0.f;
      //! The shape is declared which is then later defined to be a rectangle of width 4,height 0.4, center of box to have co-ordinates to be(0,-1.9) and density 0 
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2.0,0.2, b2Vec2(-27.9f-diff,11.8f), 0);//hz
      fd1->shape = &bs1;
      //! Another rectangle is defined to have same density,friction and restitution as the previous rectangle but the width is 0.4,height is 4, rectangle's center is located at (2,0) with density 0
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2.5, b2Vec2(-29.65f-diff,14.15f), 0);//vertical
      fd2->shape = &bs2;
      //! Another rectangle is defined to have same density,friction and restitution as the previous 2 rectangles but the width is 0.4,height is 4, rectangle's center is located at (-2,0) with density 0
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2.5, b2Vec2(-26.15f-diff,14.15f), 0);//vertical
      fd3->shape = &bs3;
       //! The 3 boxes are joined together using CreateFixture and shown.
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //! The bar is a rectangle with it's center's co-ordinates to be (10,15), density 34 and then the bar is displayed on screen using CreateFixture
      {
     
      bd->position.Set(-18,30);	
      b2FixtureDef *openFD = new b2FixtureDef;
      openFD->density = 26.3;
      openFD->friction = 0.5;
      openFD->restitution = 0.f;
      openFD->shape = new b2PolygonShape;
      b2PolygonShape openbs;
      openbs.SetAsBox(0.2,2.7);//vertical
      openFD->shape = &openbs;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(openFD);
       //! The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      //b2Vec2 worldAnchorOnBody1(-10, 15); //! Anchor point on body 1 in world axis
      //b2Vec2 worldAnchorOnBody2(10, 15); //! Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-27.0, 40); //! Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-18, 40); //! Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; //! Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
      }
      
      
      { 
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(tr+12.2,8.5f);
      bd->fixedRotation = true;
      
      //! The open box is created using 3 rectangles joined to form an open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      //! The open box is set to be shown with certain attributes like density to be 10,friction co-efficient as 0.5
      fd1->density = 1.0;
      fd1->friction = 0.5;
      //! Co-effecient of restitution to be 0
      fd1->restitution = 0.f;
      //! The shape is declared which is then later defined to be a rectangle of width 4,height 0.4, center of box to have co-ordinates to be(0,-1.9) and density 0 
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2.0,0.2, b2Vec2(-27.9f-diff,11.8f), 0);//hz
      fd1->shape = &bs1;
      //! Another rectangle is defined to have same density,friction and restitution as the previous rectangle but the width is 0.4,height is 4, rectangle's center is located at (2,0) with density 0
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2.5, b2Vec2(-29.65f-diff,14.15f), 0);//vertical
      fd2->shape = &bs2;
      //! Another rectangle is defined to have same density,friction and restitution as the previous 2 rectangles but the width is 0.4,height is 4, rectangle's center is located at (-2,0) with density 0
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 1.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2.5, b2Vec2(-26.15f-diff,14.15f), 0);//vertical
      fd3->shape = &bs3;
       //! The 3 boxes are joined together using CreateFixture and shown.
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
     
      box1->SetGravityScale(0.01);
      }
      
      {
      	b2BodyDef myBodyDef;
	myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
	myBodyDef.position.Set(-20.5, 30.92); //set the starting position
	myBodyDef.angle = 0; //set the starting angle
	b2Body* dynamicBody = m_world->CreateBody(&myBodyDef);
	b2CircleShape circle;
        circle.m_radius = 0.6;
			
	b2FixtureDef boxFixtureDef;
	boxFixtureDef.shape = &circle;
	boxFixtureDef.density = 0.21;
	dynamicBody->CreateFixture(&boxFixtureDef);
	dynamicBody->SetLinearVelocity( b2Vec2( -0.05, -0.5 ) );
	myBodyDef.position.Set(-20.7, 30.72);
	b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
	dynamicBody1->CreateFixture(&boxFixtureDef);
	dynamicBody1->SetLinearVelocity( b2Vec2( -0.05, -0.5) );
	myBodyDef.position.Set(-20.5, 31.32);
	b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
	dynamicBody2->CreateFixture(&boxFixtureDef);
	dynamicBody2->SetLinearVelocity( b2Vec2( -0.05, -0.5 ) );
	
	myBodyDef.position.Set(-20.5, 32.52);
	b2Body* dynamicBody3 = m_world->CreateBody(&myBodyDef);
	dynamicBody3->CreateFixture(&boxFixtureDef);
	dynamicBody3->SetLinearVelocity( b2Vec2( -0.05, -0.5 ) );	
      }
      //static platform
      {
      bd->type = b2_kinematicBody;
      bd->position.Set(-20.2,29);	
      bd->angle = +(b2_pi)*0.15;
      b2FixtureDef *openFD = new b2FixtureDef;
      openFD->density = 10.0;
      openFD->friction = 0.5;
      openFD->restitution = 0.f;
      openFD->shape = new b2PolygonShape;
      b2PolygonShape openbs;
      openbs.SetAsBox(0.2,2.8);//vertical
      openFD->shape = &openbs;
      b2Body* box2 = m_world->CreateBody(bd);
      
      box2->CreateFixture(openFD);
     
      }
      
       {
      //!task1 -- The triangle wedge
      b2Body* sbody;//! sbody -- b2Body variable for wedge
      b2PolygonShape poly;//! shape declared as polygon.
      b2Vec2 vertices[3];//! vertices -- b2vec2 arrayv which contains the vertices for wedge.
      vertices[0].Set(-1,0);//! vertices[0] set to (-1,0).
      vertices[1].Set(1,0);//! vertices[1] set to (1,0).
      vertices[2].Set(0,1.5);//! vertices[2] set to (0,1.5).
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;//! wedgefd -- b2FixtureDef variable for fixture definition.
      wedgefd.shape = &poly;//! shape defined as polygon.
      wedgefd.density = 10.0f;//! density set to 10.0f
      wedgefd.friction = 0.0f;//! friction set to 0.0f
      wedgefd.restitution = 0.0f;//! restitution set to 0.0f
      b2BodyDef wedgebd;//! wedgebd -- b2BodyDef varaible.
      wedgebd.position.Set(30.0f, 0.0f);//! position set to 30.0f,0.0f
      sbody = m_world->CreateBody(&wedgebd);//! triangle wedge is created.
      sbody->CreateFixture(&wedgefd);

      //!task2 -- The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);//! shape set to box with dimensions (30.0f,0.4f).
      b2BodyDef bd2;//! bd2 -- b2BodyDef variable defined.
      bd2.position.Set(30.0f, 1.5f);//!position set to (30.0f , 1.5f)
      bd2.type = b2_dynamicBody;//! the type of the body is set to b2_dynamicBody as it will be moving.
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;//! fd2 -- pointer to b2FixtureDef variable.
      fd2->density = 1.f;//! density set to 1.f
      fd2->shape = new b2PolygonShape;//! shape set to polygon
      fd2->shape = &shape;
      body->CreateFixture(fd2);//! plank object is created.

      b2RevoluteJointDef jd;//! jd -- b2RevolutejointDef variable for joint.
      b2Vec2 anchor;//! anchor declared.
      anchor.Set(30.0f, 1.5f);//!anchor position set to (30.0f, 1.5f)
      jd.Initialize(sbody, body, anchor);//! joint is intialised with the menbers anchor,body , sbody.
      m_world->CreateJoint(&jd);

      //! The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);//! shape set to box with dimensions 4.0f,4.0f)
      b2BodyDef bd3;//bd3 -- b2BodyDef variable .
      bd3.position.Set(40.0f, 2.0f);//! position set to (40.0f,2.0f)
      bd3.type = b2_dynamicBody;//! body type set to b2_dynamicBody
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;//! fd3 -- pointer to b2FixtureDef variable.
      fd3->density = 0.01f;//! density set to 0.01f
      fd3->shape = new b2PolygonShape;//! shape set to polygon
      fd3->shape = &shape2;//! shape2 assigned as the shape
      body3->CreateFixture(fd3);//! fixture assigned to body3.
    }
      
     
    }
		
		
        
        
        int sftx=20;
	    {	  
	        float vel;
	        vel = 7;  
		    b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(-10+sftx, 26.52); //set the starting position
			myBodyDef.angle = 0; //set the starting angle
			b2Body* dynamicBody = m_world->CreateBody(&myBodyDef);
			b2CircleShape circle;
            		circle.m_radius = 0.5;
			
			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &circle;
			boxFixtureDef.density = 1;
			dynamicBody->CreateFixture(&boxFixtureDef);
			dynamicBody->SetLinearVelocity( b2Vec2( vel, 0 ) );
			myBodyDef.position.Set(-9+sftx, 26.52);
			b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
			dynamicBody1->CreateFixture(&boxFixtureDef);
			dynamicBody1->SetLinearVelocity( b2Vec2( vel, 0 ) );
			myBodyDef.position.Set(-12+sftx, 26.52);
			b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
			dynamicBody2->CreateFixture(&boxFixtureDef);
			dynamicBody2->SetLinearVelocity( b2Vec2( vel+0.5, 0 ) );
			
			myBodyDef.position.Set(-8+sftx, 26.52);
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
      bd.position.Set(14.0f+sftx, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      b2BodyDef bd7;
      bd7.position.Set(20.5f+sftx, 18.0f);
      bd7.type = b2_dynamicBody;
      b2Body* body7 = m_world->CreateBody(&bd7);
      
      b2BodyDef bd8;
      bd8.position.Set(10.5f+sftx, 10.0f);
      bd8.type = b2_dynamicBody;
      b2Body* body8 = m_world->CreateBody(&bd8);
      
      
      b2BodyDef bd9;
      bd9.position.Set(14.0f+sftx, 6.0f);
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
      bd2.position.Set(x+sftx, y);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2BodyDef bd3;
      bd3.position.Set(x+2.5+sftx, y+4);
      b2Body* body3 = m_world->CreateBody(&bd3);
        
      b2BodyDef bd4;
      bd4.position.Set(x+2.5+sftx, y-4);
      b2Body* body4 = m_world->CreateBody(&bd4);
  
      b2BodyDef bd5;
      bd5.position.Set(x+sftx, y-8);
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
  	    x = -5+sftx;
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
