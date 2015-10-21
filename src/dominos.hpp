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
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_
#include "cs251_base.hpp"
//#include "callbacks.cpp"
#include "render.hpp"
#ifndef __APPLE__
#include "GL/glui.h"
#else
#include "GL/glui.h"
#endif





namespace cs251
{
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?



  class dominos_t : public base_sim_t//,public callbacks_t
  {

  public:
   //b2Body* sbody;
   //b2Vec2 vel;
    //static b2Body* body;
     //static b2Body* ssbody;static b2BodyDef swedgebd;static b2FixtureDef swedgefd;
    //! This is the constructor of dominos_t
    dominos_t();

   // void d() {body->SetLinearVelocity(sbody->GetLinearVelocity());}
   //! This is defination of static function which is invoked on pressing certain special key.
 static void Key_board(unsigned char key,int32 x,int32 y);

  //sbody->SetLinearVelocity(vel);
  //! this is defination of a static function which is invoked on pressing mouse button.
 static void sed(int32 button,int32 state,int x32,int32 y);



    //static void Keyboard(unsigned char key,int x,int y);
    //! This creates the box2d simulation.
    static base_sim_t* create()
    {
    //! this is function which creates box2d simulation
      return new dominos_t;
    }
  };
}


#endif
