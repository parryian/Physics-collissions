#include <GL/glut.h>
//#include <math.h>
#include "world.h"

#include <stdlib.h>

static float angle=0.0,ratio;
static float x=0.0f,y=1.75f,z=5.0f;
static float lx=0.0f,ly=0.0f,lz=-1.0f;
World *myWorld;
wtimer myTimer;



void changeSize(int w, int h)
	{

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if(h == 0)
		h = 1;

	ratio = 1.0f * w / h;
	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Set the viewport to be the entire window
    glViewport(0, 0, w, h);

	// Set the clipping volume
	gluPerspective(45,ratio,1,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(x, y, z,
		      x + lx,y + ly,z + lz,
			  0.0f,1.0f,0.0f);


	}





float rand_FloatRange(float a, float b)
{
return ((b-a)*((float)rand()/RAND_MAX))+a;
}

// create world and add entitys here
void initScene() {
     //myTimer = wtimer();



  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);



	delete myWorld;
	myWorld = new World;
	std::vector<Vector3f> points;
	Entity ent;

		points.push_back(Vector3f(-100.0f, 0.0f, -100.0f));
		points.push_back(Vector3f(-100.0f, 0.0f,  100.0f));
		points.push_back(Vector3f( 100.0f, 0.0f,  100.0f));
		points.push_back(Vector3f( 100.0f, 0.0f, -100.0f));
		ent = Entity(points, "ground", -1.0f);
		ent.colour = Vector3f(0.9f, 0.9f, 0.9f);
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-100.0f, 20.0f, -100.0f));
		points.push_back(Vector3f(-100.0f, 0.0f, -100.0f));
		points.push_back(Vector3f( 100.0f, 0.0f, -100.0f));
		points.push_back(Vector3f( 100.0f, 20.0f, -100.0f));
		ent = Entity(points, "wall1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
        points.push_back(Vector3f(100.0f, 20.0f, -100.0f));
		points.push_back(Vector3f(100.0f, 0.0f, -100.0f));
		points.push_back(Vector3f( 100.0f, 0.0f, 100.0f));
		points.push_back(Vector3f( 100.0f, 20.0f, 100.0f));
		ent = Entity(points, "wall2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
        points.push_back(Vector3f(100.0f, 20.0f, 100.0f));
		points.push_back(Vector3f(100.0f, 0.0f, 100.0f));
		points.push_back(Vector3f( -100.0f, 0.0f, 100.0f));
		points.push_back(Vector3f( -100.0f, 20.0f, 100.0f));
		ent = Entity(points, "wall3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-100.0f, 20.0f, 100.0f));
		points.push_back(Vector3f(-100.0f, 0.0f, 100.0f));
		points.push_back(Vector3f( -100.0f, 0.0f, -100.0f));
		points.push_back(Vector3f( -100.0f, 20.0f, -100.0f));
		ent = Entity(points, "wall4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);


		ent = Entity(Vector3f(20.0f,0.0f, 20.0f), 10, "hemisphere", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-80.0f, 30.0f, -10.0f));
		points.push_back(Vector3f(-80.0f, 30.0f, 0.0f));
		points.push_back(Vector3f( -40.0f, 30.0f, 0.0f));
		points.push_back(Vector3f( -40.0f, 30.0f, -10.0f));
		ent = Entity(points, "p1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-80.0f, 30.0f, 0.0f));
		points.push_back(Vector3f(-80.0f, 0.0f, 0.0f));
		points.push_back(Vector3f( -40.0f, 0.0f, 0.0f));
		points.push_back(Vector3f( -40.0f, 30.0f, 0.0f));
		ent = Entity(points, "p2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-80.0f, 30.0f, -10.0f));
		points.push_back(Vector3f(-80.0f, 0.0f, -10.0f));
		points.push_back(Vector3f( -80.0f, 0.0f, 0.0f));
		points.push_back(Vector3f( -80.0f, 30.0f, 0.0f));
		ent = Entity(points, "p3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-40.0f, 30.0f, -10.0f));
		points.push_back(Vector3f(-40.0f, 0.0f, -10.0f));
		points.push_back(Vector3f( -80.0f, 0.0f, -10.0f));
		points.push_back(Vector3f( -80.0f, 30.0f, -10.0f));
		ent = Entity(points, "p4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(-40.0f, 30.0f, 0.0f));
		points.push_back(Vector3f(-40.0f, 0.0f, 0.0f));
		points.push_back(Vector3f( -40.0f, 0.0f, -10.0f));
		points.push_back(Vector3f( -40.0f, 30.0f, -10.0f));
		ent = Entity(points, "p5", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);


		points.clear();
		points.push_back(Vector3f(10.0f, 10.0f, -70.0f));
		points.push_back(Vector3f( 0.0f, 0.0f, -60.0f));
		points.push_back(Vector3f( 20.0f, 0.0f, -60.0f));
		ent = Entity(points, "py1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

        points.clear();
		points.push_back(Vector3f(10.0f, 10.0f, -70.0f));
		points.push_back(Vector3f( 0.0f, 0.0f, -80.0f));
		points.push_back(Vector3f( 0.0f, 0.0f, -60.0f));
		ent = Entity(points, "py2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(10.0f, 10.0f, -70.0f));
		points.push_back(Vector3f( 20.0f, 0.0f, -80.0f));
		points.push_back(Vector3f( 0.0f, 0.0f, -80.0f));
		ent = Entity(points, "py3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f(10.0f, 10.0f, -70.0f));
		points.push_back(Vector3f( 20.0f, 0.0f, -60.0f));
		points.push_back(Vector3f( 20.0f, 0.0f, -80.0f));
		ent = Entity(points, "py4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 60.0f, 0.0f, -50.0f));
		points.push_back(Vector3f( 60.0f, 0.0f, -40.0f));
		points.push_back(Vector3f( 70.0f, 10.0f, -40.0f));
		points.push_back(Vector3f( 70.0f, 10.0f, -50.0f));
		ent = Entity(points, "f1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 60.0f, 0.0f, -40.0f));
		points.push_back(Vector3f( 70.0f, 0.0f, -40.0f));
		points.push_back(Vector3f( 70.0f, 10.0f, -40.0f));
		ent = Entity(points, "f2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 70.0f, 10.0f, -40.0f));
		points.push_back(Vector3f( 70.0f, 0.0f, -40.0f));
		points.push_back(Vector3f( 70.0f, 0.0f, -50.0f));
		points.push_back(Vector3f( 70.0f, 10.0f, -50.0f));
		ent = Entity(points, "f3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 70.0f, 10.0f, -50.0f));
		points.push_back(Vector3f( 70.0f, 0.0f, -50.0f));
		points.push_back(Vector3f( 60.0f, 0.0f, -50.0f));
		ent = Entity(points, "f4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -10.0f, 20.0f, 10.0f));
		points.push_back(Vector3f( -10.0f, 20.0f, 30.0f));
		points.push_back(Vector3f( 10.0f, 20.0f, 30.0f));
		points.push_back(Vector3f( 10.0f, 20.0f, 10.0f));
		ent = Entity(points, "bc1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -10.0f, 20.0f, 30.0f));
		points.push_back(Vector3f( -10.0f, 0.0f, 30.0f));
		points.push_back(Vector3f( 10.0f, 0.0f, 30.0f));
		points.push_back(Vector3f( 10.0f, 20.0f, 30.0f));
		ent = Entity(points, "bc2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -10.0f, 20.0f, 10.0f));
		points.push_back(Vector3f( -10.0f, 0.0f, 10.0f));
		points.push_back(Vector3f( -10.0f, 0.0f, 30.0f));
		points.push_back(Vector3f( -10.0f, 20.0f, 30.0f));
		ent = Entity(points, "bc3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 10.0f, 20.0f, 10.0f));
		points.push_back(Vector3f( 10.0f, 0.0f, 10.0f));
		points.push_back(Vector3f( -10.0f, 0.0f, 10.0f));
		points.push_back(Vector3f( -10.0f, 20.0f, 10.0f));
		ent = Entity(points, "bc4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 10.0f, 20.0f, 30.0f));
		points.push_back(Vector3f( 10.0f, 0.0f, 30.0f));
		points.push_back(Vector3f( 10.0f, 0.0f, 10.0f));
		points.push_back(Vector3f( 10.0f, 20.0f, 10.0f));
		ent = Entity(points, "bc5", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -70.0f, 10.0f, 20.0f));
		points.push_back(Vector3f( -70.0f, 10.0f, 60.0f));
		points.push_back(Vector3f( -60.0f, 10.0f, 60.0f));
		points.push_back(Vector3f( -60.0f, 10.0f, 20.0f));
		ent = Entity(points, "l1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -70.0f, 10.0f, 60.0f));
		points.push_back(Vector3f( -70.0f, 0.0f, 60.0f));
		points.push_back(Vector3f( -60.0f, 0.0f, 60.0f));
		points.push_back(Vector3f( -60.0f, 10.0f, 60.0f));
		ent = Entity(points, "l2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -70.0f, 10.0f, 20.0f));
		points.push_back(Vector3f( -70.0f, 0.0f, 20.0f));
		points.push_back(Vector3f( -70.0f, 0.0f, 60.0f));
		points.push_back(Vector3f( -70.0f, 10.0f, 60.0f));
		ent = Entity(points, "l3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -60.0f, 10.0f, 20.0f));
		points.push_back(Vector3f( -60.0f, 0.0f, 20.0f));
		points.push_back(Vector3f( -70.0f, 0.0f, 20.0f));
		points.push_back(Vector3f( -70.0f, 10.0f, 20.0f));
		ent = Entity(points, "l4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -60.0f, 10.0f, 60.0f));
		points.push_back(Vector3f( -60.0f, 0.0f, 60.0f));
		points.push_back(Vector3f( -60.0f, 0.0f, 20.0f));
		points.push_back(Vector3f( -60.0f, 10.0f, 20.0f));
		ent = Entity(points, "l5", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -50.0f, 10.0f, 40.0f));
		points.push_back(Vector3f( -50.0f, 10.0f, 50.0f));
		points.push_back(Vector3f( -40.0f, 10.0f, 50.0f));
		points.push_back(Vector3f( -40.0f, 10.0f, 40.0f));
		ent = Entity(points, "c1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -50.0f, 10.0f, 50.0f));
		points.push_back(Vector3f( -50.0f, 0.0f, 50.0f));
		points.push_back(Vector3f( -40.0f, 0.0f, 50.0f));
		points.push_back(Vector3f( -40.0f, 10.0f, 50.0f));
		ent = Entity(points, "c2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -50.0f, 10.0f, 40.0f));
		points.push_back(Vector3f( -50.0f, 0.0f, 40.0f));
		points.push_back(Vector3f( -50.0f, 0.0f, 50.0f));
		points.push_back(Vector3f( -50.0f, 10.0f, 50.0f));
		ent = Entity(points, "c3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -40.0f, 10.0f, 40.0f));
		points.push_back(Vector3f( -40.0f, 0.0f, 40.0f));
		points.push_back(Vector3f( -50.0f, 0.0f, 40.0f));
		points.push_back(Vector3f( -50.0f, 10.0f, 40.0f));
		ent = Entity(points, "c4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( -40.0f, 10.0f, 50.0f));
		points.push_back(Vector3f( -40.0f, 0.0f, 50.0f));
		points.push_back(Vector3f( -40.0f, 0.0f, 40.0f));
		points.push_back(Vector3f( -40.0f, 10.0f, 40.0f));
		ent = Entity(points, "c5", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 74.0f, 30.0f, -80.0f));
		points.push_back(Vector3f( 74.0f, 30.0f, -74.0f));
		points.push_back(Vector3f( 80.0f, 30.0f, -74.0f));
		points.push_back(Vector3f( 80.0f, 30.0f, -80.0f));
		ent = Entity(points, "o1", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 74.0f, 30.0f, -74.0f));
		points.push_back(Vector3f( 74.0f, 0.0f, -74.0f));
		points.push_back(Vector3f( 80.0f, 0.0f, -74.0f));
		points.push_back(Vector3f( 80.0f, 30.0f, -74.0f));
		ent = Entity(points, "o2", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 74.0f, 30.0f, -80.0f));
		points.push_back(Vector3f( 74.0f, 0.0f, -80.0f));
		points.push_back(Vector3f( 74.0f, 0.0f, -74.0f));
		points.push_back(Vector3f( 74.0f, 30.0f, -74.0f));
		ent = Entity(points, "o3", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 80.0f, 30.0f, -80.0f));
		points.push_back(Vector3f( 80.0f, 0.0f, -80.0f));
		points.push_back(Vector3f( 74.0f, 0.0f, -80.0f));
		points.push_back(Vector3f( 74.0f, 30.0f, -80.0f));
		ent = Entity(points, "o4", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		points.clear();
		points.push_back(Vector3f( 80.0f, 30.0f, -74.0f));
		points.push_back(Vector3f( 80.0f, 0.0f, -74.0f));
		points.push_back(Vector3f( 80.0f, 0.0f, -80.0f));
		points.push_back(Vector3f( 80.0f, 30.0f, -80.0f));
		ent = Entity(points, "o5", -1.0f);
		ent.colour = Vector3f(rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f), rand_FloatRange(0.0f, 1.0f));
		myWorld->addEntity(ent);

		/*
		for(int i = -50; i < 50; i += 10){
                for(int j = -50; j < 50; j += 10){
                        std::string n = "ball";
                        std::ostringstream osstream;
                        osstream << i;
                        osstream << j;
                        std::string string_x = osstream.str();

                        n += string_x;

                        ent = Entity(Vector3f((float)i, 60.0f, (float)j), 1.0f, n, 1.0f);
		ent.colour = Vector3f(1.0f, 0.0f, 0.0f);
		myWorld->addEntity(ent);



                }

        }




	*/
		ent = Entity(Vector3f(9.0f, 31.0f, -67.0f), 1.0f, "ball1", 1.0f);
		ent.colour = Vector3f(1.0f, 0.0f, 0.0f);
		myWorld->addEntity(ent);

			ent = Entity(Vector3f(10.0f, 30.0f, -50.0f), 1.0f, "ball2", 1.0f);
		ent.colour = Vector3f(1.0f, 0.0f, 0.0f);
		myWorld->addEntity(ent);
	/*
		for(int i = -50; i < 50; i += 25){
                for(int j = -50; j < 50; j += 25){
                        std::string n = "ball";
                        std::ostringstream osstream;
                        osstream << i;
                        osstream << j;
                        std::string string_x = osstream.str();

                        n += string_x;

                        ent = Entity(Vector3f((float)i, rand_FloatRange(40.0f, 60.0f), (float)j), 1.0f, n, 1.0f);
		ent.colour = Vector3f(1.0f, 0.0f, 0.0f);
		myWorld->addEntity(ent);
    }
}

		//*/




//std::cout<<"init time: " << myTimer.getTime()<<std::endl;
myTimer = wtimer();
}





void renderScene(void) {

    // wtimer render = wtimer();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    Vector3f temp;
    Entity obj;
    std::vector<Entity> ents = myWorld->getObjects();

    for(int i = 0; i < ents.size(); i++){
          obj = ents.at(i);

          if(obj.verts.size() > 0){
                             // std::cout<<obj.verts.size()<<std::endl;
                       glPushMatrix();
                       temp = obj.colour;
                       glColor3f(temp.X, temp.Y, temp.Z);
                       if(obj.verts.size() == 3) glBegin(GL_TRIANGLES);
                       else glBegin(GL_QUADS);
                       for(int i = 0; i < obj.verts.size(); i++)
                       {
                       glVertex3f(obj.verts[i].X, obj.verts[i].Y, obj.verts[i].Z);
                       }
                       glEnd();
                       glPopMatrix();
    }
    else
    {
        //std::cout<<obj.getBSphere().radius<<std::endl;
        glPushMatrix();
        temp = obj.getPosition();
        glTranslated(temp.X,temp.Y, temp.Z);
        temp = obj.colour;
        glColor3f(temp.X, temp.Y, temp.Z);
        glutSolidSphere(obj.getBSphere().radius, 16, 16);
        glPopMatrix();

    }

    }
   // float ft = render.getTime();

   // std::cout<<"Render routine time: " <<ft <<std::endl;

    float t = myTimer.getTime();
    myWorld->update(t);

   // std::cout<<"update "<<t<<std::endl<< "total: " << ft+t<<std::endl;
	glutSwapBuffers();
}

void orientMe(float ang) {


	lx = sin(ang);
	lz = -cos(ang);
	glLoadIdentity();
	gluLookAt(x, y, z,
		      x + lx,y + ly,z + lz,
			  0.0f,1.0f,0.0f);
}


void moveMeFlat(int i) {
	x = x + i*(lx)*0.1;
	z = z + i*(lz)*0.1;
	glLoadIdentity();
	gluLookAt(x, y, z,
		      x + lx,y + ly,z + lz,
			  0.0f,1.0f,0.0f);
}
void shootBall() {
     std::string n = "ball";
                        std::ostringstream osstream;
                        osstream << rand_FloatRange(0.0f, 10.0f);
                        osstream << rand_FloatRange(0.0f, 10.0f);
                        std::string string_x = osstream.str();

                        n += string_x;
                        std::cout<< "shoot " << n <<std::endl;
     Entity ent = Entity(Vector3f(x,y,z), 1.0f, n, 1.0f);
     Vector3f temp = Vector3f(x,y,z), shoot = Vector3f(x + 20*lx, y + 20*ly, z + 20*lz);
     temp = shoot -  temp;
     temp.Y = temp.Y +2;

     temp.print();
     ent.setVelocity(temp);
		ent.colour = Vector3f(1.0f, 0.0f, 0.0f);
		myWorld->addEntity(ent);


}
void processNormalKeys(unsigned char key, int x, int y) {

	if (key == 27)
		exit(0);

		switch (key) {
		    case (119) : moveMeFlat(1);break;
		    case (115): moveMeFlat(-1);break;
		    case (97) : angle -= 0.01f;orientMe(angle);break;
		    case (100) : angle +=0.01f;orientMe(angle);break;
		    case (73) : initScene();break;
		    case (105) : initScene();break;
		    case (32) : shootBall();break;
		}
}


void inputKey(int key, int x, int y) {

	switch (key) {
		case GLUT_KEY_LEFT : angle -= 0.1f;orientMe(angle);break;
		case GLUT_KEY_RIGHT : angle +=0.1f;orientMe(angle);break;
		case GLUT_KEY_UP : moveMeFlat(10);break;
		case GLUT_KEY_DOWN : moveMeFlat(-10);break;
	}
}

static void
idle(void)
{

          myWorld->update(myTimer.getTime());
    glutPostRedisplay();
}


int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(640,360);
	glutCreateWindow("Physics Demo");

	initScene();

	glutKeyboardFunc(processNormalKeys);
	glutSpecialFunc(inputKey);

	glutDisplayFunc(renderScene);
	glutIdleFunc(idle);

	glutReshapeFunc(changeSize);

	glutMainLoop();

	return(0);
}
