//#include "Entity.h"
#include "collisionPair.h"
#include "timer.h"
//#include <iostream>
//#include <vector>
//#include "Vector3f.h"

class World {
      public:
             World();
             ~World();
             World(float, float, float);
             Vector3f GetPositions();
             void resolveAcceleration(float);
             void resolvePosition();
             void resolveCog(float);
             void update(float);
             void collideResponse(collisionPair);
             int getIndex(std::string);
             bool CheckCollide(Entity, Entity, float);
             bool CheckCollide(bSphere, bSphere);
             bool boxSphere(Entity, Entity);
             bool boxBox(Entity, Entity);
             void addEntity(Entity);
             void removeEntity(Entity);
             bool noObjects() { return worldObjects.empty();};
             Entity getEntity(std::string);
             int count;
             void resolveForces(Entity);
             std::vector<Entity> getObjects(){ return worldObjects; }

      private:
              
              std::vector<Entity> worldObjects;
              std::vector<collisionPair> collisions;
              std::vector<collisionPair> broadPhase(float);
              float gravity;
              float timeStep;
              float cF;
              void takeStep();
              void partialStep(collisionPair);
              void bubbleSort(std::vector<collisionPair> &unsorted);
              void closestPoint(Vector3f, Vector3f, Vector3f, Vector3f&);

              bool inPoly(std::vector<Vector3f> points, Vector3f testPoint);




};
