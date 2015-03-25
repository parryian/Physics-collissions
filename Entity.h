#include "Vector3f.h"

//#include <String>
#include <vector>

class Entity {
      public:
             Entity();
             Entity(Vector3f position, float radius, std::string name, float M);
             Entity(std::vector<Vector3f>vertices, std::string name, float M);
             Vector3f getPosition() { return Position;};
             Vector3f getCentreGravity() { return CentreGravity;};
             Vector3f getAcceleration() { return acceleration;};
             Vector3f getVelocity() { return velocity;};
             void setVelocity(Vector3f);
             void setPosition(Vector3f);
             void setCog(Vector3f);
             void setAcceleration(Vector3f acc){ acceleration = acc; };
             Vector3f getForces() { return forces; };
             Vector3f getImpulses() { return impulses; };
             AABB GetAABB() { return aabb;};
             bSphere getBSphere() { return sphere; };
             std::string getName() { return entName;};
             void createAABB(std::vector<Vector3f>vertices);
             void createBSphere(Vector3f, float);
             void createBSphere(std::vector<Vector3f>vertices);
             void addForce(Vector3f);
             void addImpulse(Vector3f);
             bool sleeping;
             Entity* getEntity() {return this;};
             float getMass() { return mass; };
             float getInverseMass() { return oneOverM; };
             Vector3f getTranslate() { return translate; };
             Vector3f moveVec;
             Vector3f newPos;
             Vector3f colour;
             std::vector<Vector3f> verts;
             std::vector<Vector3f> contacts;
             Vector3f normal;
             void calculateData();
             Vector3f torques;
              float k;
              Vector3f halfSizes;              
              void addTorque(Vector3f t) { torques += t; }             
              void addPForce(Vector3f force, Vector3f pt);
              void clearTorques() { torques.zero(); }
              void clearForces() { forces.zero(); }
              void clearImpulses() { impulses.zero(); }
              void clearAccumulators() { clearTorques(); clearForces(); clearImpulses(); }
              
              Vector3f rotation;
              Quaternion orientation;
              Matrix4 transformMatrix;
              Matrix3 inverseInertiaTensor;
              Matrix3 inverseInertiaTensorWorld;
            
      private:
              float mass;
              float oneOverM;
              std::string entName;
              AABB aabb;
              bSphere sphere;
              Vector3f Position;
              Vector3f CentreGravity;
              Vector3f translate;
              
              Vector3f forces;
              Vector3f impulses;
              
              Vector3f acceleration;
              Vector3f velocity;
              
              void renderToPhysics(); 
};
             
             
