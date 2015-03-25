#include "Entity.h"
//#include <iostream>


///>BodyDerivedTransformIT
/**
 * Internal function to do an intertia tensor transform by a quaternion.
 * Note that the implementation of this function was created by an
 * automated code-generator and optimizer.
 */
static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                           const Quaternion &q,
                                           const Matrix3 &iitBody,
                                           const Matrix4 &rotmat)
{
    float t4 = rotmat.data[0]*iitBody.data[0]+
        rotmat.data[1]*iitBody.data[3]+
        rotmat.data[2]*iitBody.data[6];
    float t9 = rotmat.data[0]*iitBody.data[1]+
        rotmat.data[1]*iitBody.data[4]+
        rotmat.data[2]*iitBody.data[7];
    float t14 = rotmat.data[0]*iitBody.data[2]+
        rotmat.data[1]*iitBody.data[5]+
        rotmat.data[2]*iitBody.data[8];
    float t28 = rotmat.data[4]*iitBody.data[0]+
        rotmat.data[5]*iitBody.data[3]+
        rotmat.data[6]*iitBody.data[6];
    float t33 = rotmat.data[4]*iitBody.data[1]+
        rotmat.data[5]*iitBody.data[4]+
        rotmat.data[6]*iitBody.data[7];
    float t38 = rotmat.data[4]*iitBody.data[2]+
        rotmat.data[5]*iitBody.data[5]+
        rotmat.data[6]*iitBody.data[8];
    float t52 = rotmat.data[8]*iitBody.data[0]+
        rotmat.data[9]*iitBody.data[3]+
        rotmat.data[10]*iitBody.data[6];
    float t57 = rotmat.data[8]*iitBody.data[1]+
        rotmat.data[9]*iitBody.data[4]+
        rotmat.data[10]*iitBody.data[7];
    float t62 = rotmat.data[8]*iitBody.data[2]+
        rotmat.data[9]*iitBody.data[5]+
        rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+
        t9*rotmat.data[1]+
        t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+
        t9*rotmat.data[5]+
        t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+
        t9*rotmat.data[9]+
        t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+
        t33*rotmat.data[1]+
        t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+
        t33*rotmat.data[5]+
        t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+
        t33*rotmat.data[9]+
        t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+
        t57*rotmat.data[1]+
        t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+
        t57*rotmat.data[5]+
        t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+
        t57*rotmat.data[9]+
        t62*rotmat.data[10];
}
///<BodyDerivedTransformIT

///>BodyDerivedTransform
/**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 */
static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                             const Vector3f &position,
                                             const Quaternion &orientation)
{
    transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
        2*orientation.k*orientation.k;
    transformMatrix.data[1] = 2*orientation.i*orientation.j -
        2*orientation.r*orientation.k;
    transformMatrix.data[2] = 2*orientation.i*orientation.k +
        2*orientation.r*orientation.j;
    transformMatrix.data[3] = position.X;

    transformMatrix.data[4] = 2*orientation.i*orientation.j +
        2*orientation.r*orientation.k;
    transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
        2*orientation.k*orientation.k;
    transformMatrix.data[6] = 2*orientation.j*orientation.k -
        2*orientation.r*orientation.i;
    transformMatrix.data[7] = position.Y;

    transformMatrix.data[8] = 2*orientation.i*orientation.k -
        2*orientation.r*orientation.j;
    transformMatrix.data[9] = 2*orientation.j*orientation.k +
        2*orientation.r*orientation.i;
    transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
        2*orientation.j*orientation.j;
    transformMatrix.data[11] = position.Z;
}
///<BodyDerivedTransform






Entity::Entity() {};
Entity::Entity( std::vector<Vector3f> vertices, std::string name, float M) : verts(vertices), entName(name), mass(M), oneOverM(1/M) {
                  createBSphere(verts);
                  oneOverM = 1/M;
                  Matrix3 tensor;
                  Position = Vector3f(sphere.centre.x, sphere.centre.y, sphere.centre.z);
                  velocity = Vector3f(0.0f, 0.0f, 0.0f);

                  halfSizes.X = (aabb.max.x - aabb.min.x)*0.5f;
                  halfSizes.Y = (aabb.max.y - aabb.min.y)*0.5f;
                  halfSizes.Z = (aabb.max.z - aabb.min.z)*0.5f;
                  tensor.setBlockInertiaTensor(halfSizes, M);
                  inverseInertiaTensor.setInverse(tensor);


                  if(mass > 0){
                          sleeping = false;
                          for(int i = 0; i < verts.size(); i++){
                                  verts.at(i) = verts.at(i) - Position;

                                  }


                          }
                          else sleeping = true;




                  renderToPhysics();

};


Entity::Entity(Vector3f pos, float radius, std::string name, float M) : Position(pos), CentreGravity(pos), entName(name), mass(M) {
                  createBSphere(pos, radius);
                  oneOverM = 1/M;
                  float val = 0.2f*M*radius*radius;
                  Matrix3 tensor;
                  tensor.setDiagonal(val, val, val);
                  inverseInertiaTensor.setInverse(tensor);
                  velocity = Vector3f(0.0f, 0.0f, 0.0f);
                  renderToPhysics();

                  if(mass > 0) sleeping = false;
                  else sleeping = true;

};

void Entity::calculateData()
{
   /*  renderToPhysics();
     if(!verts.empty() && mass > 0){
                        for(int i = 0; i < verts.size(); i++){
                                verts.at(i) = verts.at(i) + translate;
                                }


                        }
                        */
                      // <BodyDerivedBase
    orientation.normalise();

///>BodyDerivedTransform
    // Calculate the transform matrix for the body.
    _calculateTransformMatrix(transformMatrix, Position, orientation);
///<BodyDerivedTransform

///>BodyDerivedTransformIT
    // Calculate the inertiaTensor in world space.
    _transformInertiaTensor(inverseInertiaTensorWorld,
        orientation,
        inverseInertiaTensor,
        transformMatrix);
///<BodyDerivedTransformIT

};


void Entity::addPForce(Vector3f force, Vector3f point)
{
     Vector3f pt = point;
    pt = pt - Position;

    forces += force;
    torques += pt / force;
};

void Entity::createBSphere(Vector3f s_centre, float s_radius) {
     sphere.radius = s_radius;
     sphere.centre.x = s_centre.getX();
     sphere.centre.y = s_centre.getY();
     sphere.centre.z = s_centre.getZ();
};

void Entity::createBSphere(std::vector<Vector3f>vertices) {
     createAABB(vertices);
     sphere.centre.x = (aabb.min.x + aabb.max.x)/2;
     sphere.centre.y = (aabb.min.y + aabb.max.y)/2;
     sphere.centre.z = (aabb.min.z + aabb.max.z)/2;
     sphere.radius = sqrt(((aabb.max.x - sphere.centre.x)*(aabb.max.x - sphere.centre.x)) + ((aabb.max.y - sphere.centre.y)*(aabb.max.y - sphere.centre.y)) + ((aabb.max.z - sphere.centre.z)*(aabb.max.z - sphere.centre.z)));

};
void Entity::createAABB(std::vector<Vector3f>vertices) {
     Vector3f curr;

     float x, y, z, xt, yt, zt;
     int count = 0;
     if(!vertices.empty()){
                           curr = vertices.back();
                           x = aabb.min.x = aabb.max.x = curr.getX();
                           y = aabb.min.y = aabb.max.y = curr.getY();
                           z = aabb.min.z = aabb.max.z = curr.getZ();

                           count++;
                           vertices.pop_back();

     }

     while(!vertices.empty()){
                              curr = vertices.back();
                              xt = curr.getX();
                              yt = curr.getY();
                              zt = curr.getZ();
                              x += xt;
                              y += yt;
                              z += zt;
                              aabb.min.x = std::min(xt, aabb.min.x);
                              aabb.min.y = std::min(yt, aabb.min.y);
                              aabb.min.z = std::min(zt, aabb.min.z);
                              aabb.max.x = std::max(xt, aabb.max.x);
                              aabb.max.y = std::max(yt, aabb.max.y);
                              aabb.max.z = std::max(zt, aabb.max.z);
                              count ++;
                              vertices.pop_back();


     }
     x = x/count;
     y = y/count;
     z = z/count;
     CentreGravity = Vector3f(x,y,z);

};

void Entity::addForce(Vector3f newForce) {
     forces += newForce;
};

void Entity::addImpulse(Vector3f newImpulse) {
     impulses += newImpulse;
};



void Entity::setVelocity(Vector3f nVel){
     velocity = nVel;
};

void Entity::setPosition(Vector3f nPos){
     Position = nPos;
};

void Entity::setCog(Vector3f nCog){
     CentreGravity = nCog;
};

void Entity::renderToPhysics(){
     float x1, y1, z1, x2, y2, z2;
     Vector3f temp = getCentreGravity();
     x1 = temp.getX();
     y1 = temp.getY();
     z1 = temp.getZ();
     temp = getPosition();
     x2 = temp.getX();
     y2 = temp.getY();
     z2 = temp.getZ();
     //std::cout<< y1 << " " << y2<< " " << -(y1-y2)<<std::endl;
     translate.setX(-(x1 - x2));
     translate.setY(-(y1 - y2));
     translate.setZ(-(z1 - z2));

};
