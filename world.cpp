#include "world.h"



World::World(){
    gravity = -9.8f;
    cF = 0.9f;


};

World::~World(){

};


World::World(float grav, float tStep, float coeff) : gravity(grav), timeStep(tStep), cF(coeff)
{
                  //std::vector<Entity> worldObjects(10);
                   //std::vector<collisionPair> collisions(10);
};

bool World::CheckCollide(bSphere sphA, bSphere sphB){
     float d_centres = ((sphA.centre.x - sphB.centre.x)*(sphA.centre.x - sphB.centre.x)) + ((sphA.centre.y - sphB.centre.y)*(sphA.centre.y - sphB.centre.y)) + ((sphA.centre.z - sphB.centre.z)*(sphA.centre.z - sphB.centre.z));
     float radii = (sphA.radius + sphB.radius);
     radii *= radii;
     if(radii >= d_centres) return true;
     else
     return false;


};

void World::collideResponse(collisionPair pair){
     worldObjects.at(getIndex(pair.ent1.getName())).setCog(worldObjects.at(getIndex(pair.ent1.getName())).getCentreGravity() + worldObjects.at(getIndex(pair.ent1.getName())).moveVec*(pair.time/timeStep));
     worldObjects.at(getIndex(pair.ent2.getName())).setCog(worldObjects.at(getIndex(pair.ent2.getName())).getCentreGravity() + worldObjects.at(getIndex(pair.ent2.getName())).moveVec*(pair.time/timeStep));

     if(pair.ent1.getMass() > 0 && pair.ent2.getMass() > 0){
    Vector3f cV, u1x, u2x, u1y, u2y, v1x, v2x, v1y, v2y;
    float a, b, M1, M2;
    M1 = pair.ent1.getMass();
    M2 = pair.ent2.getMass();

    cV = pair.ent1.getCentreGravity() - pair.ent2.getCentreGravity();
    cV = cV.unit();
    a =  cV.dot(pair.ent2.getVelocity());
    u2x = cV * a;
    u2y = pair.ent2.getVelocity() - u2x;
    cV = pair.ent2.getCentreGravity() - pair.ent1.getCentreGravity();
    cV = cV.unit();
    b = cV.dot(pair.ent1.getVelocity());
    u1x = cV * b;
    u1y = pair.ent1.getVelocity() - u1x;

    v1x = ((u1x*M1) + (u2x*M2) - ((u1x - u2x)*M2))/(M1+M2);
    v2x = ((u1x*M1) + (u2x*M2) - ((u2x - u1x)*M1))/(M1+M2);

    v1y = u1y;
    v2y = u2y;

    int ind1  =  getIndex(pair.ent1.getName());
    int ind2 = getIndex(pair.ent2.getName());

    worldObjects.at(ind1).setVelocity((v1x + v1y)*cF);
    worldObjects.at(ind2).setVelocity((v2x + v2y)*cF);
    worldObjects.at(ind1).sleeping = false;
    worldObjects.at(ind2).sleeping = false;





    }

else
{
    Entity still, moving;
        float A, B, C, D;
        Vector3f cross;
        if( pair.ent1.getMass() < 0 ){
            still = pair.ent1;
            moving = pair.ent2;
            }
            else
            {
            still = pair.ent2;
            moving = pair.ent1;
            }
            //std::cout<<"daa"<<std::endl;
           if(still.verts.size() > 2){
            cross = Vector3f((still.verts[1] - still.verts[0]) / (still.verts[2] - still.verts[0]));
            cross = cross.unit();
            }else cross = moving.getCentreGravity() - still.getCentreGravity();
            
          if(moving.contacts.size() == 0){
                still.normal = cross;
                float reflect = moving.getVelocity().mod();
                Vector3f unitVel = moving.getVelocity().unit();


                Vector3f unitVelNeg = Vector3f(-unitVel.X, -unitVel.Y, -unitVel.Z);
                unitVel= Vector3f((still.normal*(2*still.normal.dot(unitVelNeg))) + unitVel ).unit();
                int ind = getIndex(moving.getName());
               // std::cout<<"rebound velocity ";
               // (unitVel*reflect).print();
                worldObjects.at(ind).setVelocity(unitVel*reflect*cF);
                }else{
                      }



}
partialStep(pair);


if(!pair.ent1.sleeping){
                        int ind;
                        ind = getIndex(pair.ent1.getName());
float vecMag = worldObjects.at(ind).moveVec.mod2();

                   if(vecMag < 0.0001f && vecMag > -0.0001f)
                   {
                                       worldObjects.at(ind).moveVec.zero();
                                       worldObjects.at(ind).sleeping = true;
                                       worldObjects.at(ind).setVelocity(Vector3f(0.0f, 0.0f, 0.0f));
                   }
         }

if(!pair.ent2.sleeping){
                        int ind;
                        ind = getIndex(pair.ent2.getName());
float vecMag = worldObjects.at(ind).moveVec.mod2();

                   if(vecMag < 0.0001f && vecMag > -0.0001f)
                   {
                                       worldObjects.at(ind).moveVec.zero();
                                       worldObjects.at(ind).sleeping = true;
                                       worldObjects.at(ind).setVelocity(Vector3f(0.0f, 0.0f, 0.0f));
                   }
         }

//takeStep();


};
void World::closestPoint(Vector3f a, Vector3f b, Vector3f testPoint, Vector3f &closest){
     
     
};

void World::partialStep(collisionPair pair){

     Vector3f acc, cog, vel, tns, newPos, move;
     float time = timeStep - pair.time;

         //ent1
         acc = worldObjects.at(getIndex(pair.ent1.getName())).getAcceleration();
         cog = worldObjects.at(getIndex(pair.ent1.getName())).getCentreGravity();
         vel = worldObjects.at(getIndex(pair.ent1.getName())).getVelocity();

         newPos.setX(cog.getX() + (vel.getX()*time) + ((time*time)*acc.getX()*0.5));
         newPos.setY(cog.getY() + (vel.getY()*time) + ((time*time)*(acc.getY()+gravity)*0.5));
         newPos.setZ(cog.getZ() + (vel.getZ()*time) + ((time*time)*acc.getZ()*0.5));
         worldObjects.at(getIndex(pair.ent1.getName())).moveVec = newPos - cog;


         tns = worldObjects.at(getIndex(pair.ent1.getName())).getTranslate();
         tns = tns + cog;
         worldObjects.at(getIndex(pair.ent1.getName())).setPosition(tns);


         //ent2
           acc = worldObjects.at(getIndex(pair.ent2.getName())).getAcceleration();
         cog = worldObjects.at(getIndex(pair.ent2.getName())).getCentreGravity();
         vel = worldObjects.at(getIndex(pair.ent2.getName())).getVelocity();

         newPos.setX(cog.getX() + (vel.getX()*time) + ((time*time)*acc.getX()*0.5));
         newPos.setY(cog.getY() + (vel.getY()*time) + ((time*time)*(acc.getY()+gravity)*0.5));
         newPos.setZ(cog.getZ() + (vel.getZ()*time) + ((time*time)*acc.getZ()*0.5));
         worldObjects.at(getIndex(pair.ent2.getName())).moveVec = newPos - cog;


         tns = worldObjects.at(getIndex(pair.ent2.getName())).getTranslate();
         tns = tns + cog;
         worldObjects.at(getIndex(pair.ent2.getName())).setPosition(tns);
};


void World::takeStep(){
     for(int i = 0; i < worldObjects.size(); i++){
             if(!worldObjects.at(i).sleeping)
             {


                   worldObjects.at(i).setCog(worldObjects.at(i).getCentreGravity() + worldObjects.at(i).moveVec);
             }
}
resolvePosition();
};

std::vector<collisionPair> World::broadPhase(float t){

                           bSphere a, b;
                           std::vector<collisionPair> broadHits;
                           Vector3f aCentre, bCentre;


                    for(int i = 0; i < worldObjects.size()-1; i++){

                            for(int j = i+1; j < worldObjects.size(); j++){

                                    if(worldObjects.at(i).getMass() < 0 && worldObjects.at(j).getMass() < 0) {}
                                    else {


                                    aCentre = worldObjects.at(i).getCentreGravity() + worldObjects.at(i).moveVec*t;
                                    bCentre = worldObjects.at(j).getCentreGravity() + worldObjects.at(j).moveVec*t;
                                    a.centre = aCentre.vToP();
                                    a.radius = worldObjects.at(i).getBSphere().radius;

                                    b.centre = bCentre.vToP();
                                    b.radius = worldObjects.at(j).getBSphere().radius;


                                    if(CheckCollide(a, b)){

                                    collisionPair pair;
                                    pair.ent1 = worldObjects.at(i);
                                    pair.ent2 = worldObjects.at(j);

                                    broadHits.push_back(pair);



                                    }
                                    }
                            }

                    }
                    return broadHits;

};
void World::bubbleSort(std::vector<collisionPair> &unsorted){
                           int i, j, flag = 1;    // set flag to 1 to begin initial pass
                           float temp;             // holding variable
                           int length = unsorted.size();
                           for(i = 1; (i <= length) && flag; i++)
                           {
                           flag = 0;
                           for (j=0; j < (length -1); j++)
                           {
                           if (unsorted.at(j+1).time < unsorted.at(j).time)      // ascending order simply changes to <
                           {
                             temp = unsorted.at(j).time;             // swap elements
                             unsorted.at(j).time = unsorted.at(j+1).time;
                             unsorted.at(j+1).time = temp;
                             flag = 1;               // indicates that a swap occurred.
               }
          }
     }
     return;   //arrays are pas

};

void World::update(float t){
     //wtimer updateTimer = wtimer();

     timeStep = t;
     float startTime = 0.0f, updateTime = t, hitTime = 100.0f, finTime = t;
     int hits  = 0;
     //std::cout<<worldObjects.size()<<std::endl;
     resolveAcceleration(updateTime);
     resolveCog(updateTime);
     updateTime /=20.0f;
     std::vector<collisionPair> BroadPhaseHits = broadPhase(t);
    // std::cout<<"update time "<< updateTime <<std::endl;

             for(int i = 0; i < BroadPhaseHits.size(); i++){
                    // resolveAcceleration(updateTime);
                     startTime = 0.0f;
                     hitTime = 100.0f;
                     collisionPair cPair =BroadPhaseHits.at(i);

                     Entity x = cPair.ent1;
                     Entity y = cPair.ent2;


                    //td::cout<<x.getName()<< " " << y.getName()<<std::endl;
                     while(startTime < finTime){

                                     //std::cout<<"time step "<<startTime << " finTime " << finTime<<std::endl;
                   if(  CheckCollide(x, y, startTime) ){
                        //std::cout<<" start "<<startTime<<std::endl;

                        hitTime = startTime - updateTime;
                        //std::cout<<"hit time "<< hitTime <<std::endl;
                        if(hitTime < 0.0f) hitTime = startTime;
                        collisionPair pair;
                        pair.ent1 = x;
                        pair.ent2 = y;
                        pair.time = hitTime;
                        collisions.push_back(pair);
                        hits++;

                      //  x.getVelocity().print();
                      //  y.getVelocity().print();
                        // std::cout << "hits " << hitTime << " start time " <<startTime << std::endl;
                       // break;
                       startTime += finTime;
                      // std::cout<<"start "<<startTime<<" hit "<< hitTime<<" update "<<updateTime<<std::endl;
                        }

                        startTime += updateTime;
                   }
             }


     if(hits > 0){
             if(collisions.size() > 1) bubbleSort(collisions);


            for(int i = 0; i < collisions.size(); i++){

                    collideResponse(collisions.at(i));



            }
            takeStep();
            collisions.clear();

            }else{
              takeStep();

             }
             resolvePosition();




};

bool World::inPoly(std::vector<Vector3f> points, Vector3f testPoint){
    // return true;
     int numVertices = points.size();
     Vector3f p = (points.at(numVertices - 1) - testPoint)/(points.at(0) - testPoint);


     for(int i = 0; i < numVertices - 1; i++){

             Vector3f q = (points.at(i) - testPoint)/(points.at(i+1) - testPoint);
             if(p%q < 0){ return false; }
             }

             return true;



};
bool World::boxSphere(Entity a, Entity b){
     Entity sphere, box;
     if(a.verts.size() > 4){ sphere = b; box = a;}
     else{ sphere = a; box = b;}
     Vector3f centre = sphere.getCentreGravity();
     Vector3f relCentre = box.transformMatrix.transformInverse(centre);
     bSphere sph = sphere.getBSphere();
     if( fabsf(relCentre.X) - sph.radius > box.halfSizes.X ||
         fabsf(relCentre.Y) - sph.radius > box.halfSizes.Y ||
         fabsf(relCentre.Z) - sph.radius > box.halfSizes.Z) { return false; }

     else{

           Vector3f closestPt(0,0,0);
           float dist;


           dist = relCentre.X;
           if (dist > box.halfSizes.X) dist = box.halfSizes.X;
           if (dist < -box.halfSizes.X) dist = -box.halfSizes.X;
           closestPt.X = dist;

           dist = relCentre.Y;
           if (dist > box.halfSizes.Y) dist = box.halfSizes.Y;
           if (dist < -box.halfSizes.Y) dist = -box.halfSizes.Y;
           closestPt.Y = dist;

           dist = relCentre.Z;
           if (dist > box.halfSizes.Z) dist = box.halfSizes.Z;
           if (dist < -box.halfSizes.Z) dist = -box.halfSizes.Z;
           closestPt.Z = dist;

           dist = (closestPt - relCentre).mod2();
           if (dist > sph.radius * sph.radius) return false;

           Vector3f closestPtWorld = box.transformMatrix.transform(closestPt);

           worldObjects.at(getIndex(box.getName())).contacts.push_back(closestPtWorld);
           return true;

     }

};

bool World::boxBox(Entity a, Entity b){
     std::cout<<"Box-Box"<<std::endl;
     return false;
};

bool World::CheckCollide(Entity a, Entity b, float time){
     time = (time/timeStep);
     a.newPos = a.getCentreGravity() + a.moveVec*time;
     b.newPos = b.getCentreGravity() + b.moveVec*time;

     if((!a.sleeping) && (!b.sleeping)){
     
     bSphere sphA = a.getBSphere();
     bSphere sphB = b.getBSphere();
     Vector3f aNewPos, bNewPos;
     aNewPos = a.newPos;
     bNewPos = b.newPos;
     float dist = (aNewPos - bNewPos).mod2();
     float radii = (sphA.radius + sphB.radius);
     radii *= radii;


     //dist -= radii;
     if(dist <= radii){
             if(a.verts.size() > 4 && b.verts.size() > 4) return (boxBox(a, b));
             else if (a.verts.size() > 4 || b.verts.size() > 4) return (boxSphere(a, b));
          

    return true;
     }
     else{

           return false;
           }
    }
    else if( a.sleeping && b.sleeping ){

         return false;
    }
    else
    {

        Entity still, moving;
        float A, B, C, D;
        Vector3f cross;
        if( a.sleeping ){
            still = a;
            moving = b;
            }
            else
            {
            still = b;
            moving = a;
            }

           if(still.verts.size() < 3) {/*hemisphere collision */ return true;}
            cross = Vector3f((still.verts[1] - still.verts[0]) / (still.verts[2] - still.verts[0]));
            cross = cross.unit();
            A = cross.X;
            B = cross.Y;
            C = cross.Z;
            D = -(A * still.verts[0].X + B * still.verts[0].Y + C * still.verts[0].Z);

            float distance = (A*moving.newPos.X + B*moving.newPos.Y + C*moving.newPos.Z + D)/Vector3f(A, B, C).mod();
           // distance = distance;
            //std::cout<<"distance "<<distance<<std::endl;
            //std::cout<<a.getName()<< " " << b.getName()<<std::endl;
            if (distance > moving.getBSphere().radius){
                         //std::cout<<"dii"<<std::endl;
            return false;
            }
            else
            {
               // std::cout<<moving.verts.size() << moving.getName()<<std::endl;
                if(moving.verts.size() < 4){

                Vector3f point, P, Q, N;
                N = Vector3f(A, B, C).unit();
//                N = N.unit();
                P = still.verts[0];
                Q = moving.newPos;
                //point = Q - ((Q-P).dot(N)) * N;
                point = Q - P;
                float dotNP = point.dot(N);
                point = N*dotNP;
                point = Q - point;
               // still.verts.at(0).print();

                if(inPoly(still.verts, point)){
                //std::cout<<"still "<<still.getName() <<" moving "<< moving.getName()<<std::endl;
              return true;

                }
                else return false;
                }
                else
                {

                    Vector3f p;
                    moving.contacts.clear();
                    int count = 0;
                    //std::cout<<moving.verts.size()<<std::endl;
                    for(int i = 0; i < moving.verts.size(); i++){
                            p = moving.verts.at(i) + moving.getPosition();
                            distance = (A*p.X + B*p.Y + C*p.Z + D)/Vector3f(A, B, C).mod();
                            //std::cout<<"hit "<<distance<<std::endl;
                            if(distance < 0.0f ){

                                         Vector3f point, P, Q, N;

                                         N = Vector3f(A, B, C).unit();
                                         P = still.verts[0];
                                         Q = p;
                                         point = Q - P;
                                         float dotNP = point.dot(N);
                                         point = N*dotNP;
                                         point = Q - point;


                                         if(inPoly(still.verts, point)){

                                                                worldObjects.at(getIndex(moving.getName())).contacts.push_back(moving.verts.at(i));
                                                                count ++;

                                        }

                            }


                }

                 if(count > 0){ return true; }

                 else return false;


                }

                //else return false;
            }


    }


};

void World::resolveForces(Entity param){
     Vector3f acc = param.getForces() * param.getInverseMass();


};

void World::resolveAcceleration(float time){
     if(noObjects()) return;
     if(time == 0) return;
     // std::vector<Entity>::iterator entIt;
     Vector3f acc, pos, vel;
     for(int i=0; i < worldObjects.size(); i++){
             if(!worldObjects.at(i).sleeping) {
            // resolveForces(worldObjects.at(i));

           // if(worldObjects.at(i).getName() == "cube") (worldObjects.at(i).getCentreGravity()).print();



        // acc = worldObjects.at(i).getAcceleration();
         acc.zero();
         worldObjects.at(i).setAcceleration(acc);
         pos = worldObjects.at(i).getPosition();
         vel = worldObjects.at(i).getVelocity();

         Vector3f angularAcceleration =
        worldObjects.at(i).inverseInertiaTensorWorld.transform(worldObjects.at(i).torques);

         worldObjects.at(i).rotation += angularAcceleration* time;

         worldObjects.at(i).orientation.addScaledVector(worldObjects.at(i).rotation, time);



          acc += worldObjects.at(i).getForces()*worldObjects.at(i).getInverseMass();
          acc += worldObjects.at(i).getImpulses()*worldObjects.at(i).getInverseMass();
         worldObjects.at(i).clearImpulses();
         worldObjects.at(i).setAcceleration(acc);

       // std::cout<<timeStep*acc.getX()<<" "<<(vel.getX() + (timeStep*acc.getX()))<<std::endl;
         vel.setX(vel.getX() + (time*acc.getX()));
         vel.setY(vel.getY() + (time*(acc.getY()+gravity)));
         vel.setZ(vel.getZ() + (time*acc.getZ()));

         worldObjects.at(i).setVelocity(vel);
         //std::cout<<entIt->getVelocity().getX()<<std::endl;
         worldObjects.at(i).calculateData();
         worldObjects.at(i).clearAccumulators();
         }

     }

};

void World::resolvePosition(){

     for(int i = 0; i < worldObjects.size(); i++){
             if(!worldObjects.at(i).sleeping)
             {
             Vector3f tns, cog;
             cog = worldObjects.at(i).getCentreGravity();
             tns = worldObjects.at(i).getTranslate();
             tns = tns + cog;
             worldObjects.at(i).setPosition(tns);
             }
             }


};

void World::resolveCog(float time) {
      if(noObjects()) return;
      if(time == 0) return;

     Vector3f acc, cog, vel, tns, newPos, move;
     for(int i=0; i < worldObjects.size(); i++){
              if(!worldObjects.at(i).sleeping)
              {
         acc = worldObjects.at(i).getAcceleration();
         cog = worldObjects.at(i).getCentreGravity();
         vel = worldObjects.at(i).getVelocity();

         newPos.setX(cog.getX() + (vel.getX()*time) + ((time*time)*acc.getX()*0.5));
         newPos.setY(cog.getY() + (vel.getY()*time) + ((time*time)*(acc.getY()+gravity)*0.5));
         newPos.setZ(cog.getZ() + (vel.getZ()*time) + ((time*time)*acc.getZ()*0.5));
         worldObjects.at(i).moveVec = newPos - cog;
         float vecMag = worldObjects.at(i).moveVec.mod2();

         tns = worldObjects.at(i).getTranslate();
         tns = tns + cog;
         worldObjects.at(i).setPosition(tns);
}
     }
};

Vector3f World::GetPositions(){

};

void World::addEntity(Entity ent){
     worldObjects.push_back(ent);
};

void World::removeEntity(Entity ent){
     if(!worldObjects.empty()) {
     std::vector<Entity>::iterator entIt;
     entIt = worldObjects.begin();
//     std::cout<<entIt->getName()<<std::endl;
     while(entIt->getName() != ent.getName()){ entIt++;}
     worldObjects.erase(entIt);
     }
//          worldObjects.pop_back();
};

Entity World::getEntity(std::string name){
       for(int i=0; i < worldObjects.size(); i++){
             if( worldObjects.at(i).getName() == name){
                 return worldObjects.at(i);
             }

     }

};

int World::getIndex(std::string name){
   // std::cout<<"get index"<<std::endl;
       for(int i=0; i < worldObjects.size(); i++){
             if( worldObjects.at(i).getName() == name){
     //            std::cout<<"i "<< i <<std::endl;
                 return i;
             }

     }

};

