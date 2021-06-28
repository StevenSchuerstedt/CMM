#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// add more for yours
enum MethodTypes {
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADER=4, ALL=5
    };
enum IntegrationMethodTypes {
    EXPLICIT = 0, SYMPLETIC = 1, MIDPOINT = 2
};

enum BoidGroups {
    RED = 1, BLUE = 2
};

template <class T, int dim>
class Boids
{
    typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Matrix<T, dim,Eigen::Dynamic> TVStack;
    typedef Vector<T, dim> TV;
    typedef Matrix<T, dim, dim> TM;
    
private:
    TVStack positions;
    TVStack velocities;
    TVStack midpoint_pos;
    TVStack midpoint_vel;
    TVStack obstacle_force;


    int n;
    bool update = false;
public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {
        initializePositions();
    }
    ~Boids() {}

    TVStack boidGroups;
    float h = 0.03;
    float m_radius = 0.6f;
    float m_CohesionStrength = 1.0f;
    float m_AlignmentStrength = 0.7f;
    float m_sepDis = 0.3f;
    float m_sepStr = 10.0f;
    float m_leaderStrength = 1.0f;
    float m_newBoidDistance = 0.25f;
    float m_despawnRadius = 0.05f;
    float m_controlStrength = 1.0f;

    int m_blueBoids = 0;
    int m_redBoids = 0;

    TV leaderPos;
    TV circlePos;
    float circleRadius = 0.75f;

    void setParticleNumber(int n) {this->n = n;}
    int getParticleNumber() { return n; }
    void initializePositions()
    {
        setParticleNumber(400);
        m_blueBoids = 0;
        m_redBoids = 0;
        positions = TVStack::Zero(dim, n).unaryExpr([&](T dummy){return static_cast <T> (rand() * 3) / static_cast <T> (RAND_MAX);}); 
        boidGroups = TVStack::Zero(dim, n);
        for (int i = 0; i < getParticleNumber(); i++) {
            TV pos = positions.col(i);
       
            pos += TV(1,1);
     
            positions.col(i) = pos;    
            if (i % 2 == 0) {
                boidGroups.col(i) = TV(RED, 0);
                m_redBoids++;
            }  
            else {
                boidGroups.col(i) = TV(BLUE, 0);
                m_blueBoids++;
            }
                
        }
        //velocities = TVStack::Zero(dim, n).unaryExpr([&](T dummy) {return static_cast <T> (rand()) / static_cast <T> (RAND_MAX); });
         velocities = TVStack::Zero(dim, n);

         midpoint_pos = TVStack::Zero(dim, n);
         midpoint_vel = TVStack::Zero(dim, n);
         obstacle_force = TVStack::Zero(dim, n);
         leaderPos = TV(1, 1);
    }

    void addBoid(TV position, BoidGroups group) {
        if (group == RED)
            m_redBoids++;
        if (group == BLUE)
            m_blueBoids++;
        int currentBoids = getParticleNumber();
        int newBoids = currentBoids + 1;
        setParticleNumber(newBoids);
        positions.conservativeResize(dim, n);
        velocities.conservativeResize(dim, n);
        obstacle_force.conservativeResize(dim, n);
        boidGroups.conservativeResize(dim, n);

        positions.col(newBoids - 1) = position;
        velocities.col(newBoids - 1) = TV(0,0);
        obstacle_force.col(newBoids - 1) = TV(0,0);
        boidGroups.col(newBoids - 1) = TV(group, 0);
    }

    void removeBoid(int index) {

        int currentBoids = getParticleNumber();
        int newBoids = currentBoids - 1;
        setParticleNumber(newBoids);
        if (boidGroups.col(index)[0] == RED)
            m_redBoids--;
        if (boidGroups.col(index)[0] == BLUE)
            m_blueBoids--;
        unsigned int colToRemove = index;
        unsigned int numRows = positions.rows();
        unsigned int numCols = positions.cols() - 1;

        if (colToRemove < numCols) {
            positions.block(0, colToRemove, numRows, numCols - colToRemove) = positions.block(0, colToRemove + 1, numRows, numCols - colToRemove);
            velocities.block(0, colToRemove, numRows, numCols - colToRemove) = velocities.block(0, colToRemove + 1, numRows, numCols - colToRemove);
            boidGroups.block(0, colToRemove, numRows, numCols - colToRemove) = boidGroups.block(0, colToRemove + 1, numRows, numCols - colToRemove);
            obstacle_force.block(0, colToRemove, numRows, numCols - colToRemove) = obstacle_force.block(0, colToRemove + 1, numRows, numCols - colToRemove);
        }
        velocities.conservativeResize(numRows, numCols);
        boidGroups.conservativeResize(numRows, numCols);
        positions.conservativeResize(numRows, numCols);
        obstacle_force.conservativeResize(numRows, numCols);
    }

    TVStack applyFreefall(bool useMidpoint = false) {
        TVStack force = TVStack::Zero(dim, n);
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            f = TV(0.0f, 9.81f);
            force.col(i) = f;
        }
        return force;
    }
    TVStack applyCohesion() {

       TVStack force = TVStack::Zero(dim, n);
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);

            int boids_in_range = 0;
            TV average_pos = TV(0.0f, 0.0f);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_radius) {
                    boids_in_range++;
                    average_pos += posj;
                }
            }
            if (boids_in_range > 0)
                average_pos /= boids_in_range;
            else
                average_pos = posi;

            TV cohesion_vector = posi - average_pos;

            f = cohesion_vector * -m_CohesionStrength;
            force.col(i) = f;
        }
        return force;

    }
    TVStack applySeparation() {

        TVStack force = TVStack::Zero(dim, n);
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            TV sep_vector = TV(0.0, 0.0);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_sepDis) {
                    sep_vector = posi - posj;
                    break;
                }
            }

            f = sep_vector * m_sepStr;
            force.col(i) = f;
        }
        return force;

    }
    TVStack applyAlignment() {

        TVStack force = TVStack::Zero(dim, n);
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            TV veli = velocities.col(i);

            int boids_in_range = 0;
            TV average_vel = TV(0.0f, 0.0f);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_radius) {
                    boids_in_range++;
                    average_vel += velocities.col(j);
                }
            }
            if (boids_in_range > 0)
                average_vel /= boids_in_range;
            else
                average_vel = veli;

            TV alignment_vector = veli - average_vel;

            f = alignment_vector * -m_AlignmentStrength;
            force.col(i) = f;
        }
        return force;

    }
    TVStack applyAll(bool useMidpoint = false) {

        TVStack force = TVStack::Zero(dim, n);
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            if (useMidpoint)
                posi = midpoint_pos.col(i);

            int boids_in_range = 0;
            TV average_pos = TV(0.0f, 0.0f);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                if (useMidpoint)
                    posj = midpoint_pos.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_radius) {
                    boids_in_range++;
                    average_pos += posj;
                }
            }
            if (boids_in_range > 0)
                average_pos /= boids_in_range;
            else
                average_pos = posi;

            TV cohesion_vector = posi - average_pos;

            f = cohesion_vector * -m_CohesionStrength;
            force.col(i) += f;
        }
        //alignment 
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            if (useMidpoint)
                posi = midpoint_pos.col(i);
            TV veli = velocities.col(i);
            if (useMidpoint)
                veli = midpoint_vel.col(i);

            int boids_in_range = 0;
            TV average_vel = TV(0.0f, 0.0f);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                if (useMidpoint)
                    posj = midpoint_pos.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_radius) {
                    boids_in_range++;
                    average_vel += velocities.col(j);
                }
            }
            if (boids_in_range > 0)
                average_vel /= boids_in_range;
            else
                average_vel = veli;

            TV alignment_vector = veli - average_vel;

            f = alignment_vector * -m_AlignmentStrength;
            force.col(i) += f;
        }
        //separation
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            if (useMidpoint)
                posi = midpoint_pos.col(i);
            TV sep_vector = TV(0.0, 0.0);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                if (useMidpoint)
                    posj = midpoint_pos.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_sepDis) {
                    sep_vector = posi - posj;
                    break;
                }
            }

            f = sep_vector * m_sepStr;
            force.col(i) += f;
        }


        //control force
        for (int i = 0; i < getParticleNumber(); i++) {
           // if (boidGroups.col(i)[0] == BLUE)
              //  continue;
            TV f = force.col(i);
            TV posi = positions.col(i);
            float maxDistance = 10000.0f;
            int closestBoid = 0;

            for (int j = 0; j < getParticleNumber(); j++) {
                //find closest boid in the same group
                if (boidGroups.col(j)[0] != boidGroups.col(i)[0] || i == j)
                    continue;
                TV posj = positions.col(j);
                float distance = (posi - posj).norm();
                if (distance < maxDistance) {
                    maxDistance = distance;
                    closestBoid = j;
                } 
                    
            }
            TV dir = (posi - positions.col(closestBoid)).normalized();

            f = dir * -m_controlStrength;
            force.col(i) += f;
        }

        return force;

    }

    TVStack applyLeader(bool useMidpoint = false) {


        TVStack force = TVStack::Zero(dim, n);
        for (int i = 1; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            if (useMidpoint)
                posi = midpoint_pos.col(i);

            int boids_in_range = 0;
            TV average_pos = TV(0.0f, 0.0f);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                if (useMidpoint)
                    posj = midpoint_pos.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_radius) {
                    boids_in_range++;
                    average_pos += posj;
                }
            }
            if (boids_in_range > 0)
                average_pos /= boids_in_range;
            else
                average_pos = posi;

            TV cohesion_vector = posi - average_pos;

            f = cohesion_vector * -m_CohesionStrength;
            force.col(i) += f;
        }
        //alignment 
        for (int i = 1; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            if (useMidpoint)
                posi = midpoint_pos.col(i);
            TV veli = velocities.col(i);
            if (useMidpoint)
                veli = midpoint_vel.col(i);

            int boids_in_range = 0;
            TV average_vel = TV(0.0f, 0.0f);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                if (useMidpoint)
                    posj = midpoint_pos.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_radius) {
                    boids_in_range++;
                    average_vel += velocities.col(j);
                }
            }
            if (boids_in_range > 0)
                average_vel /= boids_in_range;
            else
                average_vel = veli;

            TV alignment_vector = veli - average_vel;

            f = alignment_vector * -m_AlignmentStrength;
            force.col(i) += f;
        }
        //separation
        for (int i = 1; i < getParticleNumber(); i++) {
            TV f = force.col(i);
            TV posi = positions.col(i);
            if (useMidpoint)
                posi = midpoint_pos.col(i);
            TV sep_vector = TV(0.0, 0.0);
            for (int j = 0; j < getParticleNumber(); j++) {
                if (i == j)
                    continue;
                TV posj = positions.col(j);
                if (useMidpoint)
                    posj = midpoint_pos.col(j);
                TV distance = posi - posj;
                float x = distance.norm();
                if (distance.norm() <= m_sepDis) {
                    sep_vector = posi - posj;
                    break;
                }
            }

            f = sep_vector * m_sepStr;
            force.col(i) += f;
        }
        //follow leader
        for (int i = 1; i < getParticleNumber(); i++) {
        
            TV f = force.col(i);
            TV posi = positions.col(i);

            TV leaderPosition = positions.col(0);//leaderPos;

            TV dir = (leaderPosition - posi);
            dir = dir.normalized();
            f = dir * m_leaderStrength;

            force.col(i) += f;
        }

        //leader follows mouse
        {
            TV posi = positions.col(0);//leaderPos;
            TV dir = (leaderPos - posi);
            float distance = dir.norm();
            
            TV f;

            dir = dir.normalized();
 
            f = dir * 0.4f;
 
            force.col(0) = f;
  
            
        }
        //collision avoidance
        /*
        for (int i = 0; i < getParticleNumber(); i++) {
            TV f = TV(0,0);
            TV posi = positions.col(i);
            TV veli = velocities.col(i);

            if (collision(posi)) {
                TV dir = (posi - circlePos);
                    dir = dir.normalized();
                f = dir * 10;
            }
            
            obstacle_force.col(i) = f;
        }
        */
        return force;
    }
    void updateBehavior(MethodTypes type, IntegrationMethodTypes IntType)
    {

        TVStack force = TVStack::Zero(dim, n);
        if (!update)
            return;
        switch (type)
        {
        case FREEFALL:
            if (IntType = SYMPLETIC)
                updatePositions();
                updatePositions();
                
            force = applyFreefall();

            if(IntType == EXPLICIT)
                integrateExplicitEuler(force);
            if (IntType = SYMPLETIC)
                integrateSympleticEuler(force);
            if (IntType == MIDPOINT) {
                integrateMidpoint1(force);
                force = applyFreefall(true);
                integrateMidpoint2(force);
            }
            break;
        case COHESION:
            if (IntType = SYMPLETIC)
                updatePositions();
            
            force = applyCohesion();

            if (IntType == EXPLICIT)
                integrateExplicitEuler(force);
            if (IntType = SYMPLETIC)
                integrateSympleticEuler(force);
            break;

        case ALIGNMENT:
            if (IntType = SYMPLETIC)
                updatePositions();
            
            force = applyAlignment();

            if (IntType == EXPLICIT)
                integrateExplicitEuler(force);
            if (IntType = SYMPLETIC)
                integrateSympleticEuler(force);
            break;

        case SEPARATION:
            if (IntType = SYMPLETIC)
                updatePositions();
           
            force = applySeparation();

            if (IntType == EXPLICIT)
                integrateExplicitEuler(force);
            if (IntType = SYMPLETIC)
                integrateSympleticEuler(force);
            break;


        case ALL:
            if (IntType = SYMPLETIC)
                updatePositions();
            
            force = applyAll();
            
            if (IntType == EXPLICIT)
                integrateExplicitEuler(force);
            if (IntType = SYMPLETIC)
                integrateSympleticEuler(force);

            if (IntType == MIDPOINT) {
                integrateMidpoint1(force);
                force = applyAll(true);
                integrateMidpoint2(force);
            }
            break;

        case LEADER:

            force = applyLeader();
            if (IntType == EXPLICIT)
                integrateExplicitEuler(force);
            if (IntType = SYMPLETIC)
                integrateSympleticEuler(force);
            break;
        }

        spawnBoids();
        despawnBoids();

    }

    int help = 0;
    void spawnBoids() {

        help++;
        if (help > 50) {
            help = 0;
            for (int i = 0; i < getParticleNumber() - 1; i++) {
                TV posi = positions.col(i);
                TV groupi = boidGroups.col(i);
                for (int j = 1; j < getParticleNumber(); j++) {
                    if (i == j)
                        continue;
                    TV posj = positions.col(j);
                    TV groupj = boidGroups.col(i);
                    float distance = (posi - posj).norm();
                    if (groupi[0] == groupj[0] && distance < m_newBoidDistance) {
                        addBoid((posi + posj) / 2.0f, (BoidGroups)groupi[0]);
                        return;
                    }                       
                }

            }
        }

    }

    void despawnBoids(){
        for (int i = 0; i < getParticleNumber(); i++) {
            //boid to be tested
            TV posi = positions.col(i);
            TV groupi = boidGroups.col(i);

            for (int j = 0; j < getParticleNumber(); j++) {
                //first other boid
                TV posj = positions.col(j);
                TV groupj = boidGroups.col(j);
                if (groupi[0] == groupj[0])
                    continue;
                float distanceij = (posi - posj).norm();
                if (distanceij > m_despawnRadius)
                    continue;

                for (int k = 0; k < getParticleNumber(); k++) {
                    //second other boid
                    TV posk = positions.col(k);
                    TV groupk = boidGroups.col(k);
                    if (groupi[0] == groupk[0])
                        continue;
                    float distanceik = (posi - posk).norm();
                    if (distanceik > m_despawnRadius)
                        continue;

                    for (int l = 0; l < getParticleNumber(); l++) {
                        //third other boid
                        TV posl = positions.col(l);
                        TV groupl = boidGroups.col(l);
                        if (groupi[0] == groupl[0])
                            continue;
                        float distanceil = (posi - posl).norm();
                        if (distanceil > m_despawnRadius)
                            continue;
                        removeBoid(i);


                    }

                }

            }
        
        }
    }

    void updatePositions() {
        for (int i = 0; i < getParticleNumber(); i++) {
            TV pos = positions.col(i);
            TV vel = velocities.col(i);
            pos += h * vel;

            positions.col(i) = pos;       
        }

    }

    void integrateExplicitEuler(TVStack force) {
        //update Positions and velocities using explicit euler
        for (int i = 0; i < getParticleNumber(); i++) {
            TV pos = positions.col(i);
            TV vel = velocities.col(i);
            TV f;
          
            f = force.col(i) + obstacle_force.col(i);

            pos += h * vel;
            vel += h * f;

            positions.col(i) = pos;
            velocities.col(i) = vel;
        }
    }

    void integrateSympleticEuler(TVStack force) {
        //update Positions and velocities using explicit euler
        for (int i = 0; i < getParticleNumber(); i++) {
            TV vel = velocities.col(i);
            TV f = force.col(i);
       
            vel += h * f;    
            velocities.col(i) = vel;
        }
    }

    void integrateMidpoint1(TVStack force) {
        //update Positions and velocities using explicit euler
        for (int i = 0; i < getParticleNumber(); i++) {
            TV pos = positions.col(i);
            TV vel = velocities.col(i);
            TV f = force.col(i);

            pos += h / 2.0f * vel;
            vel += h / 2.0f * f;

            midpoint_pos.col(i) = pos;
            midpoint_vel.col(i) = vel;
        }
    }

    void integrateMidpoint2(TVStack force) {
        //update Positions and velocities using explicit euler
        for (int i = 0; i < getParticleNumber(); i++) {
            TV pos = midpoint_pos.col(i);
            TV vel = midpoint_vel.col(i);
            TV f = force.col(i);

            pos += h * vel;
            vel += h * f;

            positions.col(i) = pos;
            velocities.col(i) = vel;
        }
    }


    bool collision(TV boid) {

        if ((boid - circlePos).norm() <= circleRadius) {
            return true;
        }
        return false;
    }

    void pause()
    {
        update = !update;
    }
    TVStack getPositions()
    {
        return positions;
    }
};
#endif
