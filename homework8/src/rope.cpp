#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

const double damping_factor = 0.00005;

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }

        Vector2D step = (end-start)/(num_nodes-1);

        for(int i=0;i<num_nodes;i++){
            Mass* nodeCur = new Mass(start+i*step,node_mass,false);
            masses.push_back(nodeCur);
            if(i==0) {
                masses[i]->pinned = true;
                continue;
            }
            Spring* springCur = new Spring(this->masses[i-1],this->masses[i],k);
            springs.push_back(springCur);
        }
        
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D vB2A = s->m2->position-s->m1->position;
            double dis = vB2A.norm();    

            Vector2D fB2A = s->k * vB2A * (dis - s->rest_length) / dis;
            Vector2D fA2B = -fB2A;

            s->m1->forces += fB2A;
            s->m2->forces += fA2B;

            Vector2D fB = 0.05 * vB2A / dis * (s->m2->velocity - s->m1->velocity) * vB2A / dis; 
            Vector2D fA = -fB;

            s->m1->forces += fA;
            s->m2->forces += fB;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position

                // TODO (Part 2): Add global damping

                m->forces += gravity;
                m->forces -= damping_factor * m->velocity;
                m->velocity += m->forces / m->mass * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D vB2A = s->m2->position-s->m1->position;
            double dis = vB2A.norm();    

            Vector2D fB2A = -s->k * vB2A * (dis - s->rest_length) / dis;
            Vector2D fA2B = -fB2A;

            s->m1->forces += fA2B;
            s->m2->forces += fB2A;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                m->forces += gravity;

                //m->position += (m->position - m->last_position) + m->forces/m->mass * delta_t * delta_t;

                // TODO (Part 4): Add global Verlet damping
                m->position += (1-damping_factor) * (m->position - m->last_position) + m->forces/m->mass * delta_t * delta_t;
                m->last_position = temp_position;

            }

            m->forces = Vector2D(0,0);
        }
    }
}
