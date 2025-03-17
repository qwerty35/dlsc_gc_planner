/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"

namespace MAPF {
    class PIBT : public Solver {
    public:
        static const std::string SOLVER_NAME;

    private:
        // PIBT agent
        struct Agent {
            int id;
            Node *v_now;        // current location
            Node *v_next;       // next location
            Node *g;            // goal
            Node *o;            // closest obstacle
            int elapsed;        // eta
            int init_d;         // initial distance
            float tie_breaker;  // epsilon, tie-breaker
            float obs_d;        // distance to obstacle
        };
        using Agents = std::vector<Agent *>;

        // <node-id, agent>, whether the node is occupied or not
        // work as reservation table
        Agents occupied_now;
        Agents occupied_next;

        // option
        bool disable_dist_init = false;

        // result of priority inheritance: true -> valid, false -> invalid
        bool funcPIBT(Agent *ai);

        // plan next node
        Node *planOneStep(Agent *a);

        // chose one node from candidates, used in planOneStep
        Node *chooseNode(Agent *a);

        // main
        void run();

    public:
        PIBT(Problem *_P);

        ~PIBT() {}

        void setParams(int argc, char *argv[]);

        static void printHelp();

        static float obsDist(Agent *a, Node* s);
    };
}