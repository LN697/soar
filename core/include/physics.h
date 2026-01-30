#pragma once

#include "types.h"

#include <vector>
#include <iostream>

struct RigidBody {
    double mass = 1.0;
    Mat3 inertia = Mat3::Identity();
    Mat3 inertia_inv = Mat3::Identity();

    Vec3 pos = Vec3::Zero();
    Quat ori = Quat::Identity();
    Vec3 vel = Vec3::Zero();
    Vec3 ang_vel = Vec3::Zero(); 

    Vec3 force_accum = Vec3::Zero();
    Vec3 torque_accum = Vec3::Zero();

    void setInertia(const Mat3& I) {
        inertia = I;
        inertia_inv = I.inverse();
    }

    void clearForces() {
        force_accum.setZero();
        torque_accum.setZero();
    }

    void addForceAtLocalPos(const Vec3& force_world, const Vec3& pos_local) {
        force_accum += force_world;
        
        Vec3 r_world = ori * pos_local; 
        torque_accum += r_world.cross(force_world);
    }

    void addTorqueLocal(const Vec3& torque_local) {
        torque_accum += ori * torque_local; // Convert to world for integration
    }

    void integrate(double dt) {
        Vec3 acc = force_accum / mass;
        vel += acc * dt;
        pos += vel * dt;

        Vec3 torque_body = ori.inverse() * torque_accum;
        
        Vec3 gyroscopic = ang_vel.cross(inertia * ang_vel);
        Vec3 ang_acc = inertia_inv * (torque_body - gyroscopic);
        ang_vel += ang_acc * dt;

        Quat q_dot;
        q_dot.w() = 0;
        q_dot.vec() = ang_vel;
        Quat deriv = ori * q_dot;
        
        ori.w() += deriv.w() * 0.5 * dt;
        ori.vec() += deriv.vec() * 0.5 * dt;
        ori.normalize();

        if (pos.z() < 0.0) {
            pos.z() = 0.0;
            if (vel.z() < 0) vel.z() = 0.0;
            vel.x() *= 0.95; 
            vel.y() *= 0.95;
        }
    }
};

class ForceGenerator {
    public:
        virtual ~ForceGenerator() = default;
        virtual void apply(double dt) = 0;
};

class Constraint {
    public:
        virtual ~Constraint() = default;
        virtual void apply(double dt) = 0;
};

class MultibodySystem {
    public:
        std::vector<RigidBody*> bodies;
        std::vector<ForceGenerator*> forces;
        std::vector<Constraint*> constraints;

        void addBody(RigidBody* b) { bodies.push_back(b); }
        void addForce(ForceGenerator* f) { forces.push_back(f); }

        void step(double dt) {
            for (auto* b : bodies) b->clearForces();
            for (auto* f : forces) f->apply(dt);
            for (auto* c : constraints) c->apply(dt); // TODO: Implement solver loop here
            for (auto* b : bodies) b->integrate(dt);
        }
};

class GravityForce : public ForceGenerator {
        RigidBody* body;
        Vec3 g;
    public:
        GravityForce(RigidBody* b, double gravity = 9.81) : body(b), g(0, 0, -gravity) {}
        void apply(double) override {
            body->force_accum += g * body->mass;
        }
};

class DragForce : public ForceGenerator {
        RigidBody* body;
        double k_lin;
        double k_ang;
    public:
        DragForce(RigidBody* b, double kl, double ka) : body(b), k_lin(kl), k_ang(ka) {}
        void apply(double) override {
            body->force_accum -= body->vel * k_lin * body->vel.norm();
            body->addTorqueLocal(-body->ang_vel * k_ang);
        }
};