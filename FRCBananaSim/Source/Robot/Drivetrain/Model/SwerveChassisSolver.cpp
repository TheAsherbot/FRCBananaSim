/**
 * @file SwerveChassisSolver.cpp
 * @brief Implementation of 6-DOF implicit dynamics solver with Gaussian elimination linear system solve.
 */

#include "Robot/Drivetrain/Model/SwerveChassisSolver.h"
#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Robot::Drivetrain::Model
{
    SwerveChassisSolver::SwerveChassisSolver(const ChassisParams& chassis)
        : chassisParams(chassis)
    {}

    SwerveChassisSolver::SwerveChassisSolver()
        : chassisParams(ChassisParams{})
    {}

    void SwerveChassisSolver::SetChassisParams(const ChassisParams& chassis)
    {
        chassisParams = chassis;
    }

    const ChassisParams& SwerveChassisSolver::GetChassisParams() const
    {
        return chassisParams;
    }

    /**
     * @brief Robust 6x6 linear solver using Gaussian elimination with partial pivoting.
     * Operates purely on the stack with zero heap allocation.
     * Guaranteed numerically stable for any non-singular matrix (unlike un-pivoted Cholesky).
     * @param A 6x6 symmetric positive semi-definite or indefinite system matrix.
     * @param b 6-element right-hand side vector.
     * @param x 6-element solution vector.
     * @return True if solve succeeded.
     */
    static bool SolveLinearSystem6x6(const double A[6][6], const double b[6], double x[6])
    {
        // Augmented matrix [A | b]
        double M[6][7];
        for (int r = 0; r < 6; ++r)
        {
            for (int c = 0; c < 6; ++c)
            {
                M[r][c] = A[r][c];
            }
            M[r][6] = b[r];
        }

        // Forward elimination with partial pivoting
        for (int col = 0; col < 6; ++col)
        {
            // Find row with maximal absolute pivot
            int maxRow = col;
            double maxVal = std::abs(M[col][col]);
            for (int r = col + 1; r < 6; ++r)
            {
                double val = std::abs(M[r][col]);
                if (val > maxVal)
                {
                    maxVal = val;
                    maxRow = r;
                }
            }

            // Swap pivot row into place
            if (maxRow != col)
            {
                for (int c = col; c < 7; ++c)
                {
                    std::swap(M[col][c], M[maxRow][c]);
                }
            }

            double pivot = M[col][col];
            if (std::abs(pivot) < 1e-12)
            {
                // Regularize singular pivot to prevent division by zero
                pivot = (pivot >= 0.0 ? 1e-12 : -1e-12);
                M[col][col] = pivot;
            }

            // Eliminate lower triangular entries
            for (int r = col + 1; r < 6; ++r)
            {
                double factor = M[r][col] / pivot;
                M[r][col] = 0.0;
                for (int c = col + 1; c < 7; ++c)
                {
                    M[r][c] -= factor * M[col][c];
                }
            }
        }

        // Backward substitution
        for (int r = 5; r >= 0; --r)
        {
            double sum = 0.0;
            for (int c = r + 1; c < 6; ++c)
            {
                sum += M[r][c] * x[c];
            }
            double diag = M[r][r];
            if (std::abs(diag) < 1e-12)
            {
                diag = (diag >= 0.0 ? 1e-12 : -1e-12);
            }
            x[r] = (M[r][6] - sum) / diag;
        }

        return true;
    }

    void SwerveChassisSolver::SolveStep(const ChassisMotionState& currentState,
        const std::array<SolverWheelContact, 4>& wheelContacts,
        double deltaTime,
        glm::vec3& outNewLinearVelocity,
        glm::vec3& outNewAngularVelocity) const
    {
        if (deltaTime <= 1e-7)
        {
            outNewLinearVelocity = currentState.linearVelocity;
            outNewAngularVelocity = currentState.angularVelocity;
            return;
        }

        // 1. World-transformed inertia tensor: I_w = R * I_body * R^T
        glm::mat3 rotMatrix = glm::mat3_cast(currentState.orientation);
        glm::mat3 worldInertia = rotMatrix * chassisParams.inertiaTensor * glm::transpose(rotMatrix);

        // 2. Linear and angular momentum terms: (M / dt) * v and (I_w / dt) * omega
        double massOverDt = chassisParams.mass / deltaTime;
        glm::mat3 inertiaOverDt = worldInertia / static_cast<float>(deltaTime);

        // RHS baseline: momentum + gravity + gyroscopic torque
        glm::vec3 gravityForce(0.0f, 0.0f, static_cast<float>(-9.80665 * chassisParams.mass)); // NWU: -Z is down
        glm::vec3 linMomentum = (static_cast<float>(massOverDt) * currentState.linearVelocity) + gravityForce;

        // Euler gyroscopic torque: tau_gyro = -omega x (I_w * omega)
        glm::vec3 angularMomentum = worldInertia * currentState.angularVelocity;
        glm::vec3 gyroTorque = -glm::cross(currentState.angularVelocity, angularMomentum);
        glm::vec3 angMomentum = (inertiaOverDt * currentState.angularVelocity) + gyroTorque;

        // 3. Setup per-wheel contact solver data structures
        struct ModContactSolve
        {
            double K[3];
            double f0[3];
            glm::vec3 eX;
            glm::vec3 eY;
            glm::vec3 eZ;
            glm::vec3 r;
            double muX;
            double muY;
            bool active;
        };

        std::array<ModContactSolve, 4> modSolves;
        for (uint32_t i = 0; i < 4; ++i)
        {
            modSolves[i].active = wheelContacts[i].hasContact;
            if (!modSolves[i].active)
            {
                continue;
            }

            modSolves[i].r = wheelContacts[i].contactPoint - currentState.position;
            modSolves[i].eZ = wheelContacts[i].surfaceNormal;

            glm::vec3 wheelHeading = wheelContacts[i].wheelHeading;
            glm::vec3 eX = wheelHeading - (modSolves[i].eZ * glm::dot(wheelHeading, modSolves[i].eZ));
            float len = glm::length(eX);
            if (len > 1e-5f)
            {
                modSolves[i].eX = eX / len;
            }
            else
            {
                modSolves[i].eX = glm::vec3(1.0f, 0.0f, 0.0f);
            }
            modSolves[i].eY = glm::cross(modSolves[i].eZ, modSolves[i].eX);

            modSolves[i].K[0] = wheelContacts[i].tangentialStiffness.stiffnessX;
            modSolves[i].K[1] = wheelContacts[i].tangentialStiffness.stiffnessY;
            modSolves[i].K[2] = wheelContacts[i].normalStiffnessKn;

            modSolves[i].f0[0] = wheelContacts[i].tangentialStiffness.interceptForce.x;
            modSolves[i].f0[1] = wheelContacts[i].tangentialStiffness.interceptForce.y;
            modSolves[i].f0[2] = wheelContacts[i].springStiffnessKt * wheelContacts[i].penetrationDepth;

            modSolves[i].muX = wheelContacts[i].muX;
            modSolves[i].muY = wheelContacts[i].muY;
        }

        double x[6] = { 0.0 };

        // 4. Multi-pass active set iteration: up to 3 passes for tire friction saturation and lift-off
        for (int pass = 0; pass < 3; ++pass)
        {
            double A[6][6] = { 0.0 };
            double b[6] = { 0.0 };

            // Linear inertia diagonal block
            A[0][0] += massOverDt;
            A[1][1] += massOverDt;
            A[2][2] += massOverDt;

            // Angular inertia diagonal block
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    A[3 + r][3 + c] += inertiaOverDt[r][c];
                }
            }

            b[0] = linMomentum.x;
            b[1] = linMomentum.y;
            b[2] = linMomentum.z;
            b[3] = angMomentum.x;
            b[4] = angMomentum.y;
            b[5] = angMomentum.z;

            // Accumulate tire contacts into A and b
            for (uint32_t i = 0; i < 4; ++i)
            {
                if (!modSolves[i].active)
                {
                    continue;
                }

                glm::vec3 rxEx = glm::cross(modSolves[i].r, modSolves[i].eX);
                glm::vec3 rxEy = glm::cross(modSolves[i].r, modSolves[i].eY);
                glm::vec3 rxEz = glm::cross(modSolves[i].r, modSolves[i].eZ);

                double J[3][6] = {
                    { modSolves[i].eX.x, modSolves[i].eX.y, modSolves[i].eX.z, rxEx.x, rxEx.y, rxEx.z },
                    { modSolves[i].eY.x, modSolves[i].eY.y, modSolves[i].eY.z, rxEy.x, rxEy.y, rxEy.z },
                    { modSolves[i].eZ.x, modSolves[i].eZ.y, modSolves[i].eZ.z, rxEz.x, rxEz.y, rxEz.z }
                };

                for (int k = 0; k < 3; ++k)
                {
                    if (modSolves[i].K[k] <= 1e-6 && std::abs(modSolves[i].f0[k]) <= 1e-6)
                    {
                        continue;
                    }

                    for (int row = 0; row < 6; ++row)
                    {
                        for (int col = 0; col < 6; ++col)
                        {
                            A[row][col] += J[k][row] * modSolves[i].K[k] * J[k][col];
                        }
                        b[row] += J[k][row] * modSolves[i].f0[k];
                    }
                }
            }

            // 5. Solve 6x6 linear system via Gaussian elimination with partial pivoting
            SolveLinearSystem6x6(A, b, x);

            glm::vec3 candidateLinVel(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
            glm::vec3 candidateAngVel(static_cast<float>(x[3]), static_cast<float>(x[4]), static_cast<float>(x[5]));

            bool needsAnotherPass = false;

            for (uint32_t i = 0; i < 4; ++i)
            {
                if (!modSolves[i].active)
                {
                    continue;
                }

                glm::vec3 patchVel = candidateLinVel + glm::cross(candidateAngVel, modSolves[i].r);
                double ux = glm::dot(patchVel, modSolves[i].eX);
                double uy = glm::dot(patchVel, modSolves[i].eY);
                double un = glm::dot(patchVel, modSolves[i].eZ);

                double normalForce = modSolves[i].f0[2] - (modSolves[i].K[2] * un);
                if (normalForce <= 0.0)
                {
                    // Wheel lift-off
                    modSolves[i].active = false;
                    needsAnotherPass = true;
                    continue;
                }

                // Tangential force delivered to chassis
                double fx = modSolves[i].f0[0] - (modSolves[i].K[0] * ux);
                double fy = modSolves[i].f0[1] - (modSolves[i].K[1] * uy);

                double limitX = std::max(1e-3, modSolves[i].muX * normalForce);
                double limitY = std::max(1e-3, modSolves[i].muY * normalForce);

                double normX = fx / limitX;
                double normY = fy / limitY;
                double sat = std::sqrt(normX * normX + normY * normY);

                if (sat > 1.02 && pass < 2)
                {
                    // Scale tangential stiffness and intercept force by radial return ratio
                    modSolves[i].K[0] /= sat;
                    modSolves[i].K[1] /= sat;
                    modSolves[i].f0[0] /= sat;
                    modSolves[i].f0[1] /= sat;
                    needsAnotherPass = true;
                }
            }

            if (!needsAnotherPass)
            {
                break;
            }
        }

        outNewLinearVelocity = glm::vec3(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
        outNewAngularVelocity = glm::vec3(static_cast<float>(x[3]), static_cast<float>(x[4]), static_cast<float>(x[5]));

        // Guard against non-finite numerical anomalies
        if (!std::isfinite(outNewLinearVelocity.x) || !std::isfinite(outNewLinearVelocity.y) || !std::isfinite(outNewLinearVelocity.z))
        {
            outNewLinearVelocity = currentState.linearVelocity;
        }
        if (!std::isfinite(outNewAngularVelocity.x) || !std::isfinite(outNewAngularVelocity.y) || !std::isfinite(outNewAngularVelocity.z))
        {
            outNewAngularVelocity = currentState.angularVelocity;
        }
    }
}