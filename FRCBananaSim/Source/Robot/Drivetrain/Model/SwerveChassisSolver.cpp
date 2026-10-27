/**
 * @file SwerveChassisSolver.cpp
 * @brief Implementation of 6-DOF implicit dynamics solver with 6x6 Cholesky factorisation.
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
     * @brief Closed-form 6x6 Cholesky solver: L * L^T * x = b.
     * Operates purely on the stack with zero heap allocation.
     */
    static bool SolveCholesky6x6(const double A[6][6], const double b[6], double x[6])
    {
        double L[6][6] = { 0.0 };

        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                double sum = 0.0;
                for (int k = 0; k < j; ++k)
                {
                    sum += L[i][k] * L[j][k];
                }

                if (i == j)
                {
                    double diag = A[i][i] - sum;
                    if (diag <= 1e-12)
                    {
                        diag = 1e-12; // Regularization for singular airborne configurations
                    }
                    L[i][j] = std::sqrt(diag);
                }
                else
                {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }

        // Forward solve: L * y = b
        double y[6] = { 0.0 };
        for (int i = 0; i < 6; ++i)
        {
            double sum = 0.0;
            for (int k = 0; k < i; ++k)
            {
                sum += L[i][k] * y[k];
            }
            y[i] = (b[i] - sum) / L[i][i];
        }

        // Backward solve: L^T * x = y
        for (int i = 5; i >= 0; --i)
        {
            double sum = 0.0;
            for (int k = i + 1; k < 6; ++k)
            {
                sum += L[k][i] * x[k];
            }
            x[i] = (y[i] - sum) / L[i][i];
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

        // 3. Assemble 6x6 LHS matrix A and RHS 6-vector b
        // Variable index layout: [0..2] = v (vx, vy, vz), [3..5] = omega (wx, wy, wz)
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

        // RHS baseline: momentum + gravity + gyroscopic torque
        glm::vec3 gravityForce(0.0f, 0.0f, static_cast<float>(-9.80665 * chassisParams.mass)); // NWU: -Z is down
        glm::vec3 linMomentum = (static_cast<float>(massOverDt) * currentState.linearVelocity) + gravityForce;

        // Euler gyroscopic torque: tau_gyro = -omega x (I_w * omega)
        glm::vec3 angularMomentum = worldInertia * currentState.angularVelocity;
        glm::vec3 gyroTorque = -glm::cross(currentState.angularVelocity, angularMomentum);
        glm::vec3 angMomentum = (inertiaOverDt * currentState.angularVelocity) + gyroTorque;

        b[0] = linMomentum.x;
        b[1] = linMomentum.y;
        b[2] = linMomentum.z;
        b[3] = angMomentum.x;
        b[4] = angMomentum.y;
        b[5] = angMomentum.z;

        // 4. Accumulate tire contact stiffnesses and intercept forces into A and b
        for (uint32_t i = 0; i < 4; ++i)
        {
            const auto& contact = wheelContacts[i];
            if (!contact.hasContact)
            {
                continue;
            }

            // Contact lever arm relative to chassis center of mass
            glm::vec3 r = contact.contactPoint - currentState.position;

            // Contact tangent frame vectors
            glm::vec3 eZ = contact.surfaceNormal;
            glm::vec3 wheelHeading = rotMatrix * glm::vec3(1.0f, 0.0f, 0.0f); // Forward
            glm::vec3 eX = glm::normalize(wheelHeading - (eZ * glm::dot(wheelHeading, eZ)));
            glm::vec3 eY = glm::cross(eZ, eX);

            // Construct 3x6 contact Jacobian J_i: rows are eX, eY, eZ with (r x e)
            glm::vec3 rxEx = glm::cross(r, eX);
            glm::vec3 rxEy = glm::cross(r, eY);
            glm::vec3 rxEz = glm::cross(r, eZ);

            double J[3][6] = {
                { eX.x, eX.y, eX.z, rxEx.x, rxEx.y, rxEx.z },
                { eY.x, eY.y, eY.z, rxEy.x, rxEy.y, rxEy.z },
                { eZ.x, eZ.y, eZ.z, rxEz.x, rxEz.y, rxEz.z }
            };

            // Diagonal stiffness in contact frame: [Kx, Ky, Kn]
            double K[3] = {
                contact.tangentialStiffness.stiffnessX,
                contact.tangentialStiffness.stiffnessY,
                contact.normalStiffnessKn
            };

            // Intercept force in contact frame: [f0x, f0y, kt * d]
            double f0[3] = {
                contact.tangentialStiffness.interceptForce.x,
                contact.tangentialStiffness.interceptForce.y,
                contact.normalStiffnessKn * contact.penetrationDepth / deltaTime // Consistent normal spring
            };

            // Accumulate J^T * K * J into A
            for (int k = 0; k < 3; ++k)
            {
                if (K[k] <= 1e-6)
                {
                    continue;
                }

                for (int row = 0; row < 6; ++row)
                {
                    for (int col = 0; col < 6; ++col)
                    {
                        A[row][col] += J[k][row] * K[k] * J[k][col];
                    }
                    b[row] += J[k][row] * f0[k];
                }
            }
        }

        // 5. Solve 6x6 linear system via Cholesky decomposition
        double x[6] = { 0.0 };
        SolveCholesky6x6(A, b, x);

        outNewLinearVelocity = glm::vec3(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
        outNewAngularVelocity = glm::vec3(static_cast<float>(x[3]), static_cast<float>(x[4]), static_cast<float>(x[5]));
    }
}