/**
 * @file ElectricalSystem.cpp
 * @brief Implementation of bus current aggregation, internal resistance sag, and brownout tripping.
 */

#include "Actuation/Systems/ElectricalSystem.h"
#include "Robot/Mechanisms/Components/ElevatorComponent.h"
#include "Robot/Mechanisms/Components/ArmComponent.h"
#include "Robot/Mechanisms/Components/FlywheelComponent.h"
#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"

#include <algorithm>
#include <cmath>

namespace FRCBananaSim::Actuation::Systems
{
    ElectricalSystem::ElectricalSystem(Core::ECS::Core::Scene& scene,
        FRCBananaSim::Robot::Drivetrain::System::SwerveDriveSystem* swerveSystem)
        : ISystem(scene),
        swerveSystem(swerveSystem),
        externalOutputs(nullptr),
        idleCurrentA(1.5),
        currentTerminalVoltage(12.60),
        currentTotalCurrent(1.5),
        isBrownedOutState(false)
    {}

    ElectricalSystem::~ElectricalSystem()
    {}

    void ElectricalSystem::SetSwerveSystem(FRCBananaSim::Robot::Drivetrain::System::SwerveDriveSystem* system)
    {
        swerveSystem = system;
    }

    void ElectricalSystem::SetOutputs(Control::SimOutputs* outputs)
    {
        externalOutputs = outputs;
    }

    double ElectricalSystem::GetTerminalVoltage() const
    {
        return currentTerminalVoltage;
    }

    double ElectricalSystem::GetTotalCurrent() const
    {
        return currentTotalCurrent;
    }

    bool ElectricalSystem::IsBrownedOut() const
    {
        return isBrownedOutState;
    }

    void ElectricalSystem::SetIdleCurrent(double idleCurrent)
    {
        idleCurrentA = idleCurrent;
    }

    void ElectricalSystem::FixedUpdate(double dt)
    {
        SIM_PROFILE_SCOPE("ElectricalSystem::FixedUpdate");

        if (dt <= 1e-7)
        {
            return;
        }

        double totalAmps = idleCurrentA;

        // 1. Swerve Drivetrain Supply Current Aggregation
        if (swerveSystem)
        {
            auto* model = swerveSystem->GetModel();
            if (model)
            {
                totalAmps += model->GetTotalSupplyCurrent();
            }
        }

        // 2. Elevators Supply Current Aggregation
        auto elevatorEntities = scene.Query<Robot::Mechanisms::Components::ElevatorComponent>();
        for (auto entity : elevatorEntities)
        {
            auto* elevator = scene.GetComponent<Robot::Mechanisms::Components::ElevatorComponent>(entity);
            if (elevator)
            {
                double duty = std::clamp(std::abs(elevator->appliedVoltage) / std::max(1.0, currentTerminalVoltage), 0.0, 1.0);
                double supply = std::min(elevator->statorCurrentLimitA, duty * std::abs(elevator->statorCurrentA));
                totalAmps += supply * static_cast<double>(std::max(1u, elevator->motorCount));
            }
        }

        // 3. Arms Supply Current Aggregation
        auto armEntities = scene.Query<Robot::Mechanisms::Components::ArmComponent>();
        for (auto entity : armEntities)
        {
            auto* arm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(entity);
            if (arm)
            {
                double duty = std::clamp(std::abs(arm->appliedVoltage) / std::max(1.0, currentTerminalVoltage), 0.0, 1.0);
                double supply = std::min(arm->statorCurrentLimitA, duty * std::abs(arm->statorCurrentA));
                totalAmps += supply * static_cast<double>(std::max(1u, arm->motorCount));
            }
        }

        // 4. Flywheels Supply Current Aggregation
        auto flywheelEntities = scene.Query<Robot::Mechanisms::Components::FlywheelComponent>();
        for (auto entity : flywheelEntities)
        {
            auto* flywheel = scene.GetComponent<Robot::Mechanisms::Components::FlywheelComponent>(entity);
            if (flywheel)
            {
                double duty = std::clamp(std::abs(flywheel->appliedVoltage) / std::max(1.0, currentTerminalVoltage), 0.0, 1.0);
                double supply = std::min(flywheel->statorCurrentLimitA, duty * std::abs(flywheel->statorCurrentA));
                totalAmps += supply * static_cast<double>(std::max(1u, flywheel->motorCount));
            }
        }

        // 5. Battery Model Calculation & Brownout Tripping
        auto batteryEntities = scene.Query<Components::BatteryComponent>();
        if (!batteryEntities.empty())
        {
            auto* battery = scene.GetComponent<Components::BatteryComponent>(batteryEntities[0]);
            if (battery)
            {
                currentTerminalVoltage = battery->batteryModel.CalculateTerminalVoltage(totalAmps);
                battery->terminalVoltage = currentTerminalVoltage;
                battery->totalCurrent = totalAmps;

                // WPILib standard brownout thresholds: Stage 1 = 6.8V warning, Stage 2 = 6.3V critical shutoff
                battery->isBrownoutWarning = (currentTerminalVoltage < 6.8);
                battery->isBrownoutCritical = (currentTerminalVoltage < 6.3);
                isBrownedOutState = battery->isBrownoutCritical;
            }
        }
        else
        {
            currentTerminalVoltage = std::max(0.0, 12.60 - (totalAmps * 0.018));
            isBrownedOutState = (currentTerminalVoltage < 6.3);
        }

        currentTotalCurrent = totalAmps;

        // 6. Update SimOutputs
        if (externalOutputs)
        {
            externalOutputs->batteryTerminalVoltage = currentTerminalVoltage;
            externalOutputs->totalBusCurrent = currentTotalCurrent;
            externalOutputs->isBrownedOut = isBrownedOutState;
        }
    }
}