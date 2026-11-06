/**

 * @file ElectricalSystem.cpp

 * @brief Implementation of bus current aggregation, internal resistance sag, regulated servo rail, and brownout tripping.

 */



#include "Actuation/Systems/ElectricalSystem.h"

#include "Actuation/Components/ServoComponent.h"

#include "Robot/Mechanisms/Components/ElevatorComponent.h"

#include "Robot/Mechanisms/Components/ArmComponent.h"

#include "Robot/Mechanisms/Components/FlywheelComponent.h"

#include "Robot/Mechanisms/Components/DifferentialComponent.h"

#include "Telemetry/TelemeteryViewer/TelemetryMacros.h"



#include <algorithm>

#include <cmath>



namespace FRCBananaSim::Actuation::Systems

{

    ElectricalSystem::ElectricalSystem(Core::ECS::Core::Scene& scene,

        FRCBananaSim::Robot::Drivetrain::Systems::SwerveDriveSystem* swerveSystem)

        : ISystem(scene),

        swerveSystem(swerveSystem),

        externalOutputs(nullptr),

        idleCurrentA(1.5),

        currentTerminalVoltage(12.60),

        currentTotalCurrent(1.5),

        isBrownedOutState(false),

        regulatedRailNominalVoltage(6.0),

        regulatedRailCurrentLimit(2.2),

        regulatedRailEfficiency(0.88),

        currentRegulatedRailVoltage(6.0),

        currentRegulatedRailCurrent(0.0),

        isRegulatedRailBrownedOutState(false)

    {}



    ElectricalSystem::~ElectricalSystem()

    {}



    void ElectricalSystem::SetSwerveSystem(FRCBananaSim::Robot::Drivetrain::Systems::SwerveDriveSystem* system)

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



    void ElectricalSystem::SetRegulatedRail(double nominalVolts, double currentLimitA, double efficiency)

    {

        regulatedRailNominalVoltage = std::max(0.0, nominalVolts);

        regulatedRailCurrentLimit = std::max(0.0, currentLimitA);

        regulatedRailEfficiency = std::clamp(efficiency, 0.1, 1.0);

        currentRegulatedRailVoltage = regulatedRailNominalVoltage;

    }



    double ElectricalSystem::GetRegulatedRailVoltage() const

    {

        return currentRegulatedRailVoltage;

    }



    double ElectricalSystem::GetRegulatedRailCurrent() const

    {

        return currentRegulatedRailCurrent;

    }



    bool ElectricalSystem::IsRegulatedRailBrownedOut() const

    {

        return isRegulatedRailBrownedOutState;

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



        // 2. Elevators Supply Current Aggregation (CAN motors)

        auto elevatorEntities = scene.Query<Robot::Mechanisms::Components::ElevatorComponent>();

        for (auto entity : elevatorEntities)

        {

            auto* elevator = scene.GetComponent<Robot::Mechanisms::Components::ElevatorComponent>(entity);

            if (elevator && elevator->actuatorType != Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                Actuation::DCMotorModel motor = elevator->motorModel.WithMotorCount(elevator->motorCount);

                double rDrum = std::max(elevator->drumRadiusM, 1e-4);

                double omegaRotor = (elevator->stages.empty() ? 0.0 : (elevator->stages[0].currentVelocityMps / rDrum)) * elevator->gearRatio;

                double statorI = std::abs(elevator->statorCurrentA);

                double vWinding = std::min(std::abs(elevator->appliedVoltage),

                    (statorI * motor.GetResistance()) + std::abs(motor.GetBackEmfConstantKe() * omegaRotor));

                double duty = std::clamp(vWinding / std::max(1.0, currentTerminalVoltage), 0.0, 1.0);

                double supply = std::min(elevator->statorCurrentLimitA, duty * statorI);

                totalAmps += supply;

            }

        }



        // 3. Arms Supply Current Aggregation (CAN motors)

        auto armEntities = scene.Query<Robot::Mechanisms::Components::ArmComponent>();

        for (auto entity : armEntities)

        {

            auto* arm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(entity);

            if (arm && arm->actuatorType != Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                Actuation::DCMotorModel motor = arm->motorModel.WithMotorCount(arm->motorCount);

                double omegaRotor = arm->currentVelocityRadPerSec * arm->gearRatio;

                double statorI = std::abs(arm->statorCurrentA);

                double vWinding = std::min(std::abs(arm->appliedVoltage),

                    (statorI * motor.GetResistance()) + std::abs(motor.GetBackEmfConstantKe() * omegaRotor));

                double duty = std::clamp(vWinding / std::max(1.0, currentTerminalVoltage), 0.0, 1.0);

                double supply = std::min(arm->statorCurrentLimitA, duty * statorI);

                totalAmps += supply;

            }

        }



        // 4. Flywheels Supply Current Aggregation (CAN motors)

        auto flywheelEntities = scene.Query<Robot::Mechanisms::Components::FlywheelComponent>();

        for (auto entity : flywheelEntities)

        {

            auto* flywheel = scene.GetComponent<Robot::Mechanisms::Components::FlywheelComponent>(entity);

            if (flywheel)

            {

                Actuation::DCMotorModel motor = flywheel->motorModel.WithMotorCount(flywheel->motorCount);

                double omegaRotor = flywheel->angularVelocityRadPerSec * flywheel->gearRatio;

                double statorI = std::abs(flywheel->statorCurrentA);

                double vWinding = std::min(std::abs(flywheel->appliedVoltage),

                    (statorI * motor.GetResistance()) + std::abs(motor.GetBackEmfConstantKe() * omegaRotor));

                double duty = std::clamp(vWinding / std::max(1.0, currentTerminalVoltage), 0.0, 1.0);

                double supply = std::min(flywheel->statorCurrentLimitA, duty * statorI);

                totalAmps += supply;

            }

        }



        // 4b. Differential Mechanisms Supply Current Aggregation (2 CAN Motors per Mechanism)

        auto diffEntities = scene.Query<Robot::Mechanisms::Components::DifferentialComponent>();

        for (auto entity : diffEntities)

        {

            auto* diff = scene.GetComponent<Robot::Mechanisms::Components::DifferentialComponent>(entity);

            if (!diff || diff->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                continue;

            }



            double vBus = std::max(1.0, currentTerminalVoltage);



            // Motor A (Left)

            Actuation::DCMotorModel motorA = diff->motorModelA.WithMotorCount(diff->motorCountA);

            double omegaA = diff->motorRotorSpeedsRadS.x;

            double statorIA = std::abs(diff->motorStatorCurrentsA.x);

            double vWindingA = std::min(std::abs(diff->commandedVoltageA),

                (statorIA * motorA.GetResistance()) + std::abs(motorA.GetBackEmfConstantKe() * omegaA));

            double dutyA = std::clamp(vWindingA / vBus, 0.0, 1.0);

            double supplyA = std::min(diff->supplyCurrentLimitA, dutyA * statorIA);

            totalAmps += supplyA;



            // Motor B (Right)

            Actuation::DCMotorModel motorB = diff->motorModelB.WithMotorCount(diff->motorCountB);

            double omegaB = diff->motorRotorSpeedsRadS.y;

            double statorIB = std::abs(diff->motorStatorCurrentsA.y);

            double vWindingB = std::min(std::abs(diff->commandedVoltageB),

                (statorIB * motorB.GetResistance()) + std::abs(motorB.GetBackEmfConstantKe() * omegaB));

            double dutyB = std::clamp(vWindingB / vBus, 0.0, 1.0);

            double supplyB = std::min(diff->supplyCurrentLimitB, dutyB * statorIB);

            totalAmps += supplyB;

        }



        // 4c. Regulated Servo Rail Aggregation (5V / 6V Rail)

        double totalServoAmps = 0.0;

        auto servoEntities = scene.Query<Actuation::Components::ServoComponent>();

        for (auto entity : servoEntities)

        {

            auto* servo = scene.GetComponent<Actuation::Components::ServoComponent>(entity);

            if (servo)

            {

                totalServoAmps += std::abs(servo->currentDraw);

            }

        }



        // Aggregate current from mechanisms with ActuatorType::Servo

        for (auto entity : elevatorEntities)

        {

            auto* elev = scene.GetComponent<Robot::Mechanisms::Components::ElevatorComponent>(entity);

            if (elev && elev->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                totalServoAmps += std::abs(elev->statorCurrentA);

            }

        }

        for (auto entity : armEntities)

        {

            auto* arm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(entity);

            if (arm && arm->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                totalServoAmps += std::abs(arm->statorCurrentA);

            }

        }

        for (auto entity : diffEntities)

        {

            auto* diff = scene.GetComponent<Robot::Mechanisms::Components::DifferentialComponent>(entity);

            if (diff && diff->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                totalServoAmps += std::abs(diff->motorStatorCurrentsA.x) + std::abs(diff->motorStatorCurrentsA.y);

            }

        }



        // Aggregate current from swerve modules using servo steering or drive

        if (swerveSystem)

        {

            auto* model = swerveSystem->GetModel();

            if (model)

            {

                for (uint32_t i = 0; i < 4; ++i)

                {

                    const auto& mod = model->GetModule(i);

                    if (mod.GetSteerParams().actuatorType == Robot::Drivetrain::Model::ModuleActuatorType::Servo)

                    {

                        totalServoAmps += std::abs(mod.GetState().steerStatorCurrent);

                    }

                    if (mod.GetDriveParams().actuatorType == Robot::Drivetrain::Model::ModuleActuatorType::Servo)

                    {

                        totalServoAmps += std::abs(mod.GetState().driveStatorCurrent);

                    }

                }

            }

        }



        currentRegulatedRailCurrent = totalServoAmps;

        if (totalServoAmps > regulatedRailCurrentLimit && regulatedRailCurrentLimit > 0.0)

        {

            // Output voltage droops under overload (thermal foldback / current limiting)

            currentRegulatedRailVoltage = regulatedRailNominalVoltage * (regulatedRailCurrentLimit / totalServoAmps);

            isRegulatedRailBrownedOutState = true;

        }

        else

        {

            currentRegulatedRailVoltage = regulatedRailNominalVoltage;

            isRegulatedRailBrownedOutState = false;

        }



        // Update supply rail voltage on all active servos and mechanisms

        for (auto entity : servoEntities)

        {

            auto* servo = scene.GetComponent<Actuation::Components::ServoComponent>(entity);

            if (servo)

            {

                servo->servoModel.SetSupplyVoltage(currentRegulatedRailVoltage);

            }

        }

        for (auto entity : elevatorEntities)

        {

            auto* elev = scene.GetComponent<Robot::Mechanisms::Components::ElevatorComponent>(entity);

            if (elev && elev->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                elev->servoModel.SetSupplyVoltage(currentRegulatedRailVoltage);

            }

        }

        for (auto entity : armEntities)

        {

            auto* arm = scene.GetComponent<Robot::Mechanisms::Components::ArmComponent>(entity);

            if (arm && arm->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                arm->servoModel.SetSupplyVoltage(currentRegulatedRailVoltage);

            }

        }

        for (auto entity : diffEntities)

        {

            auto* diff = scene.GetComponent<Robot::Mechanisms::Components::DifferentialComponent>(entity);

            if (diff && diff->actuatorType == Robot::Mechanisms::Components::ActuatorType::Servo)

            {

                diff->servoModelA.SetSupplyVoltage(currentRegulatedRailVoltage);

                diff->servoModelB.SetSupplyVoltage(currentRegulatedRailVoltage);

            }

        }

        if (swerveSystem)

        {

            auto* model = swerveSystem->GetModel();

            if (model)

            {

                for (uint32_t i = 0; i < 4; ++i)

                {

                    auto& mod = model->GetModule(i);

                    if (mod.GetSteerParams().actuatorType == Robot::Drivetrain::Model::ModuleActuatorType::Servo)

                    {

                        mod.GetSteerParams().servo.SetSupplyVoltage(currentRegulatedRailVoltage);

                    }

                    if (mod.GetDriveParams().actuatorType == Robot::Drivetrain::Model::ModuleActuatorType::Servo)

                    {

                        mod.GetDriveParams().servo.SetSupplyVoltage(currentRegulatedRailVoltage);

                    }

                }

            }

        }



        // Reflect buck regulator power onto the 12V battery bus: P_rail = V_rail * I_rail, P_12V = P_rail / eta

        double pRail = currentRegulatedRailVoltage * currentRegulatedRailCurrent;

        double p12V = pRail / std::max(0.1, regulatedRailEfficiency);

        double iBusServo = p12V / std::max(1.0, currentTerminalVoltage);

        totalAmps += iBusServo;



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

            externalOutputs->regulatedRailVoltageV = currentRegulatedRailVoltage;

            externalOutputs->regulatedRailCurrentA = currentRegulatedRailCurrent;

            externalOutputs->isRegulatedRailBrownedOut = isRegulatedRailBrownedOutState;

        }

    }

}

