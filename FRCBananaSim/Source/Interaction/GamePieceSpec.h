/**
 * @file GamePieceSpec.h
 * @brief Parametric physical, compliant, and aerodynamic specification for game pieces.
 */

#pragma once

#include <string>

namespace FRCBananaSim::Interaction
{
    /**
     * @struct GamePieceSpec
     * @brief Physical properties, material compliance, and aerodynamic constants for simulated game pieces.
     */
    struct GamePieceSpec
    {
        std::string name = "Note2024";
        double massKg = 0.235;
        double outerRadiusM = 0.1778;        ///< Outer radius (m) - 14 inches diameter
        double innerRadiusM = 0.1270;        ///< Inner radius (m) - 10 inches hole
        double thicknessM = 0.0508;          ///< Height / thickness (m) - 2 inches
        double momentOfInertiaKgm2 = 0.0035; ///< Principal spin axis inertia (kg*m^2)
        double dragCoefficientCd = 0.45;     ///< Quadratic air drag coefficient
        double crossSectionalAreaM2 = 0.045; ///< Frontal / planform drag reference area (m^2)
        double liftSlopeClAlpha = 1.20;      ///< Lift curve slope for Magnus rotational lift
        double complianceK = 25000.0;        ///< Hunt-Crossley contact stiffness (N/m^1.5)
        double restitutionE = 0.25;          ///< Coefficient of restitution
        double surfaceFriction = 0.68;       ///< Foam-on-tread Coulomb friction coefficient
    };
}
