/**
 * @file Units.h
 * @brief Strongly typed physical unit conversions to canonical SI base units.
 */

#pragma once

#include <string>

namespace FRCBananaSim::Core::Units
{
    /**
     * @class Units
     * @brief Conversion functions mapping real-world engineering units into SI standards.
     *
     * Canonical SI Base Units:
     * - Length: Meters (m)
     * - Mass: Kilograms (kg)
     * - Angle: Radians (rad)
     * - Time: Seconds (s)
     * - Velocity: Meters per second (m/s)
     * - Angular Velocity: Radians per second (rad/s)
     * - Force: Newtons (N)
     * - Torque: Newton-meters (N*m)
     * - Electrical: Volts (V), Amperes (A), Ohms (ohm), Watts (W)
     */
    class Units
    {
    public:
        // Length conversions -> Meters
        static double Meters(double value);
        static double Centimeters(double value);
        static double Millimeters(double value);
        static double Inches(double value);
        static double Feet(double value);

        // Mass conversions -> Kilograms
        static double Kilograms(double value);
        static double Grams(double value);
        static double Pounds(double value);
        static double Ounces(double value);

        // Angle conversions -> Radians
        static double Radians(double value);
        static double Degrees(double value);
        static double Rotations(double value);

        // Time conversions -> Seconds
        static double Seconds(double value);
        static double Milliseconds(double value);
        static double Minutes(double value);

        // Linear Velocity -> Meters per second
        static double MetersPerSecond(double value);
        static double FeetPerSecond(double value);
        static double InchesPerSecond(double value);
        static double MilesPerHour(double value);

        // Angular Velocity -> Radians per second
        static double RadiansPerSecond(double value);
        static double DegreesPerSecond(double value);
        static double RPM(double value);

        // Force and Torque -> Newtons / Newton-meters
        static double Newtons(double value);
        static double PoundForce(double value);
        static double NewtonMeters(double value);
        static double FootPounds(double value);

        // Electrical -> Volts, Amperes, Ohms, Watts
        static double Volts(double value);
        static double Amperes(double value);
        static double Ohms(double value);
        static double Watts(double value);

        /**
         * @brief Converts a quantity with a given unit string into its SI base value.
         * @param value The scalar magnitude.
         * @param unitString Case-insensitive unit identifier (e.g., "in", "lbs", "rpm", "deg").
         * @return Equivalent magnitude in SI base units.
         */
        static double ToSI(double value, const std::string& unitString);

        /**
         * @brief Converts an SI base value into a specified target unit.
         * @param siValue Value in SI base units.
         * @param unitString Target unit identifier (e.g., "in", "deg", "rpm").
         * @return Value converted into the requested unit.
         */
        static double FromSI(double siValue, const std::string& unitString);

        /**
         * @brief Converts directly between two compatible units.
         * @param value Scalar magnitude in fromUnit.
         * @param fromUnit Source unit string.
         * @param toUnit Destination unit string.
         * @return Converted scalar magnitude.
         */
        static double Convert(double value, const std::string& fromUnit, const std::string& toUnit);

        /**
         * @brief Checks if a given unit string is recognized by the conversion engine.
         * @param unitString Unit string to validate.
         * @return True if supported, false otherwise.
         */
        static bool IsValidUnit(const std::string& unitString);
    };
}

namespace FRCBananaSim::Core::Units::Literals
{
    // Length
    constexpr double operator""_m(long double v) { return static_cast<double>(v); }
    constexpr double operator""_m(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_cm(long double v) { return static_cast<double>(v) * 0.01; }
    constexpr double operator""_cm(unsigned long long v) { return static_cast<double>(v) * 0.01; }
    constexpr double operator""_mm(long double v) { return static_cast<double>(v) * 0.001; }
    constexpr double operator""_mm(unsigned long long v) { return static_cast<double>(v) * 0.001; }
    constexpr double operator""_in(long double v) { return static_cast<double>(v) * 0.0254; }
    constexpr double operator""_in(unsigned long long v) { return static_cast<double>(v) * 0.0254; }
    constexpr double operator""_ft(long double v) { return static_cast<double>(v) * 0.3048; }
    constexpr double operator""_ft(unsigned long long v) { return static_cast<double>(v) * 0.3048; }

    // Mass
    constexpr double operator""_kg(long double v) { return static_cast<double>(v); }
    constexpr double operator""_kg(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_g(long double v) { return static_cast<double>(v) * 0.001; }
    constexpr double operator""_g(unsigned long long v) { return static_cast<double>(v) * 0.001; }
    constexpr double operator""_lbs(long double v) { return static_cast<double>(v) * 0.45359237; }
    constexpr double operator""_lbs(unsigned long long v) { return static_cast<double>(v) * 0.45359237; }
    constexpr double operator""_lb(long double v) { return static_cast<double>(v) * 0.45359237; }
    constexpr double operator""_lb(unsigned long long v) { return static_cast<double>(v) * 0.45359237; }
    constexpr double operator""_oz(long double v) { return static_cast<double>(v) * 0.028349523125; }
    constexpr double operator""_oz(unsigned long long v) { return static_cast<double>(v) * 0.028349523125; }

    // Angle
    constexpr double operator""_rad(long double v) { return static_cast<double>(v); }
    constexpr double operator""_rad(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_deg(long double v) { return static_cast<double>(v) * (3.14159265358979323846 / 180.0); }
    constexpr double operator""_deg(unsigned long long v) { return static_cast<double>(v) * (3.14159265358979323846 / 180.0); }
    constexpr double operator""_rot(long double v) { return static_cast<double>(v) * (2.0 * 3.14159265358979323846); }
    constexpr double operator""_rot(unsigned long long v) { return static_cast<double>(v) * (2.0 * 3.14159265358979323846); }

    // Time
    constexpr double operator""_s(long double v) { return static_cast<double>(v); }
    constexpr double operator""_s(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_ms(long double v) { return static_cast<double>(v) * 0.001; }
    constexpr double operator""_ms(unsigned long long v) { return static_cast<double>(v) * 0.001; }
    constexpr double operator""_min(long double v) { return static_cast<double>(v) * 60.0; }
    constexpr double operator""_min(unsigned long long v) { return static_cast<double>(v) * 60.0; }

    // Velocity
    constexpr double operator""_mps(long double v) { return static_cast<double>(v); }
    constexpr double operator""_mps(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_fps(long double v) { return static_cast<double>(v) * 0.3048; }
    constexpr double operator""_fps(unsigned long long v) { return static_cast<double>(v) * 0.3048; }
    constexpr double operator""_ips(long double v) { return static_cast<double>(v) * 0.0254; }
    constexpr double operator""_ips(unsigned long long v) { return static_cast<double>(v) * 0.0254; }
    constexpr double operator""_mph(long double v) { return static_cast<double>(v) * 0.44704; }
    constexpr double operator""_mph(unsigned long long v) { return static_cast<double>(v) * 0.44704; }

    // Angular Velocity
    constexpr double operator""_radps(long double v) { return static_cast<double>(v); }
    constexpr double operator""_radps(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_degps(long double v) { return static_cast<double>(v) * (3.14159265358979323846 / 180.0); }
    constexpr double operator""_degps(unsigned long long v) { return static_cast<double>(v) * (3.14159265358979323846 / 180.0); }
    constexpr double operator""_rpm(long double v) { return static_cast<double>(v) * (2.0 * 3.14159265358979323846 / 60.0); }
    constexpr double operator""_rpm(unsigned long long v) { return static_cast<double>(v) * (2.0 * 3.14159265358979323846 / 60.0); }
    constexpr double operator""_sp60deg(long double v) { return (static_cast<double>(v) > 1e-6) ? ((3.14159265358979323846 / 3.0) / static_cast<double>(v)) : 0.0; }
    constexpr double operator""_sp60deg(unsigned long long v) { return (static_cast<double>(v) > 1e-6) ? ((3.14159265358979323846 / 3.0) / static_cast<double>(v)) : 0.0; }

    // Force & Torque
    constexpr double operator""_N(long double v) { return static_cast<double>(v); }
    constexpr double operator""_N(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_lbf(long double v) { return static_cast<double>(v) * 4.4482216152605; }
    constexpr double operator""_lbf(unsigned long long v) { return static_cast<double>(v) * 4.4482216152605; }
    constexpr double operator""_Nm(long double v) { return static_cast<double>(v); }
    constexpr double operator""_Nm(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_ftlb(long double v) { return static_cast<double>(v) * 1.3558179483314; }
    constexpr double operator""_ftlb(unsigned long long v) { return static_cast<double>(v) * 1.3558179483314; }
    constexpr double operator""_ftlbs(long double v) { return static_cast<double>(v) * 1.3558179483314; }
    constexpr double operator""_ftlbs(unsigned long long v) { return static_cast<double>(v) * 1.3558179483314; }
    constexpr double operator""_ozin(long double v) { return static_cast<double>(v) * 0.00706155183333; }
    constexpr double operator""_ozin(unsigned long long v) { return static_cast<double>(v) * 0.00706155183333; }
    constexpr double operator""_kgm(long double v) { return static_cast<double>(v) * 9.80665; }
    constexpr double operator""_kgm(unsigned long long v) { return static_cast<double>(v) * 9.80665; }
    constexpr double operator""_kgcm(long double v) { return static_cast<double>(v) * 0.0980665; }
    constexpr double operator""_kgcm(unsigned long long v) { return static_cast<double>(v) * 0.0980665; }

    // Electrical
    constexpr double operator""_V(long double v) { return static_cast<double>(v); }
    constexpr double operator""_V(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_A(long double v) { return static_cast<double>(v); }
    constexpr double operator""_A(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_mA(long double v) { return static_cast<double>(v) / 1000.0; }
    constexpr double operator""_mA(unsigned long long v) { return static_cast<double>(v) / 1000.0; }
    constexpr double operator""_ohm(long double v) { return static_cast<double>(v); }
    constexpr double operator""_ohm(unsigned long long v) { return static_cast<double>(v); }
    constexpr double operator""_W(long double v) { return static_cast<double>(v); }
    constexpr double operator""_W(unsigned long long v) { return static_cast<double>(v); }
}
