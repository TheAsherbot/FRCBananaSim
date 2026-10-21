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