/**
 * @file Units.cpp
 * @brief Implementation of dimensional unit conversion mathematics and string parsing.
 */

#include "Units.h"
#include "Utils/Log.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace FRCBananaSim::Core::Units
{
    namespace
    {
        constexpr double pi = std::numbers::pi;

        std::string ToLower(const std::string& str)
        {
            std::string result = str;
            std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c)
                {
                    return static_cast<char>(std::tolower(c));
                });
            return result;
        }
    }

    // Length
    double Units::Meters(double value) { return value; }
    double Units::Centimeters(double value) { return value * 0.01; }
    double Units::Millimeters(double value) { return value * 0.001; }
    double Units::Inches(double value) { return value * 0.0254; }
    double Units::Feet(double value) { return value * 0.3048; }

    // Mass
    double Units::Kilograms(double value) { return value; }
    double Units::Grams(double value) { return value * 0.001; }
    double Units::Pounds(double value) { return value * 0.45359237; }
    double Units::Ounces(double value) { return value * 0.028349523125; }

    // Angle
    double Units::Radians(double value) { return value; }
    double Units::Degrees(double value) { return value * (pi / 180.0); }
    double Units::Rotations(double value) { return value * (2.0 * pi); }

    // Time
    double Units::Seconds(double value) { return value; }
    double Units::Milliseconds(double value) { return value * 0.001; }
    double Units::Minutes(double value) { return value * 60.0; }

    // Linear Velocity
    double Units::MetersPerSecond(double value) { return value; }
    double Units::FeetPerSecond(double value) { return value * 0.3048; }
    double Units::InchesPerSecond(double value) { return value * 0.0254; }
    double Units::MilesPerHour(double value) { return value * 0.44704; }

    // Angular Velocity
    double Units::RadiansPerSecond(double value) { return value; }
    double Units::DegreesPerSecond(double value) { return value * (pi / 180.0); }
    double Units::RPM(double value) { return value * (2.0 * pi / 60.0); }

    // Force & Torque
    double Units::Newtons(double value) { return value; }
    double Units::PoundForce(double value) { return value * 4.4482216152605; }
    double Units::NewtonMeters(double value) { return value; }
    double Units::FootPounds(double value) { return value * 1.3558179483314; }

    // Electrical
    double Units::Volts(double value) { return value; }
    double Units::Amperes(double value) { return value; }
    double Units::Ohms(double value) { return value; }
    double Units::Watts(double value) { return value; }

    double Units::ToSI(double value, const std::string& unitString)
    {
        std::string unit = ToLower(unitString);

        // Dimensionless / None
        if (unit.empty() || unit == "none" || unit == "scalar" || unit == "ratio" || unit == "count")
        {
            return value;
        }

        // Length
        if (unit == "m" || unit == "meter" || unit == "meters") return Meters(value);
        if (unit == "cm" || unit == "centimeter" || unit == "centimeters") return Centimeters(value);
        if (unit == "mm" || unit == "millimeter" || unit == "millimeters") return Millimeters(value);
        if (unit == "in" || unit == "inch" || unit == "inches") return Inches(value);
        if (unit == "ft" || unit == "foot" || unit == "feet") return Feet(value);

        // Mass
        if (unit == "kg" || unit == "kilogram" || unit == "kilograms") return Kilograms(value);
        if (unit == "g" || unit == "gram" || unit == "grams") return Grams(value);
        if (unit == "lb" || unit == "lbs" || unit == "pound" || unit == "pounds") return Pounds(value);
        if (unit == "oz" || unit == "ounce" || unit == "ounces") return Ounces(value);

        // Angle
        if (unit == "rad" || unit == "radian" || unit == "radians") return Radians(value);
        if (unit == "deg" || unit == "degree" || unit == "degrees") return Degrees(value);
        if (unit == "rot" || unit == "rev" || unit == "rotation" || unit == "revolution" || unit == "rotations") return Rotations(value);

        // Time
        if (unit == "s" || unit == "sec" || unit == "second" || unit == "seconds") return Seconds(value);
        if (unit == "ms" || unit == "millisecond" || unit == "milliseconds") return Milliseconds(value);
        if (unit == "min" || unit == "minute" || unit == "minutes") return Minutes(value);

        // Velocity
        if (unit == "m/s" || unit == "mps") return MetersPerSecond(value);
        if (unit == "ft/s" || unit == "fps") return FeetPerSecond(value);
        if (unit == "in/s" || unit == "ips") return InchesPerSecond(value);
        if (unit == "mph") return MilesPerHour(value);

        // Angular Velocity
        if (unit == "rad/s" || unit == "radps") return RadiansPerSecond(value);
        if (unit == "deg/s" || unit == "degps") return DegreesPerSecond(value);
        if (unit == "rpm") return RPM(value);

        // Force & Torque
        if (unit == "n" || unit == "newton" || unit == "newtons") return Newtons(value);
        if (unit == "lbf") return PoundForce(value);
        if (unit == "n*m" || unit == "nm" || unit == "newton-meter" || unit == "newton_meter") return NewtonMeters(value);
        if (unit == "ft*lb" || unit == "ft*lbf" || unit == "ft-lb" || unit == "ft_lb" || unit == "ft-lbs") return FootPounds(value);

        // Electrical
        if (unit == "v" || unit == "volt" || unit == "volts") return Volts(value);
        if (unit == "a" || unit == "amp" || unit == "amps" || unit == "ampere") return Amperes(value);
        if (unit == "ohm" || unit == "ohms") return Ohms(value);
        if (unit == "w" || unit == "watt" || unit == "watts") return Watts(value);

        SIM_LOG_WARN("Unrecognized unit string '%s'. Returning raw value.", unitString.c_str());
        return value;
    }

    double Units::FromSI(double siValue, const std::string& unitString)
    {
        std::string unit = ToLower(unitString);

        if (unit.empty() || unit == "none" || unit == "scalar" || unit == "ratio") return siValue;

        // Length
        if (unit == "m" || unit == "meter" || unit == "meters") return siValue;
        if (unit == "cm" || unit == "centimeter") return siValue / 0.01;
        if (unit == "mm" || unit == "millimeter") return siValue / 0.001;
        if (unit == "in" || unit == "inch" || unit == "inches") return siValue / 0.0254;
        if (unit == "ft" || unit == "foot" || unit == "feet") return siValue / 0.3048;

        // Mass
        if (unit == "kg" || unit == "kilogram") return siValue;
        if (unit == "g" || unit == "gram") return siValue / 0.001;
        if (unit == "lb" || unit == "lbs" || unit == "pound") return siValue / 0.45359237;
        if (unit == "oz" || unit == "ounce") return siValue / 0.028349523125;

        // Angle
        if (unit == "rad" || unit == "radian") return siValue;
        if (unit == "deg" || unit == "degree") return siValue * (180.0 / pi);
        if (unit == "rot" || unit == "rev") return siValue / (2.0 * pi);

        // Time
        if (unit == "s" || unit == "second") return siValue;
        if (unit == "ms" || unit == "millisecond") return siValue / 0.001;
        if (unit == "min" || unit == "minute") return siValue / 60.0;

        // Velocity
        if (unit == "m/s" || unit == "mps") return siValue;
        if (unit == "ft/s" || unit == "fps") return siValue / 0.3048;
        if (unit == "in/s" || unit == "ips") return siValue / 0.0254;
        if (unit == "mph") return siValue / 0.44704;

        // Angular Velocity
        if (unit == "rad/s") return siValue;
        if (unit == "deg/s") return siValue * (180.0 / pi);
        if (unit == "rpm") return siValue * (60.0 / (2.0 * pi));

        // Force & Torque
        if (unit == "n" || unit == "newton") return siValue;
        if (unit == "lbf") return siValue / 4.4482216152605;
        if (unit == "n*m" || unit == "nm") return siValue;
        if (unit == "ft*lb" || unit == "ft*lbf" || unit == "ft-lbs") return siValue / 1.3558179483314;

        // Electrical
        if (unit == "v") return siValue;
        if (unit == "a") return siValue;
        if (unit == "ohm") return siValue;
        if (unit == "w") return siValue;

        SIM_LOG_WARN("Unrecognized unit string '%s' in FromSI. Returning SI value.", unitString.c_str());
        return siValue;
    }

    double Units::Convert(double value, const std::string& fromUnit, const std::string& toUnit)
    {
        double siValue = ToSI(value, fromUnit);
        return FromSI(siValue, toUnit);
    }

    bool Units::IsValidUnit(const std::string& unitString)
    {
        std::string u = ToLower(unitString);
        return (u.empty() || u == "none" || u == "scalar" || u == "ratio" || u == "count" ||
            u == "m" || u == "cm" || u == "mm" || u == "in" || u == "ft" ||
            u == "kg" || u == "g" || u == "lb" || u == "lbs" || u == "oz" ||
            u == "rad" || u == "deg" || u == "rot" || u == "rev" ||
            u == "s" || u == "sec" || u == "ms" || u == "min" ||
            u == "m/s" || u == "ft/s" || u == "in/s" || u == "mph" ||
            u == "rad/s" || u == "deg/s" || u == "rpm" ||
            u == "n" || u == "lbf" || u == "n*m" || u == "nm" || u == "ft*lb" || u == "ft*lbf" || u == "ft-lbs" ||
            u == "v" || u == "a" || u == "ohm" || u == "w");
    }
}