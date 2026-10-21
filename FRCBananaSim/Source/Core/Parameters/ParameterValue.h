/**
 * @file ParameterValue.h
 * @brief Representation of an individual configuration parameter with unit metadata.
 */

#pragma once

#include <string>

namespace FRCBananaSim::Core::Parameters
{
    /**
     * @struct ParameterValue
     * @brief Holds scalar or string data alongside dimensional unit information.
     */
    struct ParameterValue
    {
        double rawValue{ 0.0 };         ///< Stored numeric value as written in configuration.
        double siValue{ 0.0 };          ///< Converted canonical value in SI base units.
        std::string unit{ "" };         ///< Explicit unit specification (e.g. "kg", "in", "rpm").
        std::string stringValue{ "" };  ///< Raw text value for non-numeric fields.
        bool isNumeric{ false };        ///< True if parameter holds a numeric scalar.
        bool isString{ false };         ///< True if parameter holds a string literal.
        bool isBoolean{ false };        ///< True if parameter holds a boolean value.
        bool boolValue{ false };        ///< Evaluated boolean value.
    };
}