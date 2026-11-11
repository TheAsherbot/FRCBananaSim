#pragma once

#include <imgui/imgui.h>
#include <string>
#include <unordered_map>
#include <cmath>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <cstdio>
#include <cstring>

namespace FRCBananaSim::Tools::Configurator
{
    enum class FieldTargetUnit
    {
        Generic,
        Meters,
        Inches,
        Millimeters,
        Centimeters,
        Feet,
        Kilograms,
        Pounds,
        Radians,
        Degrees,
        NewtonMeters,
        KgCm,
        InchLbs,
        SecondsPer60Deg,
        RPM,
        Amperes,
        Volts,
        Milliseconds,
        Seconds
    };

    /**
     * @brief Detects the native target engineering unit of a field based on its UI label.
     */
    inline FieldTargetUnit DetectTargetUnitFromLabel(const char* label)
    {
        if (!label) return FieldTargetUnit::Generic;
        std::string lbl(label);

        // Strip ## ImGui ID if present
        size_t hashPos = lbl.find("##");
        if (hashPos != std::string::npos)
        {
            lbl = lbl.substr(0, hashPos);
        }

        // Convert to lowercase for matching
        std::string lowerLbl;
        lowerLbl.reserve(lbl.size());
        for (char c : lbl)
        {
            lowerLbl.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }

        // Check unit markers inside parentheses or keywords
        if (lowerLbl.find("(s/60") != std::string::npos || lowerLbl.find("s/60 deg") != std::string::npos)
            return FieldTargetUnit::SecondsPer60Deg;
        if (lowerLbl.find("(rpm)") != std::string::npos)
            return FieldTargetUnit::RPM;
        if (lowerLbl.find("(n*m)") != std::string::npos || lowerLbl.find("(nm)") != std::string::npos)
            return FieldTargetUnit::NewtonMeters;
        if (lowerLbl.find("(kg*cm)") != std::string::npos || lowerLbl.find("(kgcm)") != std::string::npos)
            return FieldTargetUnit::KgCm;
        if (lowerLbl.find("(in-lb)") != std::string::npos || lowerLbl.find("(in*lb)") != std::string::npos)
            return FieldTargetUnit::InchLbs;
        if (lowerLbl.find("(rad)") != std::string::npos || lowerLbl.find("(rad / m)") != std::string::npos)
            return FieldTargetUnit::Radians;
        if (lowerLbl.find("(deg)") != std::string::npos)
            return FieldTargetUnit::Degrees;
        if (lowerLbl.find("(mm)") != std::string::npos)
            return FieldTargetUnit::Millimeters;
        if (lowerLbl.find("(cm)") != std::string::npos)
            return FieldTargetUnit::Centimeters;
        if (lowerLbl.find("(in)") != std::string::npos || lowerLbl.find("(inch)") != std::string::npos)
            return FieldTargetUnit::Inches;
        if (lowerLbl.find("(ft)") != std::string::npos)
            return FieldTargetUnit::Feet;
        if (lowerLbl.find("(m)") != std::string::npos)
            return FieldTargetUnit::Meters;
        if (lowerLbl.find("(kg)") != std::string::npos)
            return FieldTargetUnit::Kilograms;
        if (lowerLbl.find("(lbs)") != std::string::npos || lowerLbl.find("(lb)") != std::string::npos)
            return FieldTargetUnit::Pounds;
        if (lowerLbl.find("(ms)") != std::string::npos)
            return FieldTargetUnit::Milliseconds;
        if (lowerLbl.find("(s)") != std::string::npos)
            return FieldTargetUnit::Seconds;
        if (lowerLbl.find("(a)") != std::string::npos)
            return FieldTargetUnit::Amperes;
        if (lowerLbl.find("(v)") != std::string::npos)
            return FieldTargetUnit::Volts;

        return FieldTargetUnit::Generic;
    }

    /**
     * @brief Parses a user-typed string with optional unit suffix (e.g. 24", 600mm, 2.5ft, 120lbs, 90deg)
     *        and converts it automatically to the field's target unit.
     */
    inline bool ParseUnitExpression(const std::string& inputStr, const char* fieldLabel, float& outVal)
    {
        // 1. Trim input
        size_t first = inputStr.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return false;
        size_t last = inputStr.find_last_not_of(" \t\r\n");
        std::string str = inputStr.substr(first, last - first + 1);

        // 2. Identify start of unit suffix
        size_t unitStart = std::string::npos;
        for (size_t i = 0; i < str.size(); ++i)
        {
            unsigned char c = static_cast<unsigned char>(str[i]);
            // Check for exponent in scientific notation: e.g. 1e-3, 2.5E+4
            if ((c == 'e' || c == 'E') && i > 0 && std::isdigit(static_cast<unsigned char>(str[i - 1])))
            {
                if (i + 1 < str.size())
                {
                    char next = str[i + 1];
                    if (std::isdigit(static_cast<unsigned char>(next)) || next == '+' || next == '-')
                    {
                        continue; // part of number
                    }
                }
            }

            // Unit characters: letters, quotes, hash, or non-ascii (like degree symbol)
            if (std::isalpha(c) || c == '"' || c == '\'' || c == '#' || c >= 128)
            {
                unitStart = i;
                break;
            }
        }

        std::string exprPart = (unitStart == std::string::npos) ? str : str.substr(0, unitStart);
        std::string unitPart = (unitStart == std::string::npos) ? "" : str.substr(unitStart);

        // Trim exprPart
        size_t eFirst = exprPart.find_first_not_of(" \t\r\n");
        size_t eLast = exprPart.find_last_not_of(" \t\r\n");
        if (eFirst == std::string::npos)
        {
            return false;
        }
        exprPart = exprPart.substr(eFirst, eLast - eFirst + 1);

        // Trim and lowercase unitPart
        size_t uFirst = unitPart.find_first_not_of(" \t\r\n");
        if (uFirst != std::string::npos)
        {
            size_t uLast = unitPart.find_last_not_of(" \t\r\n");
            unitPart = unitPart.substr(uFirst, uLast - uFirst + 1);
        }
        else
        {
            unitPart.clear();
        }

        std::string lowerUnit;
        lowerUnit.reserve(unitPart.size());
        for (char uc : unitPart)
        {
            lowerUnit.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(uc))));
        }

        // 3. Evaluate numeric expression in exprPart
        double rawVal = 0.0;
        try
        {
            size_t slashPos = exprPart.find('/');
            if (slashPos != std::string::npos)
            {
                std::string left = exprPart.substr(0, slashPos);
                std::string right = exprPart.substr(slashPos + 1);
                double num = std::stod(left);
                double den = std::stod(right);
                if (std::abs(den) < 1e-9) return false;
                rawVal = num / den;
            }
            else
            {
                size_t starPos = exprPart.find('*');
                if (starPos != std::string::npos)
                {
                    std::string left = exprPart.substr(0, starPos);
                    std::string right = exprPart.substr(starPos + 1);
                    rawVal = std::stod(left) * std::stod(right);
                }
                else
                {
                    rawVal = std::stod(exprPart);
                }
            }
        }
        catch (...)
        {
            return false;
        }

        // 4. Unit Conversion
        FieldTargetUnit target = DetectTargetUnitFromLabel(fieldLabel);

        if (lowerUnit.empty())
        {
            outVal = static_cast<float>(rawVal);
            return true;
        }

        bool isInch = (lowerUnit == "\"" || lowerUnit == "in" || lowerUnit == "inch" || lowerUnit == "inches");
        bool isFoot = (lowerUnit == "'" || lowerUnit == "ft" || lowerUnit == "feet" || lowerUnit == "foot");
        bool isYard = (lowerUnit == "yd" || lowerUnit == "yard" || lowerUnit == "yards");
        bool isMm   = (lowerUnit == "mm" || lowerUnit == "millimeter" || lowerUnit == "millimeters");
        bool isCm   = (lowerUnit == "cm" || lowerUnit == "centimeter" || lowerUnit == "centimeters");
        bool isM    = (lowerUnit == "m" || lowerUnit == "meter" || lowerUnit == "meters");

        bool isPound = (lowerUnit == "#" || lowerUnit == "lb" || lowerUnit == "lbs" || lowerUnit == "pound" || lowerUnit == "pounds");
        bool isOunce = (lowerUnit == "oz" || lowerUnit == "ounce" || lowerUnit == "ounces");
        bool isGram  = (lowerUnit == "g" || lowerUnit == "gram" || lowerUnit == "grams");
        bool isKg    = (lowerUnit == "kg" || lowerUnit == "kilogram" || lowerUnit == "kilograms");

        bool isDeg  = (lowerUnit == "deg" || lowerUnit == "degree" || lowerUnit == "degrees" || unitPart == "°" || unitPart == "\xc2\xb0");
        bool isRad  = (lowerUnit == "rad" || lowerUnit == "radian" || lowerUnit == "radians");
        bool isRev  = (lowerUnit == "rev" || lowerUnit == "rot" || lowerUnit == "revolution" || lowerUnit == "revolutions");

        bool isRpm  = (lowerUnit == "rpm");
        bool isS60  = (lowerUnit == "s/60" || lowerUnit == "s/60deg" || lowerUnit == "s/60 deg" || lowerUnit == "sec/60");

        bool isNm   = (lowerUnit == "nm" || lowerUnit == "n*m" || lowerUnit == "n-m");
        bool isKgcm = (lowerUnit == "kgcm" || lowerUnit == "kg*cm" || lowerUnit == "kg-cm");
        bool isInlb = (lowerUnit == "inlb" || lowerUnit == "in*lb" || lowerUnit == "in-lb" || lowerUnit == "in*lbs" || lowerUnit == "in-lbs");
        bool isFtlb = (lowerUnit == "ftlb" || lowerUnit == "ft*lb" || lowerUnit == "ft-lb" || lowerUnit == "ft*lbs" || lowerUnit == "ft-lbs");

        bool isMs   = (lowerUnit == "ms" || lowerUnit == "msec" || lowerUnit == "millisecond" || lowerUnit == "milliseconds");
        bool isS    = (lowerUnit == "s" || lowerUnit == "sec" || lowerUnit == "second" || lowerUnit == "seconds");

        bool isA    = (lowerUnit == "a" || lowerUnit == "amp" || lowerUnit == "amps" || lowerUnit == "ampere" || lowerUnit == "amperes");
        bool isMa   = (lowerUnit == "ma" || lowerUnit == "milliamp" || lowerUnit == "milliamps");

        bool isV    = (lowerUnit == "v" || lowerUnit == "volt" || lowerUnit == "volts");
        bool isMv   = (lowerUnit == "mv" || lowerUnit == "millivolt" || lowerUnit == "millivolts");

        constexpr double kPi = 3.14159265358979323846;
        double converted = rawVal;

        switch (target)
        {
        case FieldTargetUnit::Meters:
            if (isInch) converted = rawVal * 0.0254;
            else if (isFoot) converted = rawVal * 0.3048;
            else if (isYard) converted = rawVal * 0.9144;
            else if (isMm) converted = rawVal * 0.001;
            else if (isCm) converted = rawVal * 0.01;
            else if (isM) converted = rawVal;
            break;

        case FieldTargetUnit::Inches:
            if (isInch) converted = rawVal;
            else if (isFoot) converted = rawVal * 12.0;
            else if (isMm) converted = rawVal / 25.4;
            else if (isCm) converted = rawVal / 2.54;
            else if (isM) converted = rawVal / 0.0254;
            break;

        case FieldTargetUnit::Millimeters:
            if (isMm) converted = rawVal;
            else if (isInch) converted = rawVal * 25.4;
            else if (isFoot) converted = rawVal * 304.8;
            else if (isCm) converted = rawVal * 10.0;
            else if (isM) converted = rawVal * 1000.0;
            break;

        case FieldTargetUnit::Centimeters:
            if (isCm) converted = rawVal;
            else if (isMm) converted = rawVal * 0.1;
            else if (isInch) converted = rawVal * 2.54;
            else if (isM) converted = rawVal * 100.0;
            break;

        case FieldTargetUnit::Feet:
            if (isFoot) converted = rawVal;
            else if (isInch) converted = rawVal / 12.0;
            else if (isM) converted = rawVal / 0.3048;
            break;

        case FieldTargetUnit::Kilograms:
            if (isPound) converted = rawVal * 0.45359237;
            else if (isOunce) converted = rawVal * 0.02834952;
            else if (isGram) converted = rawVal * 0.001;
            else if (isKg) converted = rawVal;
            break;

        case FieldTargetUnit::Pounds:
            if (isPound) converted = rawVal;
            else if (isKg) converted = rawVal / 0.45359237;
            else if (isGram) converted = rawVal / 453.59237;
            else if (isOunce) converted = rawVal / 16.0;
            break;

        case FieldTargetUnit::Radians:
            if (isDeg) converted = rawVal * (kPi / 180.0);
            else if (isRev) converted = rawVal * (2.0 * kPi);
            else if (isRad) converted = rawVal;
            else if (isInch) converted = rawVal * 0.0254;
            else if (isMm) converted = rawVal * 0.001;
            break;

        case FieldTargetUnit::Degrees:
            if (isRad) converted = rawVal * (180.0 / kPi);
            else if (isRev) converted = rawVal * 360.0;
            else if (isDeg) converted = rawVal;
            break;

        case FieldTargetUnit::NewtonMeters:
            if (isKgcm) converted = rawVal * 0.0980665;
            else if (isInlb) converted = rawVal * 0.112984829;
            else if (isFtlb) converted = rawVal * 1.355817948;
            else if (isNm) converted = rawVal;
            break;

        case FieldTargetUnit::KgCm:
            if (isNm) converted = rawVal / 0.0980665;
            else if (isInlb) converted = rawVal * 1.15212462;
            else if (isKgcm) converted = rawVal;
            break;

        case FieldTargetUnit::SecondsPer60Deg:
            if (isRpm && rawVal > 1e-4) converted = 10.0 / rawVal;
            else if (isS60) converted = rawVal;
            break;

        case FieldTargetUnit::RPM:
            if (isS60 && rawVal > 1e-4) converted = 10.0 / rawVal;
            else if (isRad) converted = rawVal * (60.0 / (2.0 * kPi));
            else if (isRpm) converted = rawVal;
            break;

        case FieldTargetUnit::Milliseconds:
            if (isS) converted = rawVal * 1000.0;
            else if (isMs) converted = rawVal;
            break;

        case FieldTargetUnit::Seconds:
            if (isMs) converted = rawVal * 0.001;
            else if (isS) converted = rawVal;
            break;

        case FieldTargetUnit::Amperes:
            if (isMa) converted = rawVal * 0.001;
            else if (isA) converted = rawVal;
            break;

        case FieldTargetUnit::Volts:
            if (isMv) converted = rawVal * 0.001;
            else if (isV) converted = rawVal;
            break;

        default:
            converted = rawVal;
            break;
        }

        outVal = static_cast<float>(converted);
        return true;
    }

    struct DirectInputState
    {
        char buffer[64] = { 0 };
        bool isEditing = false;
        bool focusRequested = false;
    };

    /**
     * @brief Unified single-control slider widget with direct keyboard input and automatic unit conversion.
     *        - Single compact widget: no redundant side-by-side duplicate numbers.
     *        - Click & drag with mouse to scrub/slide the value.
     *        - Double-click, Ctrl+Click, or Right-click to edit directly in-place.
     *        - Typing units (e.g. 24", 600mm, 2.5ft, 120lbs, 90deg, 1/9) automatically parses and converts.
     */
    inline bool SliderWithDirectInput(const char* label, float* val, float vMin, float vMax,
        const char* format = "%.2f", float sliderWidth = 95.0f, float unusedInputWidth = 0.0f)
    {
        static std::unordered_map<ImGuiID, DirectInputState> s_inputStates;
        bool changed = false;

        ImGui::PushID(label);
        ImGuiID widgetId = ImGui::GetID("##UnifiedField");
        auto& state = s_inputStates[widgetId];

        ImGui::BeginGroup();

        if (state.isEditing)
        {
            ImGui::SetNextItemWidth(sliderWidth);
            if (state.focusRequested)
            {
                ImGui::SetKeyboardFocusHere();
                state.focusRequested = false;
            }

            bool enterPressed = ImGui::InputText("##Edit", state.buffer, sizeof(state.buffer),
                ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);

            if (enterPressed || ImGui::IsItemDeactivatedAfterEdit())
            {
                state.isEditing = false;
                float parsedVal = *val;
                if (ParseUnitExpression(state.buffer, label, parsedVal))
                {
                    if (vMin < vMax)
                    {
                        parsedVal = std::clamp(parsedVal, vMin, vMax);
                    }
                    *val = parsedVal;
                    changed = true;
                }
            }
            else if (ImGui::IsItemDeactivated() || (ImGui::IsItemActive() && ImGui::IsKeyPressed(ImGuiKey_Escape)))
            {
                state.isEditing = false;
            }
        }
        else
        {
            ImGui::SetNextItemWidth(sliderWidth);
            if (ImGui::SliderFloat("##Slider", val, vMin, vMax, format, ImGuiSliderFlags_NoInput))
            {
                changed = true;
            }

            bool doubleClicked = ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0);
            bool ctrlClicked = (ImGui::IsItemActive() || ImGui::IsItemClicked(0)) && ImGui::GetIO().KeyCtrl;
            bool rightClicked = ImGui::IsItemHovered() && ImGui::IsMouseClicked(1);

            if (doubleClicked || ctrlClicked || rightClicked)
            {
                state.isEditing = true;
                state.focusRequested = true;
                snprintf(state.buffer, sizeof(state.buffer), format, *val);
            }
        }

        // 2. Label Text & Tooltip
        ImGui::SameLine(0.0f, 8.0f);
        const char* hashPos = std::strstr(label, "##");
        if (hashPos)
        {
            ImGui::TextUnformatted(label, hashPos);
        }
        else
        {
            ImGui::TextUnformatted(label);
        }

        ImGui::EndGroup();

        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Tip: Drag to slide, or Double-click / Ctrl+Click / Right-click to type directly with units\n(e.g. 24\", 600mm, 2.5ft, 120lbs, 90deg, 1/9)");
        }

        ImGui::PopID();
        return changed;
    }

    /**
     * @brief Renders a rich tooltip explaining motor inversion conventions for CTRE vs REV.
     */
    inline void ShowMotorInvertTooltip(const std::string& vendor, bool inverted)
    {
        ImGui::BeginTooltip();
        ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Motor Inversion Direction:");
        ImGui::Separator();
        ImGui::Text("State: %s", inverted ? "INVERTED (True)" : "NORMAL (False)");
        ImGui::Spacing();
        if (vendor.find("CTRE") != std::string::npos || vendor.find("Talon") != std::string::npos ||
            vendor.find("Kraken") != std::string::npos || vendor.find("Falcon") != std::string::npos)
        {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "CTRE (Kraken X60, Falcon 500, Talon FX):");
            ImGui::BulletText("Viewpoint: Looking directly at motor face (shaft tip pointing toward you)");
            ImGui::BulletText("Normal (Inv = false): Counter-Clockwise (CCW) is positive");
            ImGui::BulletText("Inverted (Inv = true): Clockwise (CW) is positive");
        }
        else if (vendor.find("REV") != std::string::npos || vendor.find("Spark") != std::string::npos ||
                 vendor.find("SPARK") != std::string::npos || vendor.find("NEO") != std::string::npos)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "REV (NEO, NEO Vortex, SPARK MAX, SPARK Flex):");
            ImGui::BulletText("Viewpoint: Looking from rear of motor (shaft tip farthest from you)");
            ImGui::BulletText("Normal (Inv = false): CCW looking from back (= CW looking at shaft face)");
            ImGui::BulletText("Inverted (Inv = true): CW looking from back (= CCW looking at shaft face)");
        }
        else
        {
            ImGui::BulletText("CTRE Kraken/Falcon: Normal = CCW looking at shaft tip");
            ImGui::BulletText("REV NEO/Vortex: Normal = CCW looking from rear (CW at shaft tip)");
        }
        ImGui::Separator();
        ImGui::TextDisabled("Note: CTRE and REV define positive shaft rotation opposite to each other!");
        ImGui::EndTooltip();
    }

    /**
     * @brief Renders the general motor inversion reference tooltip for headers and help markers.
     */
    inline void ShowMotorInvertHelpMarker()
    {
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Motor Inversion Direction Reference:");
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "CTRE (Kraken X60, Falcon 500, Talon FX):");
            ImGui::BulletText("Viewpoint: Looking directly at motor face (shaft tip closer to you)");
            ImGui::BulletText("Normal (Inv = false): Counter-Clockwise (CCW) is positive");
            ImGui::BulletText("Inverted (Inv = true): Clockwise (CW) is positive");
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "REV (NEO, NEO Vortex, SPARK MAX, SPARK Flex):");
            ImGui::BulletText("Viewpoint: Looking from rear of motor (shaft tip farthest from you)");
            ImGui::BulletText("Normal (Inv = false): CCW from rear (= CW looking at shaft face)");
            ImGui::BulletText("Inverted (Inv = true): CW from rear (= CCW looking at shaft face)");
            ImGui::Separator();
            ImGui::TextDisabled("Note: CTRE and REV define positive shaft rotation opposite to each other when viewing the shaft face.");
            ImGui::EndTooltip();
        }
    }
}
