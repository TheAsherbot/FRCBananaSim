/**
 * @file RobotManifest.h
 * @brief Formal data structures for Robot Data Schema v1 manifest.json (Physical CAD hierarchy).
 */

#pragma once

#include <string>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace FRCBananaSim::Robot::Schema
{
    /**
     * @struct MassSummary
     * @brief Overall robot mass and center-of-mass summary in WPILib NWU frame.
     */
    struct MassSummary
    {
        double totalMassKg = 0.0;
        glm::dvec3 centerOfMassM = { 0.0, 0.0, 0.0 };
    };

    /**
     * @struct InertiaTensor
     * @brief 3x3 symmetric rotational mass moments of inertia in link body frame (kg*m^2).
     */
    struct InertiaTensor
    {
        double ixx = 0.0;
        double iyy = 0.0;
        double izz = 0.0;
        double ixy = 0.0;
        double ixz = 0.0;
        double iyz = 0.0;

        /**
         * @brief Converts the inertia components into a standard symmetric 3x3 matrix.
         * @return 3x3 inertia matrix.
         */
        glm::dmat3 ToMatrix() const;

        /**
         * @brief Verifies whether the inertia tensor is strictly positive-definite via Sylvester's criterion.
         * @return True if all leading principal minors are strictly positive (> 1e-9).
         */
        bool IsPositiveDefinite() const;
    };

    /**
     * @struct ColliderDefinition
     * @brief Collision proxy geometry definition attached to a rigid link.
     */
    struct ColliderDefinition
    {
        std::string name;
        std::string type; ///< "box", "cylinder", "sphere", "capsule", "convex_hull", "mesh"
        glm::dvec3 originM = { 0.0, 0.0, 0.0 };
        glm::dquat rotationQuat = { 1.0, 0.0, 0.0, 0.0 };
        glm::dvec3 dimensionsM = { 0.0, 0.0, 0.0 }; ///< Box (dx, dy, dz), Cylinder/Sphere (r, h, 0)
        std::string material;
    };

    /**
     * @struct LinkDefinition
     * @brief Physical link in the robot kinematic tree.
     */
    struct LinkDefinition
    {
        std::string name;
        bool isRoot = false;
        double massKg = 0.0;
        glm::dvec3 centerOfMassM = { 0.0, 0.0, 0.0 };
        InertiaTensor inertiaTensor;
        std::string material;
        std::string visualMesh;
        std::vector<ColliderDefinition> colliders;
    };

    /**
     * @struct JointLimits
     * @brief Hard stop limits and compliance spring parameters for a joint.
     */
    struct JointLimits
    {
        bool hasLimits = false;
        double lower = 0.0;
        double upper = 0.0;
        std::string unit = "rad"; ///< "rad", "deg", "m"
        double springFrequencyHz = 40.0;
        double springDampingRatio = 1.0;
    };

    /**
     * @struct JointDefinition
     * @brief Kinematic joint connecting two links.
     */
    struct JointDefinition
    {
        std::string name;
        std::string type; ///< "prismatic", "revolute", "fixed"
        std::string parentLink;
        std::string childLink;
        glm::dvec3 originM = { 0.0, 0.0, 0.0 };
        glm::dvec3 axis = { 0.0, 0.0, 1.0 };
        JointLimits limits;
    };

    /**
     * @struct ContactZoneDefinition
     * @brief Contact engagement volume for intake capture or shooter compression.
     */
    struct ContactZoneDefinition
    {
        std::string name;
        std::string role; ///< "intake", "shooter", "indexer"
        std::string parentLink;
        std::string shape = "box";
        glm::dvec3 originM = { 0.0, 0.0, 0.0 };
        glm::dquat rotationQuat = { 1.0, 0.0, 0.0, 0.0 };
        glm::dvec3 dimensionsM = { 0.0, 0.0, 0.0 };
        glm::dvec3 tangentAxis = { 1.0, 0.0, 0.0 };
        glm::dvec3 compressionAxis = { 0.0, 0.0, 1.0 };
        double nominalGapM = 0.0;
    };

    /**
     * @struct RobotManifest
     * @brief Complete parsed manifest.json physical specification.
     */
    struct RobotManifest
    {
        std::string schemaVersion = "1.0.0";
        std::string robotName;
        std::string units = "SI";
        std::string coordinateSystem = "NWU";
        MassSummary massSummary;
        std::vector<LinkDefinition> links;
        std::vector<JointDefinition> joints;
        std::vector<ContactZoneDefinition> contactZones;

        /**
         * @brief Looks up a link definition by name.
         * @param linkName Unique link identifier.
         * @return Pointer to link definition, or nullptr if not found.
         */
        const LinkDefinition* FindLink(const std::string& linkName) const;

        /**
         * @brief Looks up a joint definition by name.
         * @param jointName Unique joint identifier.
         * @return Pointer to joint definition, or nullptr if not found.
         */
        const JointDefinition* FindJoint(const std::string& jointName) const;

        /**
         * @brief Looks up a contact zone by name.
         * @param zoneName Unique contact zone identifier.
         * @return Pointer to contact zone definition, or nullptr if not found.
         */
        const ContactZoneDefinition* FindContactZone(const std::string& zoneName) const;

        /**
         * @brief Returns the designated root link (base_link).
         * @return Pointer to the root link definition, or nullptr if none marked isRoot.
         */
        const LinkDefinition* GetRootLink() const;
    };
}