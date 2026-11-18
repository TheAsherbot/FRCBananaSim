/**
 * @file EmbeddedRobotData.h
 * @brief Self-contained fallback Robot Comp 2 definitions embedded directly into the executable binary.
 * Ensures the simulator launches cleanly anywhere without requiring external resource files on disk.
 */

#pragma once

#include <string_view>

namespace FRCBananaSim::Robot::Schema::Embedded
{
    inline constexpr std::string_view kManifestJson = R"ROBOT_JSON(
{
  "schemaVersion": "1.0.0",
  "robotName": "Comp 2 Robot EDITED",
  "units": "SI",
  "coordinateSystem": "NWU",
  "massSummary": {
    "totalMass_kg": 66.65601,
    "centerOfMass_m": [
      -0.065598,
      0.013779,
      0.162341
    ]
  },
  "links": [
    {
      "name": "base_link",
      "isRoot": true,
      "mass_kg": 48.0,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.06
      ],
      "inertiaTensor_kgm2": {
        "ixx": 2.987273047,
        "iyy": 5.263786983,
        "izz": 7.418807541,
        "ixy": 0.359681545,
        "ixz": -0.047834484,
        "iyz": 0.087315609
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/base_link.glb",
      "colliders": [
        {
          "name": "base_link_chassis_box",
          "type": "box",
          "origin_m": [
            0.0,
            0.0,
            0.06
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.74,
            0.74,
            0.12
          ],
          "material": "aluminum_6061"
        }
      ]
    },
    {
      "name": "gear",
      "isRoot": false,
      "mass_kg": 0.5,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.02
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.003168028,
        "iyy": 0.002639374,
        "izz": 0.005684991,
        "ixy": 8.09e-07,
        "ixz": -6.2564e-05,
        "iyz": -9.9e-08
      },
      "material": "pla_solid",
      "visualMesh": "Meshes/gear.glb",
      "colliders": [
        {
          "name": "gear_ring_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.02
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.14,
            0.04,
            0.14
          ],
          "material": "pla_solid"
        }
      ]
    },
    {
      "name": "sidepannel",
      "isRoot": false,
      "mass_kg": 0.9,
      "centerOfMass_m": [
        -0.1,
        0.0,
        0.06
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.036307459,
        "iyy": 0.016758801,
        "izz": 0.051284144,
        "ixy": 0.0,
        "ixz": 0.003040825,
        "iyz": 0.0
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/sidepannel.glb",
      "colliders": [
        {
          "name": "sidepannel_hopper_col",
          "type": "box",
          "origin_m": [
            -0.1,
            0.0,
            0.06
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.36,
            0.46,
            0.12
          ],
          "material": "polycarbonate"
        }
      ]
    },
    {
      "name": "moving_hood",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.06,
        0.0,
        0.04
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.001838126,
        "iyy": 0.001843983,
        "izz": 0.003395373,
        "ixy": 1.05e-07,
        "ixz": -0.000290957,
        "iyz": 1.7e-08
      },
      "material": "aluminum_6061",
      "visualMesh": "Meshes/moving_hood.glb",
      "colliders": [
        {
          "name": "moving_hood_col",
          "type": "box",
          "origin_m": [
            0.06,
            0.0,
            0.04
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.14,
            0.2,
            0.06
          ],
          "material": "aluminum_6061"
        }
      ]
    },
    {
      "name": "side_plates_2",
      "isRoot": false,
      "mass_kg": 2.5,
      "centerOfMass_m": [
        -0.14,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.306293367,
        "iyy": 0.088370339,
        "izz": 0.266940609,
        "ixy": -0.002669259,
        "ixz": 0.012805409,
        "iyz": 9.1886e-05
      },
      "material": "aluminum_6061",
      "visualMesh": "Meshes/side_plates_2.glb",
      "colliders": [
        {
          "name": "intake_arms_col",
          "type": "box",
          "origin_m": [
            -0.14,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.28,
            0.5,
            0.06
          ],
          "material": "aluminum_6061"
        }
      ]
    },
    {
      "name": "wheels",
      "isRoot": false,
      "mass_kg": 0.65,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.00137949,
        "iyy": 0.000330192,
        "izz": 0.00137949,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0
      },
      "material": "nylon_66",
      "visualMesh": "Meshes/wheels.glb",
      "colliders": [
        {
          "name": "flywheel_rollers_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.05,
            0.18,
            0.05
          ],
          "material": "nylon_66"
        }
      ]
    },
    {
      "name": "side_plates",
      "isRoot": false,
      "mass_kg": 0.4,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.003941294,
        "iyy": 0.003076865,
        "izz": 0.003909644,
        "ixy": 0.0,
        "ixz": -0.000120699,
        "iyz": 0.0
      },
      "material": "polycarbonate",
      "visualMesh": "Meshes/side_plates.glb",
      "colliders": [
        {
          "name": "funnel_col",
          "type": "box",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.16,
            0.22,
            0.1
          ],
          "material": "polycarbonate"
        }
      ]
    },
    {
      "name": "2_hex_shaft_36_length_v2_3",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.010951789,
        "iyy": 1.2362e-05,
        "izz": 0.010951788,
        "ixy": 8e-09,
        "ixz": 0.0,
        "iyz": -1e-09
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/2_hex_shaft_36_length_v2_3.glb",
      "colliders": [
        {
          "name": "2_hex_shaft_36_length_v2_3_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.015,
            0.46,
            0.015
          ],
          "material": "steel_4140"
        }
      ]
    },
    {
      "name": "2_hex_shaft_36_length_v2_4",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.010887194,
        "iyy": 0.000733894,
        "izz": 0.011600855,
        "ixy": -0.000218427,
        "ixz": -3.74e-07,
        "iyz": 1.34e-07
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/2_hex_shaft_36_length_v2_4.glb",
      "colliders": [
        {
          "name": "2_hex_shaft_36_length_v2_4_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.015,
            0.46,
            0.015
          ],
          "material": "steel_4140"
        }
      ]
    },
    {
      "name": "2_hex_shaft_36_length_v2_5",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.012676488,
        "iyy": 2.0119e-05,
        "izz": 0.012676375,
        "ixy": -1.97e-07,
        "ixz": 0.0,
        "iyz": 3.41e-07
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/2_hex_shaft_36_length_v2_5.glb",
      "colliders": [
        {
          "name": "2_hex_shaft_36_length_v2_5_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.015,
            0.46,
            0.015
          ],
          "material": "steel_4140"
        }
      ]
    },
    {
      "name": "2_hex_shaft_36_length_v2_2",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.012578384,
        "iyy": 2.0132e-05,
        "izz": 0.012578283,
        "ixy": -2.09e-07,
        "ixz": 0.0,
        "iyz": 3.65e-07
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/2_hex_shaft_36_length_v2_2.glb",
      "colliders": [
        {
          "name": "2_hex_shaft_36_length_v2_2_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.015,
            0.46,
            0.015
          ],
          "material": "steel_4140"
        }
      ]
    },
    {
      "name": "2_hex_shaft_36_length_v2_6",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.012578384,
        "iyy": 2.0132e-05,
        "izz": 0.012578283,
        "ixy": -2.09e-07,
        "ixz": 0.0,
        "iyz": 3.65e-07
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/2_hex_shaft_36_length_v2_6.glb",
      "colliders": [
        {
          "name": "2_hex_shaft_36_length_v2_6_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.015,
            0.46,
            0.015
          ],
          "material": "steel_4140"
        }
      ]
    },
    {
      "name": "2_hex_shaft_36_length_v2",
      "isRoot": false,
      "mass_kg": 0.35,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.010666911,
        "iyy": 1.1875e-05,
        "izz": 0.010666911,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/2_hex_shaft_36_length_v2.glb",
      "colliders": [
        {
          "name": "2_hex_shaft_36_length_v2_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.015,
            0.46,
            0.015
          ],
          "material": "steel_4140"
        }
      ]
    },
    {
      "name": "25_hub_sprocket_16t_12in_hex_2",
      "isRoot": false,
      "mass_kg": 0.45,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.016719065,
        "iyy": 2.2761e-05,
        "izz": 0.016719066,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": -5e-08
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/25_hub_sprocket_16t_12in_hex_2.glb",
      "colliders": [
        {
          "name": "25_hub_sprocket_16t_12in_hex_2_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.025,
            0.48,
            0.025
          ],
          "material": "polycarbonate"
        }
      ]
    },
    {
      "name": "25_hub_sprocket_16t_12in_hex",
      "isRoot": false,
      "mass_kg": 0.45,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.019634669,
        "iyy": 2.6907e-05,
        "izz": 0.019634673,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 1.1e-07
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/25_hub_sprocket_16t_12in_hex.glb",
      "colliders": [
        {
          "name": "25_hub_sprocket_16t_12in_hex_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.025,
            0.48,
            0.025
          ],
          "material": "polycarbonate"
        }
      ]
    },
    {
      "name": "25_hub_sprocket_16t_12in_hex_3",
      "isRoot": false,
      "mass_kg": 0.45,
      "centerOfMass_m": [
        0.0,
        0.0,
        0.0
      ],
      "inertiaTensor_kgm2": {
        "ixx": 0.017788798,
        "iyy": 2.4372e-05,
        "izz": 0.0177888,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 1.29e-07
      },
      "material": "steel_4140",
      "visualMesh": "Meshes/25_hub_sprocket_16t_12in_hex_3.glb",
      "colliders": [
        {
          "name": "25_hub_sprocket_16t_12in_hex_3_col",
          "type": "cylinder",
          "origin_m": [
            0.0,
            0.0,
            0.0
          ],
          "rotation_quat": [
            1.0,
            0.0,
            0.0,
            0.0
          ],
          "dimensions_m": [
            0.025,
            0.48,
            0.025
          ],
          "material": "polycarbonate"
        }
      ]
    }
  ],
  "joints": [
    {
      "name": "base_plate_1_Revolute 14",
      "type": "revolute",
      "parentLink": "base_link",
      "childLink": "gear",
      "origin_m": [
        0.22701,
        0.0,
        0.361848
      ],
      "axis": [
        0.0,
        0.0,
        1.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 14",
      "type": "fixed",
      "parentLink": "base_link",
      "childLink": "sidepannel",
      "origin_m": [
        0.0635,
        0.0,
        0.123374
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": true,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 0.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "hardware_1_HoodPivotJoint",
      "type": "revolute",
      "parentLink": "base_link",
      "childLink": "moving_hood",
      "origin_m": [
        0.156811,
        0.0,
        0.406618
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": true,
        "lower": -0.261799,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "intake_1_Revolute 21",
      "type": "revolute",
      "parentLink": "base_link",
      "childLink": "side_plates_2",
      "origin_m": [
        -0.3302,
        0.0,
        0.2413
      ],
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "hasLimits": true,
        "lower": -2.70526,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "new_frc_turret_1_FlywheelJoint",
      "type": "revolute",
      "parentLink": "base_link",
      "childLink": "wheels",
      "origin_m": [
        0.156811,
        0.0,
        0.406618
      ],
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "turret_feeding_mech_Revolute 11",
      "type": "revolute",
      "parentLink": "base_link",
      "childLink": "side_plates",
      "origin_m": [
        0.135771,
        0.0,
        0.301143
      ],
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 15",
      "type": "revolute",
      "parentLink": "sidepannel",
      "childLink": "2_hex_shaft_36_length_v2_3",
      "origin_m": [
        -0.032692,
        0.0,
        0.030117
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 16",
      "type": "revolute",
      "parentLink": "sidepannel",
      "childLink": "2_hex_shaft_36_length_v2_4",
      "origin_m": [
        -0.085395,
        0.0,
        0.065538
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 17",
      "type": "revolute",
      "parentLink": "sidepannel",
      "childLink": "2_hex_shaft_36_length_v2_5",
      "origin_m": [
        -0.148895,
        0.0,
        0.065538
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 18",
      "type": "revolute",
      "parentLink": "sidepannel",
      "childLink": "2_hex_shaft_36_length_v2_2",
      "origin_m": [
        -0.239357,
        0.0,
        0.065538
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 19",
      "type": "revolute",
      "parentLink": "sidepannel",
      "childLink": "2_hex_shaft_36_length_v2_6",
      "origin_m": [
        -0.329819,
        0.0,
        0.065538
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "bottom_basket_1_Revolute 20",
      "type": "revolute",
      "parentLink": "sidepannel",
      "childLink": "2_hex_shaft_36_length_v2",
      "origin_m": [
        -0.393319,
        0.0,
        0.065538
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "intake_1_Revolute 29",
      "type": "revolute",
      "parentLink": "side_plates_2",
      "childLink": "25_hub_sprocket_16t_12in_hex_2",
      "origin_m": [
        -0.302722,
        0.0,
        -0.04685
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "intake_1_Revolute 30",
      "type": "revolute",
      "parentLink": "side_plates_2",
      "childLink": "25_hub_sprocket_16t_12in_hex",
      "origin_m": [
        -0.253706,
        0.0,
        0.042144
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    },
    {
      "name": "intake_1_Revolute 31",
      "type": "revolute",
      "parentLink": "side_plates_2",
      "childLink": "25_hub_sprocket_16t_12in_hex_3",
      "origin_m": [
        -0.082908,
        0.0,
        -0.0508
      ],
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "hasLimits": false,
        "lower": 0.0,
        "upper": 0.0,
        "unit": "rad",
        "springFrequency_Hz": 40.0,
        "springDampingRatio": 1.0
      }
    }
  ],
  "contactZones": []
}
)ROBOT_JSON";

    inline constexpr std::string_view kSidecarJson = R"ROBOT_JSON(
{
  "bumpers": {
    "color": "red",
    "enabled": true,
    "height_m": 0.127,
    "length_m": 0.88,
    "mass_kg": 5.0,
    "material": "bumper_fabric",
    "offset_m": [
      0.0,
      0.0,
      0.08
    ],
    "shape": "perimeter",
    "targetLink": "base_link",
    "thickness_m": 0.06984999775886536,
    "width_m": 0.88
  },
  "contactZones": [
    {
      "compressionAxis": [
        0.0,
        0.0,
        1.0
      ],
      "dimensions_m": [
        0.5199999809265137,
        0.5,
        0.3050000071525574
      ],
      "name": "basketZone",
      "nominalGap_m": 0.08,
      "origin_m": [
        -0.17499999701976776,
        0.0,
        0.3700000047683716
      ],
      "parentLink": "base_link",
      "role": "intake",
      "rotation_quat": [
        1.0,
        0.0,
        0.0,
        0.0
      ],
      "shape": "box",
      "tangentAxis": [
        1.0,
        0.0,
        0.0
      ]
    },
    {
      "compressionAxis": [
        0.0,
        0.0,
        1.0
      ],
      "dimensions_m": [
        0.35,
        0.55,
        0.3
      ],
      "name": "IntakeZone",
      "nominalGap_m": 0.08,
      "origin_m": [
        -0.45,
        0.0,
        0.18
      ],
      "parentLink": "base_link",
      "role": "intake",
      "rotation_quat": [
        1.0,
        0.0,
        0.0,
        0.0
      ],
      "shape": "box",
      "tangentAxis": [
        1.0,
        0.0,
        0.0
      ]
    },
    {
      "compressionAxis": [
        0.0,
        0.0,
        1.0
      ],
      "dimensions_m": [
        0.22499999403953552,
        0.19499999284744263,
        0.22499999403953552
      ],
      "name": "FunnelContactZone",
      "nominalGap_m": 0.08,
      "origin_m": [
        0.20000000298023224,
        0.0,
        0.2199999988079071
      ],
      "parentLink": "base_link",
      "role": "intake",
      "rotation_quat": [
        1.0,
        0.0,
        0.0,
        0.0
      ],
      "shape": "box",
      "tangentAxis": [
        1.0,
        0.0,
        0.0
      ]
    },
    {
      "compressionAxis": [
        -0.70710678,
        0.0,
        0.70710678
      ],
      "dimensions_m": [
        0.24,
        0.24,
        0.24
      ],
      "name": "FlyWHeelZone",
      "nominalGap_m": 0.08,
      "origin_m": [
        0.18,
        0.0,
        0.42
      ],
      "parentLink": "base_link",
      "role": "shooter",
      "rotation_quat": [
        1.0,
        0.0,
        0.0,
        0.0
      ],
      "shape": "box",
      "tangentAxis": [
        0.70710678,
        0.0,
        0.70710678
      ]
    }
  ],
  "couplings": [
    {
      "name": "BottomBasket1",
      "powerCoupling": {
        "backlashRad": 0.0,
        "efficiency": 0.95,
        "inverted": false,
        "masterJoint": "bottom_basket_1_Revolute 15",
        "ratio": 1.0,
        "slaveJoint": "bottom_basket_1_Revolute 16",
        "transmissionType": "belt"
      },
      "solverMode": "analytic_coupled",
      "type": "power_coupling"
    },
    {
      "name": "BottomBasket2",
      "powerCoupling": {
        "backlashRad": 0.0,
        "efficiency": 0.95,
        "inverted": false,
        "masterJoint": "bottom_basket_1_Revolute 16",
        "ratio": 1.0,
        "slaveJoint": "bottom_basket_1_Revolute 17",
        "transmissionType": "belt"
      },
      "solverMode": "analytic_coupled",
      "type": "power_coupling"
    },
    {
      "name": "BottomBasket3",
      "powerCoupling": {
        "backlashRad": 0.0,
        "efficiency": 0.95,
        "inverted": false,
        "masterJoint": "bottom_basket_1_Revolute 17",
        "ratio": 1.0,
        "slaveJoint": "bottom_basket_1_Revolute 18",
        "transmissionType": "belt"
      },
      "solverMode": "analytic_coupled",
      "type": "power_coupling"
    },
    {
      "name": "BottomBasket4",
      "powerCoupling": {
        "backlashRad": 0.0,
        "efficiency": 0.95,
        "inverted": false,
        "masterJoint": "bottom_basket_1_Revolute 18",
        "ratio": 1.0,
        "slaveJoint": "bottom_basket_1_Revolute 19",
        "transmissionType": "belt"
      },
      "solverMode": "analytic_coupled",
      "type": "power_coupling"
    },
    {
      "name": "BottomBasket5",
      "powerCoupling": {
        "backlashRad": 0.0,
        "efficiency": 0.95,
        "inverted": false,
        "masterJoint": "bottom_basket_1_Revolute 19",
        "ratio": 1.0,
        "slaveJoint": "bottom_basket_1_Revolute 20",
        "transmissionType": "gear"
      },
      "solverMode": "analytic_coupled",
      "type": "power_coupling"
    },
    {
      "name": "FrontIntake",
      "powerCoupling": {
        "backlashRad": 0.0,
        "efficiency": 0.95,
        "inverted": false,
        "masterJoint": "intake_1_Revolute 29",
        "ratio": 1.0,
        "slaveJoint": "intake_1_Revolute 30",
        "transmissionType": "chain"
      },
      "solverMode": "analytic_coupled",
      "type": "power_coupling"
    }
  ],
  "electrical": {
    "brownoutRecovery_V": 7.5,
    "brownoutShutoff_V": 6.3,
    "brownoutWarning_V": 6.8,
    "internalResistance_Ohm": 0.023,
    "openCircuitVoltage_V": 12.6,
    "quiescentCurrent_A": 1.5,
    "servoRailBuckEfficiency": 0.88,
    "servoRailCurrentLimit_A": 2.2,
    "servoRailVoltage_V": 6.0
  },
  "jointOverrides": {
    "base_plate_1_Revolute 14": {
      "hasLimits": true,
      "lower": 1.5707963705062866,
      "springDampingRatio": 1.0,
      "springFrequency_Hz": 40.0,
      "unit": "rad",
      "upper": 4.71238899230957
    },
    "bottom_basket_1_Revolute 14": {
      "hasLimits": true,
      "lower": 0.0,
      "springDampingRatio": 1.0,
      "springFrequency_Hz": 0.0,
      "unit": "rad",
      "upper": 0.0
    }
  },
  "massReconciliation": {
    "linkMassMultiplier": 0.8147802754480997,
    "unmodeledBallast": {
      "mass_kg": 0.0,
      "offset_m": [
        0.0,
        0.0,
        0.0
      ],
      "targetLink": "base_link"
    }
  },
  "mechanisms": [
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 14,
          "inverted": false,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "gearRatio": 4.0,
      "motorPreset": "KrakenX44",
      "name": "Turret",
      "parentLink": "base_link",
      "role": "turret",
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "base_plate_1_Revolute 14",
      "viscousDamping": 0.0
    },
    {
      "actuatorType": "servo",
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "gearRatio": 9.050000190734863,
      "name": "Hood",
      "parentLink": "base_link",
      "role": "arm",
      "servo": {
        "mode": "positional",
        "preset": "GoBildaTorque",
        "pwmChannel": 9,
        "supplyVoltage_V": 6.0,
        "travelRange_deg": 300.0
      },
      "targetJoint": "hardware_1_HoodPivotJoint",
      "viscousDamping": 0.0
    },
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 12,
          "inverted": false,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        },
        {
          "canBus": "canivore",
          "canId": 11,
          "inverted": true,
          "role": "follower",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.800000011920929,
      "flywheelInertia_kgm2": 0.0,
      "gearRatio": 81.0,
      "motorPreset": "KrakenX60",
      "name": "IntakePivot",
      "parentLink": "base_link",
      "role": "arm",
      "rollerRadius_m": 0.0508,
      "spinAxis": [
        0.0,
        1.0,
        0.0
      ],
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "intake_1_Revolute 21",
      "viscousDamping": 0.0
    },
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 15,
          "inverted": true,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "flywheelInertia_kgm2": 0.0032,
      "gearRatio": 1.0,
      "motorPreset": "KrakenX44",
      "mountOffset_m": [
        0.156811,
        0.0,
        0.406618
      ],
      "name": "Flywheel",
      "parentLink": "base_link",
      "role": "flywheel",
      "rollerRadius_m": 0.0508,
      "spinAxis": [
        0.0,
        1.0,
        0.0
      ],
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "new_frc_turret_1_FlywheelJoint",
      "viscousDamping": 0.0,
      "visualLink": "wheels"
    },
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 13,
          "inverted": false,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "flywheelInertia_kgm2": 0.0,
      "gearRatio": 4.0,
      "motorPreset": "KrakenX44",
      "mountOffset_m": [
        0.135771,
        0.0,
        0.301143
      ],
      "name": "Funnel",
      "parentLink": "base_link",
      "role": "intake",
      "rollerRadius_m": 0.0508,
      "spinAxis": [
        0.0,
        1.0,
        0.0
      ],
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "turret_feeding_mech_Revolute 11",
      "viscousDamping": 0.0,
      "visualLink": "side_plates"
    },
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 10,
          "inverted": true,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "flywheelInertia_kgm2": 0.0,
      "gearRatio": 4.0,
      "motorPreset": "KrakenX44",
      "name": "BottomBasket",
      "parentLink": "sidepannel",
      "role": "intake",
      "rollerRadius_m": 0.0508,
      "spinAxis": [
        0.0,
        1.0,
        0.0
      ],
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "bottom_basket_1_Revolute 15",
      "viscousDamping": 0.0
    },
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 8,
          "inverted": false,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "flywheelInertia_kgm2": 0.0,
      "gearRatio": 4.0,
      "motorPreset": "KrakenX44",
      "name": "FrontIntake",
      "parentLink": "side_plates_2",
      "role": "intake",
      "rollerRadius_m": 0.0508,
      "spinAxis": [
        0.0,
        1.0,
        0.0
      ],
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "intake_1_Revolute 29",
      "viscousDamping": 0.0
    },
    {
      "controllers": [
        {
          "canBus": "canivore",
          "canId": 9,
          "inverted": false,
          "role": "leader",
          "vendor": "CTRE_TalonFX"
        }
      ],
      "coulombFriction": 0.0,
      "efficiency": 0.95,
      "flywheelInertia_kgm2": 0.0,
      "gearRatio": 4.0,
      "motorPreset": "KrakenX60",
      "name": "BackIntake",
      "parentLink": "side_plates_2",
      "role": "intake",
      "rollerRadius_m": 0.0508,
      "spinAxis": [
        0.0,
        1.0,
        0.0
      ],
      "statorCurrentLimit_A": 40.0,
      "supplyCurrentLimit_A": 40.0,
      "targetJoint": "intake_1_Revolute 31",
      "viscousDamping": 0.0
    }
  ],
  "schemaVersion": "1.0.0",
  "sensors": [
    {
      "bus": "canivore",
      "canId": 24,
      "debounceTime_s": 0.01,
      "limitDirection": "forward",
      "name": "imu_pigeon2",
      "parentLink": "base_link",
      "type": "imu_pigeon2"
    },
    {
      "bus": "i2c_onboard",
      "cpr": 1024,
      "customStringParams": {
        "encoderMode": "duty_cycle"
      },
      "debounceTime_s": 0.01,
      "i2cAddress": 1,
      "limitDirection": "forward",
      "name": "PivotEncoder",
      "offset_rad": 0.0,
      "secondaryDioChannel": 3,
      "sensorToMechanismRatio": 1.0,
      "targetJoint": "intake_1_Revolute 21",
      "type": "rev_through_bore"
    },
    {
      "bus": "rio",
      "contactZone": "IntakeZone",
      "debounceTime_s": 0.01,
      "dioChannel": 2,
      "inverted": true,
      "name": "intake_beam_break",
      "type": "beam_break"
    },
    {
      "bus": "rio",
      "contactZone": "FlyWHeelZone",
      "debounceTime_s": 0.01,
      "dioChannel": 3,
      "inverted": true,
      "name": "shooter_beam_break",
      "type": "beam_break"
    }
  ],
  "swerve": {
    "chassisLink": "base_link",
    "driveEfficiency": 0.96,
    "driveGearRatio": 6.03,
    "driveMotorPreset": "KrakenX60",
    "driveStatorLimit_A": 60.0,
    "driveSupplyLimit_A": 40.0,
    "modules": [
      {
        "canBus": "canivore",
        "cancoderId": 22,
        "driveCanId": 1,
        "name": "FrontLeft",
        "pos_m": [
          -0.3140000104904175,
          0.0,
          0.25099998712539673
        ],
        "steerCanId": 0
      },
      {
        "canBus": "canivore",
        "cancoderId": 23,
        "driveCanId": 3,
        "name": "FrontRight",
        "pos_m": [
          0.3140000104904175,
          0.0,
          0.25099998712539673
        ],
        "steerCanId": 2
      },
      {
        "canBus": "canivore",
        "cancoderId": 21,
        "driveCanId": 5,
        "name": "BackLeft",
        "pos_m": [
          -0.3140000104904175,
          0.0,
          -0.25099998712539673
        ],
        "steerCanId": 4
      },
      {
        "canBus": "canivore",
        "cancoderId": 20,
        "driveCanId": 7,
        "name": "BackRight",
        "pos_m": [
          0.3140000104904175,
          0.0,
          -0.25099998712539673
        ],
        "steerCanId": 6
      }
    ],
    "steerEfficiency": 0.92,
    "steerGearRatio": 26.091,
    "steerMotorPreset": "KrakenX44",
    "steerStatorLimit_A": 40.0,
    "steerSupplyLimit_A": 30.0,
    "wheelRadius_m": 0.0508
  },
  "targetManifest": "manifest.json"
}
)ROBOT_JSON";

    inline constexpr std::string_view kMaterialsJson = R"ROBOT_JSON(
{
  "schemaVersion": "1.0.0",
  "materials": [
    {
      "name": "abs_infill_35",
      "density_kg_m3": 600.0,
      "friction": 0.38,
      "restitution": 0.18
    },
    {
      "name": "abs_solid",
      "density_kg_m3": 1040.0,
      "friction": 0.38,
      "restitution": 0.18
    },
    {
      "name": "aluminum_6061",
      "density_kg_m3": 2700.0,
      "friction": 0.45,
      "restitution": 0.05
    },
    {
      "name": "battery_frc_12v",
      "density_kg_m3": 2570.0,
      "friction": 0.45,
      "restitution": 0.05
    },
    {
      "name": "bilinear_carpet",
      "density_kg_m3": 400.0,
      "friction": 0.95,
      "restitution": 0.01
    },
    {
      "name": "bumper_fabric",
      "density_kg_m3": 500.0,
      "friction": 0.70,
      "restitution": 0.05
    },
    {
      "name": "bumper_wood_birch",
      "density_kg_m3": 690.0,
      "friction": 0.45,
      "restitution": 0.10
    },
    {
      "name": "cats_tongue_tape",
      "density_kg_m3": 1150.0,
      "friction": 1.45,
      "restitution": 0.10
    },
    {
      "name": "colson_performa",
      "density_kg_m3": 1100.0,
      "friction": 1.10,
      "restitution": 0.05
    },
    {
      "name": "duct_tape",
      "density_kg_m3": 1020.0,
      "friction": 0.60,
      "restitution": 0.05
    },
    {
      "name": "game_piece_foam",
      "density_kg_m3": 80.0,
      "friction": 0.65,
      "restitution": 0.20
    },
    {
      "name": "motor_775pro",
      "density_kg_m3": 3800.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_bare_ftc_550",
      "density_kg_m3": 3540.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_cim",
      "density_kg_m3": 4500.0,
      "friction": 0.45,
      "restitution": 0.05
    },
    {
      "name": "motor_ctre_minion",
      "density_kg_m3": 3850.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_falcon_500",
      "density_kg_m3": 3700.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_gobilda_yellowjacket",
      "density_kg_m3": 3530.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_kraken_x44",
      "density_kg_m3": 3720.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_kraken_x60",
      "density_kg_m3": 3760.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_neo_550",
      "density_kg_m3": 3740.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_neo_v1",
      "density_kg_m3": 3410.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_neo_v2",
      "density_kg_m3": 3420.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_neo_vortex",
      "density_kg_m3": 3470.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "motor_rev_core_hex",
      "density_kg_m3": 2900.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "nylon_66",
      "density_kg_m3": 1140.0,
      "friction": 0.28,
      "restitution": 0.12
    },
    {
      "name": "nylon_glass_filled",
      "density_kg_m3": 1360.0,
      "friction": 0.32,
      "restitution": 0.15
    },
    {
      "name": "pla_cf_infill_35",
      "density_kg_m3": 720.0,
      "friction": 0.36,
      "restitution": 0.12
    },
    {
      "name": "pla_cf_solid",
      "density_kg_m3": 1280.0,
      "friction": 0.36,
      "restitution": 0.12
    },
    {
      "name": "pla_infill_35",
      "density_kg_m3": 700.0,
      "friction": 0.35,
      "restitution": 0.15
    },
    {
      "name": "pla_solid",
      "density_kg_m3": 1240.0,
      "friction": 0.35,
      "restitution": 0.15
    },
    {
      "name": "plexi_glass",
      "density_kg_m3": 1190.0,
      "friction": 0.40,
      "restitution": 0.20
    },
    {
      "name": "polycarbonate",
      "density_kg_m3": 1200.0,
      "friction": 0.40,
      "restitution": 0.15
    },
    {
      "name": "pool_noodle_hollow",
      "density_kg_m3": 28.0,
      "friction": 0.60,
      "restitution": 0.12
    },
    {
      "name": "pool_noodle_solid",
      "density_kg_m3": 32.0,
      "friction": 0.60,
      "restitution": 0.15
    },
    {
      "name": "rev_maxplanetary",
      "density_kg_m3": 4200.0,
      "friction": 0.45,
      "restitution": 0.05
    },
    {
      "name": "sds_spike_grip_tread",
      "density_kg_m3": 1210.0,
      "friction": 1.45,
      "restitution": 0.02
    },
    {
      "name": "silicon_35a",
      "density_kg_m3": 1100.0,
      "friction": 1.35,
      "restitution": 0.38
    },
    {
      "name": "silicon_40a",
      "density_kg_m3": 1120.0,
      "friction": 1.30,
      "restitution": 0.35
    },
    {
      "name": "silicon_50a",
      "density_kg_m3": 1150.0,
      "friction": 1.20,
      "restitution": 0.30
    },
    {
      "name": "silicon_60a",
      "density_kg_m3": 1180.0,
      "friction": 1.10,
      "restitution": 0.25
    },
    {
      "name": "steel_4140",
      "density_kg_m3": 7850.0,
      "friction": 0.50,
      "restitution": 0.10
    },
    {
      "name": "steel_mild_1018",
      "density_kg_m3": 7870.0,
      "friction": 0.55,
      "restitution": 0.10
    },
    {
      "name": "surgical_tubing",
      "density_kg_m3": 960.0,
      "friction": 1.10,
      "restitution": 0.40
    },
    {
      "name": "tpu_85a",
      "density_kg_m3": 1150.0,
      "friction": 1.05,
      "restitution": 0.30
    },
    {
      "name": "tpu_90a",
      "density_kg_m3": 1180.0,
      "friction": 0.95,
      "restitution": 0.28
    },
    {
      "name": "tpu_95a",
      "density_kg_m3": 1210.0,
      "friction": 0.85,
      "restitution": 0.25
    },
    {
      "name": "wheel_sds_mk5_lumped",
      "density_kg_m3": 1250.0,
      "friction": 1.45,
      "restitution": 0.03
    },
    {
      "name": "servo_standard",
      "density_kg_m3": 2000.0,
      "friction": 0.40,
      "restitution": 0.05
    },
    {
      "name": "steel_52100_bearing",
      "density_kg_m3": 7810.0,
      "friction": 0.15,
      "restitution": 0.08
    }
  ],
  "materialPairs": [
    {
      "materialA": "colson_performa",
      "materialB": "bilinear_carpet",
      "staticFriction": 1.25,
      "dynamicFriction": 1.10,
      "restitution": 0.02,
      "rollingResistance": 0.015
    },
    {
      "materialA": "sds_spike_grip_tread",
      "materialB": "bilinear_carpet",
      "staticFriction": 1.65,
      "dynamicFriction": 1.45,
      "restitution": 0.02,
      "rollingResistance": 0.020
    },
    {
      "materialA": "wheel_sds_mk5_lumped",
      "materialB": "bilinear_carpet",
      "staticFriction": 1.65,
      "dynamicFriction": 1.45,
      "restitution": 0.02,
      "rollingResistance": 0.020
    },
    {
      "materialA": "tpu_95a",
      "materialB": "bilinear_carpet",
      "staticFriction": 1.15,
      "dynamicFriction": 1.00,
      "restitution": 0.04,
      "rollingResistance": 0.015
    },
    {
      "materialA": "colson_performa",
      "materialB": "polycarbonate",
      "staticFriction": 0.65,
      "dynamicFriction": 0.55,
      "restitution": 0.08,
      "rollingResistance": 0.005
    },
    {
      "materialA": "sds_spike_grip_tread",
      "materialB": "polycarbonate",
      "staticFriction": 0.70,
      "dynamicFriction": 0.60,
      "restitution": 0.08,
      "rollingResistance": 0.008
    },
    {
      "materialA": "game_piece_foam",
      "materialB": "polycarbonate",
      "staticFriction": 0.75,
      "dynamicFriction": 0.68,
      "restitution": 0.22,
      "rollingResistance": 0.0
    },
    {
      "materialA": "game_piece_foam",
      "materialB": "plexi_glass",
      "staticFriction": 0.70,
      "dynamicFriction": 0.62,
      "restitution": 0.25,
      "rollingResistance": 0.0
    },
    {
      "materialA": "cats_tongue_tape",
      "materialB": "game_piece_foam",
      "staticFriction": 1.45,
      "dynamicFriction": 1.30,
      "restitution": 0.10,
      "rollingResistance": 0.0
    },
    {
      "materialA": "colson_performa",
      "materialB": "game_piece_foam",
      "staticFriction": 0.95,
      "dynamicFriction": 0.85,
      "restitution": 0.18,
      "rollingResistance": 0.0
    },
    {
      "materialA": "silicon_40a",
      "materialB": "game_piece_foam",
      "staticFriction": 1.35,
      "dynamicFriction": 1.20,
      "restitution": 0.25,
      "rollingResistance": 0.0
    },
    {
      "materialA": "silicon_50a",
      "materialB": "game_piece_foam",
      "staticFriction": 1.25,
      "dynamicFriction": 1.10,
      "restitution": 0.22,
      "rollingResistance": 0.0
    },
    {
      "materialA": "tpu_95a",
      "materialB": "game_piece_foam",
      "staticFriction": 0.90,
      "dynamicFriction": 0.80,
      "restitution": 0.20,
      "rollingResistance": 0.0
    },
    {
      "materialA": "bumper_fabric",
      "materialB": "polycarbonate",
      "staticFriction": 0.55,
      "dynamicFriction": 0.45,
      "restitution": 0.10,
      "rollingResistance": 0.0
    },
    {
      "materialA": "bumper_fabric",
      "materialB": "aluminum_6061",
      "staticFriction": 0.50,
      "dynamicFriction": 0.42,
      "restitution": 0.08,
      "rollingResistance": 0.0
    },
    {
      "materialA": "bumper_fabric",
      "materialB": "bumper_fabric",
      "staticFriction": 0.65,
      "dynamicFriction": 0.55,
      "restitution": 0.05,
      "rollingResistance": 0.0
    }
  ]
}
)ROBOT_JSON";
}
