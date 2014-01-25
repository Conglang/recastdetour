{
    "scene":
    {
        "file": "Meshes/uturn.obj"
    },
    "behaviors":
    {
        "default":
        [
            {
                "type": "pathFollowing",
                "visibilityPathOptimizationRange": 6,
                "initialPathfindIterCount": 0,
                "anticipateTurns": true,
                "localPathReplanningInterval": 0.5
            },
            {
                "type": "collisionAvoidance",
                "weightDesiredVelocity": 2.0,
                "weightCurrentVelocity": 0.75,
                "weightCurrentAvoidanceSide": 0,
                "weightTimeToCollision": 2.5,
                "sampleOriginScale": 0.4
            }
        ]
    },
    "agents":
    [
        {
            "position": [6.5, 0, 9],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "detectionRange": 4.0,
            "behavior" : "default",
            "behaviorParams":
            {
               "pathFollowing":
               {
                    "destination": [-6.5, 0, 9]
               }
            }
        },
        {
            "position": [5.3, 0, 0],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "detectionRange": 4.0
        },
        {
            "position": [4, 0, -3.5],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "detectionRange": 4.0
        },
        {
            "position": [0, 0, -5.3],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "detectionRange": 4.0
        },
        {
            "position": [0.2, 0, -6.3],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "detectionRange": 4.0
        }
        ,
        {
            "position": [-5, 0, -5],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "detectionRange": 4.0
        }
    ]
}