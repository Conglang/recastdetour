{
    "scene":
    {
        "file": "Meshes/mazyhall.obj"
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
                "localPathReplanningInterval": 1
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
            "position": [9, 0, 0],
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
                    "destination": [-9.5, 0, 0]
               }
            }
        },
        {
            "position": [-9.5, 0, 0],
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
                    "destination": [9.5, 0, 0]
               }
            }
        },
        {
            "position": [9.5, 0, 2.5],
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
                    "destination": [-9.5, 0, -2.5]
               }
            }
        },
        {
            "position": [-9.5, 0, 2.5],
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
                    "destination": [9.5, 0, -2.5]
                }
            }
        },
        {
            "position": [9.5, 0, -2.5],
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
                    "destination": [-9.5, 0, 2.5]
                }
            }
        },
        {
            "position": [-9.5, 0, -2.5],
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
                    "destination": [9.5, 0, 2.5]
                }
            }
        },
        {
            "position": [9.5, 0, 5],
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
                    "destination": [-9.5, 0, -5]
                }
            }
        },
        {
            "position": [-9.5, 0, 5],
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
                    "destination": [9.5, 0, -5]
                }
            }
        },
        {
            "position": [9.5, 0, -5],
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
                    "destination": [-9.5, 0, 5]
                }
            }
        },
        {
            "position": [-9.5, 0, -5],
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
                    "destination": [9.5, 0, 5]
                }
            }
        }
    ]
}