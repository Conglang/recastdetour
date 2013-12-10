{
    "scene":
    {
        "file": "Meshes/corridor_120.obj"
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
                "weightCurrentAvoidanceSide": 0.75,
                "weightTimeToCollision": 2.5,
                "sampleOriginScale": 0.4
            }
        ]
    },
    "agents":
    [
        {
            "position": [-4, 0, 0],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "perceptionDistance": 4.0,
            "behavior" : "default",
            "behaviorParams":
            {
               "pathFollowing":
               {
                    "destination": [4, 0, 0]
               }
            }
        },
        {
            "position": [4, 0, 0],
            "maxSpeed": 2,
            "maxAcceleration": 10,
            "radius": 0.2,
            "height": 1.7,
            "perceptionDistance": 4.0,
            "behavior" : "default",
            "behaviorParams":
            {
               "pathFollowing":
               {
                    "destination": [-4, 0, 0]
               }
            }
        }
    ]
}