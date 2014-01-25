{
    "scene":
    {
        "file": "Meshes/zig_zag_200.obj"
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
            "position": [5, 0, -1],
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
                    "destination": [1, 0, -25]
               }
            }
        }
    ]
}