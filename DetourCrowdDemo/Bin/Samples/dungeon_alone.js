{
	"scene":
	{
		"file": "Meshes/dungeon.obj"
	},
	"agents":
	[
		{
			"position": [40, 10, 0],
			"parameters": 
			{
				"maxSpeed": 2,
				"maxAcceleration": 10,
				"radius": 0.2,
				"height": 1.7,
				"collisionQueryRange": 4.0,
				"pipeline":
				[
					{
						"behavior":
						{
							"type": "pathFollowing",
							"destination": [20, 20, -80],
							"visibilityPathOptimizationRange": 6,
							"initialPathfindIterCount": 0,
							"anticipateTurns": true,
							"localPathReplanningInterval": 0.5
						}
					},
					{                       
						"behavior":
						{
							"type": "collisionAvoidance"
						}
					}
				]
			}
		}
	]
}