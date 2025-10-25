<br />
<p align="center">
  <h1 align="center">
    CZ3004/SC2079 Multidisciplinary Project - Algorithm API
  </h1>
</p>


## Setup

```bash
pip install -r requirements.txt
```

Start the server by

```bash
python main.py
```

The server will be running at `localhost:5050`


### Primers - Constants and Parameters 

#### Direction of the robot (d)

* `NORTH` - `UP` - 0
* `EAST` - `RIGHT` - 2
* `SOUTH` - `DOWN` - 4
* `WEST` - `LEFT` 6

#### Parameters

* `EXPANDED_CELL` - Size of an expanded cell, normally set to just 1 unit, but expanding it to 1.5 or 2 will allow the robot to have more space to move around the obstacle at the cost of it being harder to find a shortest path. Useful to tweak if robot is banging into obstacles.
* `WIDTH` - Width of the area (in 10cm units)
* `HEIGHT` - Height of the area (in 10cm units)
* `ITERATIONS` - Number of iterations to run the algorithm for. Higher number of iterations will result in a more accurate shortest path, but will take longer to run. Useful to tweak if robot is not finding the shortest path.
* `TURN_RADIUS` - Number of units the robot turns. We set the turns to `3 * TURN_RADIUS, 1 * TURN_RADIUS` units. Can be tweaked in the algorithm
* `SAFE_COST` - Used to penalise the robot for moving too close to the obstacles. Currently set to `1000`. Take a look at `get_safe_cost` to tweak.
* `SCREENSHOT_COST` - Used to penalise the robot for taking pictures from a position that is not directly in front of the symbol. 

### API Endpoints:


##### 1. POST Request to /path:

Sample JSON request body:

```bash
{
  "obstacles": [
    {
      "x": 5,
      "y": 10,
      "d": 0,
      "id": 1
    },
    {
      "x": 15,
      "y": 5,
      "d": 2,
      "id": 2
    },
    {
      "x": 8,
      "y": 15,
      "d": 4,
      "id": 3
    }
  ],
  "retrying": false,
  "robot_x": 1,
  "robot_y": 1,
  "robot_dir": 0
}
```

Sample JSON response:

```
{
  "data": {
    "commands": [
      "FW030",
      "FR090",
      "FW020",
      "SNAP1_C",
      "BL090",
      "FW040",
      "FL090",
      "SNAP2_L",
      "BR090",
      "FW020",
      "SNAP3_R",
      "FIN"
    ],
    "path": [
      [1, 1, 0],
      [1, 4, 0],
      [5, 4, 2],
      [7, 4, 2],
      [7, 8, 0],
      [7, 12, 0],
      [11, 12, 6],
      [11, 15, 6],
      [8, 15, 4]
    ]
  }
}
```
