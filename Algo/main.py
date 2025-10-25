import time
from algo.algo import MazeSolver 
from flask import Flask, request, jsonify
from flask_cors import CORS
from model import *
from helper import command_generator

app = Flask(__name__)
CORS(app)
#model = load_model()
model = None
@app.route('/status', methods=['GET'])
def status():
    """
    This is a health check endpoint to check if the server is running
    :return: a json object with a key "result" and value "ok"
    """
    return jsonify({"result": "ok"})


@app.route('/path', methods=['POST'])
def path_finding():
    """
    This is the main endpoint for the path finding algorithm
    :return: a json object with a key "data" and value a dictionary with keys "distance", "path", and "commands"
    """
    # Get the json data from the request
    content = request.json

    # Support both direct and RPI-wrapped input
    if 'type' in content and content['type'] == 'FASTEST_PATH' and 'data' in content:
        data = content['data']
    else:
        data = content

    obstacles = data['obstacles']
    retrying = data['retrying']
    robot_x, robot_y = data['robot_x'], data['robot_y']
    robot_direction = int(data['robot_dir'])

    maze_solver = MazeSolver(20, 20, robot_x, robot_y, robot_direction, big_turn=1)
    for ob in obstacles:
        maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

    start = time.time()
    optimal_path, distance = maze_solver.get_optimal_order_dp(retrying=retrying)
    print(f"Time taken to find shortest path using A* search: {time.time() - start}s")
    print(f"Distance to travel: {distance} units")

    commands = command_generator(optimal_path, obstacles)

    # Format commands: pad numbers to 3 digits, angles to 3 digits, keep SNAP/FIN as is

    def format_command(cmd):
        import re
        if cmd.startswith("SNAP") or cmd == "FIN":
            return cmd
        # Always output 090 for 90-degree turns
        if cmd[:2] in ["FR", "FL", "BR", "BL"]:
            return f"{cmd[:2]}090"
        m = re.match(r"([A-Z]+)([0-9]+)", cmd)
        if m:
            prefix, num = m.group(1), m.group(2)
            return f"{prefix}{int(num):03d}"
        return cmd

    formatted_commands = [format_command(c) for c in commands]

    # Format path: output as list of [x, y, d] (float or int)
    path_results = []
    for state in optimal_path:
        # state may be a CellState with x, y, direction
        # Use float if value is float, else int
        x = float(state.x) if isinstance(state.x, float) or (isinstance(state.x, int) and state.x != int(state.x)) else int(state.x)
        y = float(state.y) if isinstance(state.y, float) or (isinstance(state.y, int) and state.y != int(state.y)) else int(state.y)
        d = int(state.direction) if hasattr(state, 'direction') else 0
        path_results.append([x, y, d])

    return jsonify({
        "data": {
            'commands': formatted_commands,
            'path': path_results
        }
    })


@app.route('/image', methods=['POST'])
def image_predict():
    """
    This is the main endpoint for the image prediction algorithm
    :return: a json object with a key "result" and value a dictionary with keys "obstacle_id" and "image_id"
    """
    file = request.files['file']
    filename = file.filename
    file.save(os.path.join('uploads', filename))
    # filename format: "<timestamp>_<obstacle_id>_<signal>.jpeg"
    constituents = file.filename.split("_")
    obstacle_id = constituents[1]

    ## Week 8 ## 
    #signal = constituents[2].strip(".jpg")
    #image_id = predict_image(filename, model, signal)

    ## Week 9 ## 
    # We don't need to pass in the signal anymore
    image_id = predict_image_week_9(filename,model)

    # Return the obstacle_id and image_id
    result = {
        "obstacle_id": obstacle_id,
        "image_id": image_id
    }
    return jsonify(result)

@app.route('/stitch', methods=['GET'])
def stitch():
    """
    This is the main endpoint for the stitching command. Stitches the images using two different functions, in effect creating two stitches, just for redundancy purposes
    """
    img = stitch_image()
    img.show()
    img2 = stitch_image_own()
    img2.show()
    return jsonify({"result": "ok"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050, debug=True)
