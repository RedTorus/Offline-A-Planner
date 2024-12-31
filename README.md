# Offline multigoal A* planner for catching moving target with known trajectory in 2D map

Heuristic used: Backward Djikstra <br>
Additional feature: Backtracks traget's trajectory if it reaches a point that is ahead of target <br>
More details in PlanningHW1.pdf

## How to Run the Code

1. **Build the Executable:**
   - Navigate to the `code` directory in your terminal.
   - Create a `build` directory and compile the code:
     ```bash
     mkdir build
     cd build
     cmake ..
     make
     ```
   - Alternatively, use `g++` to compile the code:
     ```bash
     g++ src/runtest.cpp src/planner.cpp -o runtest
     ```

2. **Run the Planner:**
   - Execute the planner with a map file:
     ```bash
     ./runtest maps/map<number>.txt
     ```
   - Example:
     ```bash
     ./runtest maps/map1.txt
     ```

3. **Visualize the Results:**
   - Use the provided Python visualization script to display the robot and target trajectories:
     ```bash
     python scripts/visualizer.py maps/map<number>.txt
     ```
   - Example:
     ```bash
     python scripts/visualizer.py maps/map1.txt
     ```

4. **Simulation Outputs:**
   - The simulation outputs will include:
     - Whether the target was caught.
     - The time taken for the simulation.
     - The number of moves made by the robot.
     - The total cost incurred during the path traversal.

