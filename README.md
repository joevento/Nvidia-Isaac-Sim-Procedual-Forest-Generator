# Terrain Generation and Forest Rendering Extension

This project implements an extension for NVIDIA Isaac Sim for generating terrain and rendering forests within the simulation environment. It utilizes USD (Universal Scene Description) for scene representation and manipulation, allowing for efficient handling of large-scale environments. The extension was first made for virtual robot testing based on parametrized data taken from real world forests.

## Features

- **Terrain Generation**: The extension can generate terrain based on specified parameters such as size, roughness, and terrain type.
- **Forest Rendering**: Users can generate forests with different types of trees (e.g., birch, spruce, pine) and adjust parameters like density and age range.
- **HDR Environment**: The extension supports the addition and removal of HDR (High Dynamic Range) environment textures for realistic lighting.
- **Rocks Generation**: Rocks can be generated on the terrain with adjustable rockiness parameters.
- **Vegitation Generation**: Other types of vegetation can also be generated alongside trees, with customizable density.
- **Collision Detection**: Terrain collision properties are automatically configured for physics simulation.
- **User Interface**: The extension provides a user-friendly UI for adjusting parameters and triggering actions.

## Installation

1. Clone the repository to your local machine.
2. Open the NVIDIA Isaac Sim environment.
3. Copy the extension files into the appropriate directory within the Isaac Sim environment.
4. Enable the extension from the Isaac Sim environment's menu.

## Usage

1. Open the extension from the menu in the Isaac Sim environment.
2. Adjust the parameters in the UI for terrain generation, forest rendering, HDR environment, rocks generation, and other options.
3. Click the corresponding buttons to trigger actions such as generating terrain, forests, rocks, or HDR environments.
4. Interact with the simulation environment and observe the generated terrain, forests, and other elements.
5. Save your simulation scenario and project as needed.

## Requirements

- NVIDIA Isaac Sim
- Python (for USD scripting)

## Credits

This project was developed by Joel Ventola as part of research into virtual mobile robot testing conducted by Biomimetics and Intelligent Systems Group
from the University of Oulu.

## License

This project is licensed under the [MIT License](LICENSE).
