# Landmark Model File Generation
This folder contains and a blender to file to generate textures for the landmarks, to be used in a rover simulation.

## Setup
Run the following commands to setup a virtual environment:
```
# Install the package manager uv, if you don't have it:
# curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv
source .venv/bin/activate
uv pip install opencv-python numpy pillow
```

## Usage
Usage:
Run `python texture_generation.py --landmark_nr i` to generate a texture object for the landmark number `i`.
Open `landmark.blend` in blender. Assert that the model shows the correct landmark number. Open a 'Text' workspace in blender and there, open the file model_export. Run the script, selecting the file `landmark.blend` if prompted. The generated Gazebo model will be written to `out/*`
