# APRG - An Efficient Plane Segmentation Algorithm  

This is the open-source code for the **APRG (On Review)**.

## Dependencies

The code depends on the following libraries:
- [OpenCV](https://opencv.org/)
- [Eigen3](https://eigen.tuxfamily.org/)

## Recommended Development Environment

We recommend using **VS Code Dev Container** to create a development environment from the provided `Dockerfile`.

### Steps to Set Up the Development Environment

1. **Install Prerequisites**:
   - Install [Docker](https://www.docker.com/).
   - Install [Visual Studio Code](https://code.visualstudio.com/).
   - Install the [VS Code Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

2. **Build and Launch the Dev Container**:
   - Open the project folder in VS Code.
   - VS Code will detect the `Dockerfile` and suggest reopening in a Dev Container. Follow the prompts to build and open the container.

3. **Compile and Run in the Container**:
   - Once inside the container's interactive terminal, compile and run the code as follows:
     ```bash
     mkdir build && cd build
     cmake ..
     make
     cd ..
     ./build/Run_EVOPS ./config/sample.json
     ```  

## Notes
- Adjust the `Dockerfile` if additional dependencies are required.

Feel free to contribute to this repository by submitting issues or creating pull requests!
