# React Tools Documentation

For tools that need additional components

## Camera
This component is used to display the camera feed inside the WebGUI.

Contains the web cam functionality for the camera tool.

Can be found in react_frontend/src/components/visualizers/Camera.tsx

## WebGUI

There are 2 types of components:

### Preview

Contains the preview component for the WebGUI while it loads. Can be found in react_frontend/src/components/visualizers/WebGUIPreview.tsx

### Exercise WebGUI components

Contains components to be used inside each exercise WebGUI:

- WebGUI3D: 3D viewer -> react_frontend/src/components/exercise/WebGUI3D.tsx
- WebGUIContainer: Container for the WebGUI -> react_frontend/src/components/exercise/WebGUIContainer.tsx
- WebGUIImage: Container for a image in the WebGUI -> react_frontend/src/components/exercise/WebGUIImage.tsx
