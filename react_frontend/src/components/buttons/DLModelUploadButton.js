import React from "react";
import "./../../styles/buttons/DLModelUploadButton.css";
import "./../../styles/tailwind.css";

// Constants
const white_list_exercises = [
  "end_to_end_visual_control",
  "digit_classification",
  "human_detection",
];
const MB = 1024 * 1024;

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

window.RoboticsReactComponents.DeepLearningModel = (function () {
  let model_buffer = null;

  const setModelBuffer = (modelBuffer) => {
    model_buffer = modelBuffer;
  };

  const getModelBuffer = () => model_buffer;

  return {
    setModelBuffer: setModelBuffer,
    getModelBuffer: getModelBuffer,
  };
})();

// Deep Learning Model Upload Button Component
const DLModelUploadButton = () => {
  const [exerciseName, setExerciseName] = React.useState("");
  const [fileName, setFileName] = React.useState("");
  const [fileSize, setFileSize] = React.useState(0);
  const fileRef = React.useRef(null);

  // Effect to set the exercise name based on the current URL
  React.useEffect(() => {
    const currentPath = window.location.pathname;
    const split = currentPath.split("/").filter(Boolean); // Get the last part of the URL
    const exercise = split[split.length - 1]; // Assuming the last part is the exercise name
    // Check if the exercise is in the whitelist
    if (white_list_exercises.includes(exercise)) {
      setExerciseName(exercise);
    } else {
      setExerciseName(""); // Reset if not in the whitelist
    }
  }, []);

  // Handle file upload
  const handleUpload = async (event) => {
    const file = event.target.files[0];
    if (!file || !file.name.endsWith(".onnx")) {
      alert("Please upload a valid .onnx file");
      return;
    }

    const buffer = await file.arrayBuffer(); // Binary buffer

    console.log("====================================");
    console.log("File uploaded:", file.name);
    console.log("File size:", (file.size / (1024 * 1024)).toFixed(2), "MB");
    console.log("File type:", file.type);
    console.log("====================================");

    // Set the model in the DeepLearningModel component
    window.RoboticsReactComponents.DeepLearningModel.setModelBuffer(buffer);
    // Update the state with file details
    setFileName(file.name);
    setFileSize(file.size);
  };

  // Render the upload button if the exercise is in the whitelist
  return (
    <>
      {exerciseName && white_list_exercises.includes(exerciseName) ? (
        <div
          className={`tooltip-container m-1 border-[1px] hover:border-[1.8px] border-[#008D017F] hover:border-[#008D01] cursor-pointer duration-[250ms] transition-colors`}
          style={{
            ...styles.uploadButtonContainer,
            background:
              fileName.length > 0 && fileSize > 0 ? "#008D01" : "white",
          }}
        >
          <div
            onClick={() => fileRef.current.click()}
            className="cursor-pointer w-16 h-9 flex justify-center items-center"
          >
            <UploadIcon
              fillColor={
                fileName.length > 0 && fileSize > 0 ? "white" : "#008D01"
              }
            />
          </div>

          {/* file details */}
          <div className="tooltip-details">
            {fileName.length > 0 && fileSize > 0 ? (
              <div
                className="flex flex-col items-center text-sm text-white gap-2"
                title={fileName}
              >
                {fileName.length > 0 && <p>File Name: {fileName}</p>}
                {fileSize > 0 && (
                  <p>File Size: {(fileSize / MB).toFixed(4)} MB</p>
                )}
              </div>
            ) : (
              <p className="text-sm text-white">Upload a .onnx model</p>
            )}
          </div>

          <input
            type="file"
            accept=".onnx"
            onChange={handleUpload}
            ref={fileRef}
            style={{ display: "none" } /* Hide the default file input */}
            className="cursor-pointer"
            onClick={(e) => {
              e.target.value = null; // Reset the file input value
            }}
          />
        </div>
      ) : null}
    </>
  );
};

export default DLModelUploadButton;

/* styles */
const styles = {
  uploadButtonContainer: {
    display: "flex",
    flexDirection: "column",
    justifyContent: "center",
    alignItems: "center",
    borderRadius: "6px",
  },
};
// digit_classification;
const UploadIcon = ({ cssClass, fillColor }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 -960 960 960"
    width="28"
    height="28"
    fill={fillColor || "currentColor"}
    className={`${cssClass}`}
  >
    <path d="M440-320v-326L336-542l-56-58 200-200 200 200-56 58-104-104v326h-80ZM240-160q-33 0-56.5-23.5T160-240v-120h80v120h480v-120h80v120q0 33-23.5 56.5T720-160H240Z" />
    Add commentMore actions
  </svg>
);
