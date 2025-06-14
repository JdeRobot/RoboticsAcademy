import React from "react";
import "./../../styles/tailwind.css"; // Ensure Tailwind CSS is imported

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

const FileUploadComponent = () => {
  const [fileName, setFileName] = React.useState("");
  const [fileSize, setFileSize] = React.useState(0);
  const fileRef = React.useRef(null);

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

  return (
    <div style={styles.uploadButtonContainer}>
      <div
        onClick={() => fileRef.current.click()}
        className="cursor-pointer w-10 h-10 flex justify-center items-center bg-slate-500 hover:bg-slate-600 rounded-full shadow-lg hover:shadow-xl transition-shadow duration-300"
      >
        <UploadIcon />
      </div>

      <div
        className="flex flex-col items-center text-sm text-gray-500"
        title={fileName}
      >
        {fileName.length > 0 && (
          <p>
            {fileName.slice(0, 15)} {fileName.length >= 15 && `...`}
          </p>
        )}
        {fileSize > 0 && <p>{(fileSize / MB).toFixed(2)} MB</p>}
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
  );
};

export default FileUploadComponent;

/* styles */
const styles = {
  uploadButtonContainer: {
    position: "absolute",
    right: "5px",
    top: "150px",
    zIndex: "99",
    display: "flex",
    flexDirection: "column",
    justifyContent: "center",
    alignItems: "center",
    marginTop: "20px",
  },
};
// digit_classification;
const UploadIcon = ({ cssClass }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 -960 960 960"
    width="28"
    height="28"
    fill="#ffffff"
    className={`${cssClass}`}
  >
    <path d="M440-320v-326L336-542l-56-58 200-200 200 200-56 58-104-104v326h-80ZM240-160q-33 0-56.5-23.5T160-240v-120h80v120h480v-120h80v120q0 33-23.5 56.5T720-160H240Z" />
  </svg>
);
