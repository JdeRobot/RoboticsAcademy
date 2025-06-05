import React from "react";

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

const FileUploadComponent = () => {
    const handleUpload = async (event) => {
        const file = event.target.files[0];
        if (!file || !file.name.endsWith(".onnx")) {
            alert("Please upload a valid .onnx file");
            return;
        }

        const buffer = await file.arrayBuffer(); // Binary buffer

        window.RoboticsReactComponents.model = buffer;
        console.log("====================================");
        console.log("File uploaded:", file.name);
        console.log("File size:", (file.size / (1024 * 1024)).toFixed(2), "MB");
        console.log("File type:", file.type);
        console.log("====================================");
    };

    return (
        <div className='App'>
            <h2>Upload and Zip ONNX File</h2>
            <input type='file' accept='.onnx' onChange={handleUpload} />
        </div>
    );
};

export default FileUploadComponent;
