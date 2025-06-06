import React from "react";

window.RoboticsReactComponents = window.RoboticsReactComponents || {};

const MB = 1024 * 1024;

const FileUploadComponent = () => {
    const [fileSize, setFileSize] = React.useState(0);
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

        setFileSize(file.size);
    };

    return (
        <div className='App'>
            <h2>Upload DL File (.onnx only)</h2>
            <input type='file' accept='.onnx' onChange={handleUpload} />
            <div>
                {fileSize > 0 && (
                    <p>File size: {(fileSize / MB).toFixed(2)} MB</p>
                )}
            </div>
        </div>
    );
};

export default FileUploadComponent;
