import React, { useState, useRef } from "react";
import "./../../styles/tailwind.css";

const VideoUploader = ({ onVideoSelected }) => {
  const [isDragging, setIsDragging] = useState(false);
  const fileInputRef = useRef(null);

  // const handleVideoSelected = (file) => {
  //   console.log("file ", file);

  //   // setSelectedVideo(file);
  //   // const url = URL.createObjectURL(file);
  //   // setVideoUrl(url);
  // };
  const handleDragEnter = (e) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(true);
  };

  const handleDragLeave = () => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(false);
  };

  const handleDragOver = () => {
    e.preventDefault();
    e.stopPropagation();
    if (!isDragging) {
      setIsDragging(true);
    }
  };

  const handleDrop = () => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(false);

    if (e.dataTransfer.files && e.dataTransfer.files.length > 0) {
      const file = e.dataTransfer.files[0];
      if (file.type.startsWith("video/")) {
        onVideoSelected(file);
      } else {
        alert("Please upload a valid video file");
      }
    }
  };

  const handleFileChange = (e) => {
    if (e.target.files && e.target.files.length > 0) {
      const file = e.target.files[0];
      if (file.type.startsWith("video/")) {
        onVideoSelected(file);
      } else {
        alert("Please upload a valid video file");
      }
    }
  };

  const handleButtonClick = () => {
    if (fileInputRef.current) {
      fileInputRef.current.click();
    }
  };

  return (
    <div
      className={`w-full mx-auto border-2 border-dashed rounded-lg p-8 text-center transition-all duration-200 bg-red-600 ${
        isDragging
          ? "border-blue-500 bg-blue-50 scale-105"
          : "border-gray-300 hover:border-blue-400"
      }`}
      onDragEnter={handleDragEnter}
      onDragLeave={handleDragLeave}
      onDragOver={handleDragOver}
      onDrop={handleDrop}
    >
      <div className="flex flex-col items-center justify-center">
        <UploadIcon
          cssClass={`w-16 h-16 mb-4 transition-colors duration-200 ${
            isDragging ? "text-blue-500" : "text-gray-400"
          }`}
        />
        <h3 className="text-lg font-medium mb-2">Upload your video</h3>
        <p className="text-sm text-gray-500 mb-4">
          Drag and drop or click to browse
        </p>
        <button
          onClick={handleButtonClick}
          className="px-4 py-2 bg-blue-500 text-white rounded-md hover:bg-blue-600 transition-colors duration-200 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-opacity-50"
        >
          Select Video
        </button>
        <input
          type="file"
          ref={fileInputRef}
          onChange={handleFileChange}
          accept="video/*"
          className="hidden"
        />
        {/* <input
          type="file"
          accept="video/*"
          onChange={(e) => handleVideoSelected(e)}
        /> */}
      </div>
    </div>
  );
};

export default VideoUploader;

// Upload Icon
const UploadIcon = ({ cssClass }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    width="24"
    height="24"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    stroke-width="2"
    stroke-linecap="round"
    stroke-linejoin="round"
    class="lucide lucide-upload-icon lucide-upload"
    className={cssClass}
  >
    <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
    <polyline points="17 8 12 3 7 8" />
    <line x1="12" x2="12" y1="3" y2="15" />
  </svg>
);
