import React, { useEffect, useState } from "react";
import "./../../styles/tailwind.css";

import VideoUploader from "../local_video/VideoUploader";
import VideoPlayer from "../local_video/VideoPlayer";
import { base64Data } from "./tmp_base64_data";
// import { FilmIcon } from "lucide-react";

const LocalVideo = () => {
  const [selectedVideo, setSelectedVideo] = useState(null);
  const [videoUrl, setVideoUrl] = useState("");

  const handleVideoSelected = (file) => {
    // console.log("file ", file);

    setSelectedVideo(file);
    const url = URL.createObjectURL(file);
    setVideoUrl(url);
  };

  // React.useEffect(() => {
  //   const callback = (message) => {
  //     console.log(message);

  //     // Send the ACK of the msg
  //     window.RoboticsExerciseComponents.commsManager.send(
  //       "gui",
  //       `pick${base64Data}xxxxxxxxxxxxxxxxxxxx`
  //     );
  //   };
  //   window.RoboticsExerciseComponents.commsManager.subscribe(
  //     [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
  //     callback
  //   );

  //   return () => {
  //     console.log("TestShowScreen unsubscribing from ['state-changed'] events");
  //     window.RoboticsExerciseComponents.commsManager.unsubscribe(
  //       [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
  //       callback
  //     );
  //   };
  // }, []);

  // useEffect(() => {
  //   const imageId = document.getElementById("gui_canvas");
  //   console.log("gui_canvas ", imageId);
  // }, []);

  return (
    <div className="xmin-h-screen bg-gray-50 flex flex-col items-center py-12 px-4">
      <div className="w-full max-w-4xl mx-auto space-y-8">
        {!selectedVideo ? (
          <div className="transition-all duration-300 ease-in-out">
            <VideoUploader onVideoSelected={handleVideoSelected} />
          </div>
        ) : (
          <div className="space-y-6 transition-all duration-300 ease-in-out">
            <VideoPlayer videoUrl={videoUrl} />

            <div className="flex justify-between items-center px-4">
              <div>
                <h3 className="text-lg font-medium text-gray-800">
                  {selectedVideo.name}
                </h3>
                <p className="text-sm text-gray-500">
                  {(selectedVideo.size / (1024 * 1024)).toFixed(2)} MB
                </p>
              </div>

              {/* <button
                onClick={() => {
                  setSelectedVideo(null);
                  setVideoUrl("");
                  URL.revokeObjectURL(videoUrl);
                }}
                className="px-4 py-2 bg-gray-200 text-gray-700 rounded-md hover:bg-gray-300 transition-colors duration-200 focus:outline-none focus:ring-2 focus:ring-gray-400 focus:ring-opacity-50"
              >
                Upload another video
              </button> */}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default LocalVideo;

// function LocalVideo() {
//   const [videoUrl, setVideoUrl] = useState(null);

//   const handleVideoChange = (event) => {
//     const file = event.target.files[0];
//     if (file) {
//       const url = URL.createObjectURL(file);
//       setVideoUrl(url);
//     }
//   };

//   return (
//     <div style={{ padding: "2rem", textAlign: "center" }}>
//       {/* <h2>Simple Video Loader</h2>

//       <input type="file" accept="video/*" onChange={handleVideoChange} />

//       {videoUrl && (
//         <div style={{ marginTop: "1rem" }}>
//           <video width="600" controls>
//             <source src={videoUrl} type="video/mp4" />
//             Your browser does not support the video tag.
//           </video>
//         </div>
//       )} */}
//       <VideoUploader />
//     </div>
//   );
// }

// export default LocalVideo;
