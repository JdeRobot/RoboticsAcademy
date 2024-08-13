import React, { useEffect, useRef } from 'react';

function Camera() {
  const videoRef = useRef(null);

  useEffect(() => {
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
      navigator.mediaDevices.getUserMedia({ video: true })
        .then(stream => {
          if (videoRef.current) {
            videoRef.current.srcObject = stream;
          }
        })
        .catch(err => console.log(err));
    }
  }, []);

  return (
    <div style={{display: "flex", width: "100%", height: "100%"}}>
      <video ref={videoRef} autoPlay></video>
    </div>
  );
}

export default Camera;
