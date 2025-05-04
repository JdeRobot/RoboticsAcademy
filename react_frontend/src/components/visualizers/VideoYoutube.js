import React from 'react';

function VideoYoutube() {
  const videoId = '3n4Fdrz4LxQ'; // Puedes cambiarlo por otro

  return (
    <div style={{ textAlign: 'center', marginTop: '50px' }}>
      <h1>Reproductor de YouTube</h1>
      <div style={{ display: 'flex', justifyContent: 'center' }}>
        <iframe
          width="560"
          height="315"
          src={`https://www.youtube.com/embed/${videoId}`}
          title="YouTube video player"
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
          allowFullScreen
        ></iframe>
      </div>
    </div>
  );
}


export default VideoYoutube;
